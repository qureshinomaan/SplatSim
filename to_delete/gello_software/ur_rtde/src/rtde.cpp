#include <ur_rtde/robot_state.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_utility.h>

#include <boost/asio/connect.hpp>
#include <boost/asio/detail/socket_option.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/socket_base.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind/bind.hpp>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>

const unsigned HEADER_SIZE = 3;
#define RTDE_PROTOCOL_VERSION 2
#define DEBUG_OUTPUT false

#if DEBUG_OUTPUT
#define DEBUG(a)                                                \
  {                                                             \
    std::cout << "RTDE:" << __LINE__ << ": " << a << std::endl; \
  }
#else
#define DEBUG(a) \
  {              \
  }
#endif

using boost::asio::ip::tcp;
using namespace std::chrono;
using namespace boost::placeholders;

namespace ur_rtde
{
RTDE::RTDE(const std::string hostname, int port, bool verbose)
    : hostname_(std::move(hostname)),
      port_(port),
      verbose_(verbose),
      conn_state_(ConnectionState::DISCONNECTED),
      deadline_(io_service_)
{
  // No deadline is required until the first socket operation is started. We
  // set the deadline to positive infinity so that the actor takes no action
  // until a specific deadline is set.
  deadline_.expires_at(boost::posix_time::pos_infin);

  // Start the persistent actor that checks for deadline expiry.
  check_deadline();
}

RTDE::~RTDE() = default;

void RTDE::connect()
{
  try
  {
    buffer_.clear();  // ensure empty state in case of reconnect
    socket_.reset(new boost::asio::ip::tcp::socket(io_service_));
    socket_->open(boost::asio::ip::tcp::v4());
    boost::asio::ip::tcp::no_delay no_delay_option(true);
    boost::asio::socket_base::reuse_address sol_reuse_option(true);
    socket_->set_option(no_delay_option);
    socket_->set_option(sol_reuse_option);
#if defined(__linux) || defined(linux) || defined(__linux__)
    boost::asio::detail::socket_option::boolean<IPPROTO_TCP, TCP_QUICKACK> quickack(true);
    socket_->set_option(quickack);
#endif
    resolver_ = std::make_shared<boost::asio::ip::tcp::resolver>(io_service_);
    boost::asio::ip::tcp::resolver::query query(hostname_, std::to_string(port_));
    boost::asio::connect(*socket_, resolver_->resolve(query));
    conn_state_ = ConnectionState::CONNECTED;
    if (verbose_)
      std::cout << "Connected successfully to: " << hostname_ << " at " << port_ << std::endl;
  }
  catch (const boost::system::system_error &error)
  {
    std::cerr << error.what() << std::endl;
    std::string error_msg =
        "Error: Could not connect to: " + hostname_ + " at " + std::to_string(port_) + ", verify the IP";
    throw std::runtime_error(error_msg);
  }
}

void RTDE::disconnect()
{
  if (ConnectionState::CONNECTED == conn_state_)
  {
   sendPause();
  }
  /* We use reset() to safely close the socket,
   * see: https://stackoverflow.com/questions/3062803/how-do-i-cleanly-reconnect-a-boostsocket-following-a-disconnect
   */
  socket_.reset();
  conn_state_ = ConnectionState::DISCONNECTED;
  if (verbose_)
    std::cout << "RTDE - Socket disconnected" << std::endl;
}

bool RTDE::isConnected()
{
  return conn_state_ == ConnectionState::CONNECTED || conn_state_ == ConnectionState::STARTED;
}

bool RTDE::isStarted()
{
  return conn_state_ == ConnectionState::STARTED;
}

bool RTDE::negotiateProtocolVersion()
{
  std::uint8_t cmd = RTDE_REQUEST_PROTOCOL_VERSION;
  // Pack RTDE_PROTOCOL_VERSION into payload
  uint8_t null_byte = 0;
  uint8_t version = RTDE_PROTOCOL_VERSION;
  std::vector<char> buffer;
  buffer.push_back(null_byte);
  buffer.push_back(version);
  std::string payload(buffer.begin(), buffer.end());
  sendAll(cmd, payload);
  DEBUG("Done sending RTDE_REQUEST_PROTOCOL_VERSION");
  receive();
  return true;
}

bool RTDE::sendInputSetup(const std::vector<std::string> &input_names)
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_SETUP_INPUTS;
  // Concatenate input_names to a single string
  std::string input_names_str;
  for (const auto &input_name : input_names)
    input_names_str += input_name + ",";
  sendAll(cmd, input_names_str);
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_SETUP_INPUTS");
  receive();
  return true;
}

bool RTDE::sendOutputSetup(const std::vector<std::string> &output_names, double frequency)
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS;

  // First save the output_names for use in the receiveData function
  output_names_ = output_names;

  std::string freq_as_hexstr = RTDEUtility::double2hexstr(frequency);
  std::vector<char> freq_packed = RTDEUtility::hexToBytes(freq_as_hexstr);
  // Concatenate output_names to a single string
  std::string output_names_str;
  for (const auto &output_name : output_names)
    output_names_str += output_name + ",";

  std::copy(output_names_str.begin(), output_names_str.end(), std::back_inserter(freq_packed));
  std::string payload(std::begin(freq_packed), std::end(freq_packed));
  sendAll(cmd, payload);
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS");
  receive();
  return true;
}

void RTDE::send(const RobotCommand &robot_cmd)
{
  std::uint8_t command = RTDE_DATA_PACKAGE;
  std::vector<char> cmd_packed;
  cmd_packed = RTDEUtility::packInt32(robot_cmd.type_);

  if (robot_cmd.type_ == RobotCommand::FT_RTDE_INPUT_ENABLE ||
      robot_cmd.type_ == RobotCommand::ENABLE_EXTERNAL_FT_SENSOR)
  {
    std::vector<char> ft_rtde_input_enable_packed = RTDEUtility::packInt32(robot_cmd.ft_rtde_input_enable_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(ft_rtde_input_enable_packed.begin()),
                      std::make_move_iterator(ft_rtde_input_enable_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::FREEDRIVE_MODE)
  {
    std::vector<char> free_axes_packed = RTDEUtility::packVectorNInt32(robot_cmd.free_axes_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(free_axes_packed.begin()),
                      std::make_move_iterator(free_axes_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::SET_INPUT_INT_REGISTER)
  {
    std::vector<char> reg_int_packed = RTDEUtility::packInt32(robot_cmd.reg_int_val_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(reg_int_packed.begin()),
                      std::make_move_iterator(reg_int_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::SET_INPUT_DOUBLE_REGISTER)
  {
    std::vector<char> reg_double_packed = RTDEUtility::packDouble(robot_cmd.reg_double_val_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(reg_double_packed.begin()),
                      std::make_move_iterator(reg_double_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::WATCHDOG)
  {
    cmd_packed = RTDEUtility::packInt32(RobotCommand::NO_CMD);
  }

  if (robot_cmd.type_ == RobotCommand::FORCE_MODE)
  {
    std::vector<char> force_mode_type_packed = RTDEUtility::packInt32(robot_cmd.force_mode_type_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(force_mode_type_packed.begin()),
                      std::make_move_iterator(force_mode_type_packed.end()));

    std::vector<char> sel_vector_packed = RTDEUtility::packVectorNInt32(robot_cmd.selection_vector_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(sel_vector_packed.begin()),
                      std::make_move_iterator(sel_vector_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::GET_ACTUAL_JOINT_POSITIONS_HISTORY)
  {
    std::vector<char> actual_joint_positions_history_packed = RTDEUtility::packUInt32(robot_cmd.steps_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(actual_joint_positions_history_packed.begin()),
                      std::make_move_iterator(actual_joint_positions_history_packed.end()));
  }

  if (!robot_cmd.val_.empty())
  {
    std::vector<char> vector_nd_packed = RTDEUtility::packVectorNd(robot_cmd.val_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(vector_nd_packed.begin()),
                      std::make_move_iterator(vector_nd_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::MOVEJ || robot_cmd.type_ == RobotCommand::MOVEJ_IK ||
      robot_cmd.type_ == RobotCommand::MOVEL || robot_cmd.type_ == RobotCommand::MOVEL_FK ||
      robot_cmd.type_ == RobotCommand::MOVE_PATH || robot_cmd.type_ == RobotCommand::STOPJ ||
      robot_cmd.type_ == RobotCommand::STOPL)
  {
    std::vector<char> async_packed = RTDEUtility::packInt32(robot_cmd.async_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(async_packed.begin()),
                      std::make_move_iterator(async_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::SET_STD_DIGITAL_OUT)
  {
    cmd_packed.push_back(robot_cmd.std_digital_out_mask_);
    cmd_packed.push_back(robot_cmd.std_digital_out_);
  }

  if (robot_cmd.type_ == RobotCommand::SET_CONF_DIGITAL_OUT)
  {
    cmd_packed.push_back(robot_cmd.configurable_digital_out_mask_);
    cmd_packed.push_back(robot_cmd.configurable_digital_out_);
  }

  if (robot_cmd.type_ == RobotCommand::SET_TOOL_DIGITAL_OUT)
  {
    cmd_packed.push_back(robot_cmd.std_tool_out_mask_);
    cmd_packed.push_back(robot_cmd.std_tool_out_);
  }

  if (robot_cmd.type_ == RobotCommand::SET_SPEED_SLIDER)
  {
    std::vector<char> speed_slider_mask_packed = RTDEUtility::packInt32(robot_cmd.speed_slider_mask_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(speed_slider_mask_packed.begin()),
                      std::make_move_iterator(speed_slider_mask_packed.end()));

    std::vector<char> speed_slider_fraction_packed = RTDEUtility::packDouble(robot_cmd.speed_slider_fraction_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(speed_slider_fraction_packed.begin()),
                      std::make_move_iterator(speed_slider_fraction_packed.end()));
  }

  if (robot_cmd.type_ == RobotCommand::SET_STD_ANALOG_OUT)
  {
    cmd_packed.push_back(robot_cmd.std_analog_output_mask_);
    cmd_packed.push_back(robot_cmd.std_analog_output_type_);
    std::vector<char> std_analog_output_0_packed = RTDEUtility::packDouble(robot_cmd.std_analog_output_0_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_analog_output_0_packed.begin()),
                      std::make_move_iterator(std_analog_output_0_packed.end()));
    std::vector<char> std_analog_output_1_packed = RTDEUtility::packDouble(robot_cmd.std_analog_output_1_);
    cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_analog_output_1_packed.begin()),
                      std::make_move_iterator(std_analog_output_1_packed.end()));
  }

  cmd_packed.insert(cmd_packed.begin(), robot_cmd.recipe_id_);
  std::string sent(cmd_packed.begin(), cmd_packed.end());

  sendAll(command, sent);
  DEBUG("Done sending RTDE_DATA_PACKAGE");
}

void RTDE::sendAll(const std::uint8_t &command, std::string payload)
{
  DEBUG("Payload size is: " << payload.size());
  // Pack size and command into header
  uint16_t size = htons(HEADER_SIZE + (uint16_t)payload.size());
  uint8_t type = command;

  char buffer[3];
  memcpy(buffer + 0, &size, sizeof(size));
  memcpy(buffer + 2, &type, sizeof(type));

  // Create vector<char> that includes the header
  std::vector<char> header_packed;
  std::copy(buffer, buffer + sizeof(buffer), std::back_inserter(header_packed));

  // Add the payload to the header_packed vector
  std::copy(payload.begin(), payload.end(), std::back_inserter(header_packed));

  std::string sent(header_packed.begin(), header_packed.end());
  DEBUG("SENDING buf containing: " << sent << " with len: " << sent.size());

  // This is a workaround for the moment to prevent crash when calling this
  // function is RTDE is disconnected - i.e. in case of desynchronization
  if (isConnected())
  {
    boost::asio::write(*socket_, boost::asio::buffer(header_packed, header_packed.size()));
  }
}

void RTDE::sendStart()
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_START;
  sendAll(cmd, "");
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_START");
  receive();
}

void RTDE::sendPause()
{
  std::uint8_t cmd = RTDE_CONTROL_PACKAGE_PAUSE;
  sendAll(cmd, "");
  DEBUG("Done sending RTDE_CONTROL_PACKAGE_PAUSE");
  receive();
}

void RTDE::receive()
{
  DEBUG("Receiving...");
  // Read Header
  std::vector<char> data(HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));
  uint32_t message_offset = 0;
  uint16_t msg_size = RTDEUtility::getUInt16(data, message_offset);
  uint8_t msg_cmd = data.at(2);

  DEBUG("ControlHeader: ");
  DEBUG("size is: " << msg_size);
  DEBUG("command is: " << static_cast<int>(msg_cmd));

  // Read Body
  data.resize(msg_size - HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));

  switch (msg_cmd)
  {
    case RTDE_TEXT_MESSAGE:
    {
      uint8_t msg_length = data.at(0);
      for (int i = 1; i < msg_length; i++)
      {
        DEBUG(data[i]);
      }
      break;
    }

    case RTDE_REQUEST_PROTOCOL_VERSION:
    {
      break;
    }

    case RTDE_GET_URCONTROL_VERSION:
    {
      DEBUG("ControlVersion: ");
      // std::uint32_t message_offset = 0;
      // std::uint32_t v_major = RTDEUtility::getUInt32(data, message_offset);
      // std::uint32_t v_minor = RTDEUtility::getUInt32(data, message_offset);
      // std::uint32_t v_bugfix = RTDEUtility::getUInt32(data, message_offset);
      // std::uint32_t v_build = RTDEUtility::getUInt32(data, message_offset);
      // DEBUG(v_major << "." << v_minor << "." << v_bugfix << "." << v_build);
      break;
    }

    case RTDE_CONTROL_PACKAGE_SETUP_INPUTS:
    {
      // char id = data.at(0);
      // DEBUG("ID:" << (int)id);
      std::string datatypes(std::begin(data) + 1, std::end(data));
      DEBUG("Datatype:" << datatypes);
      std::string in_use_str("IN_USE");
      if (datatypes.find(in_use_str) != std::string::npos)
      {
        throw std::runtime_error(
            "One of the RTDE input registers are already in use! Currently you must disable the EtherNet/IP adapter, "
            "PROFINET or any MODBUS unit configured on the robot. This might change in the future.");
      }
      break;
    }

    case RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS:
    {
      // char id = data.at(0);
      // DEBUG("ID:" << id);
      std::string datatypes(std::begin(data) + 1, std::end(data));
      DEBUG("Datatype:" << datatypes);
      output_types_ = RTDEUtility::split(datatypes, ',');

      std::string not_found_str("NOT_FOUND");
      std::vector<int> not_found_indexes;
      if (datatypes.find(not_found_str) != std::string::npos)
      {
        for (unsigned int i = 0; i < output_types_.size(); i++)
        {
          if (output_types_[i] == "NOT_FOUND")
            not_found_indexes.push_back(i);
        }

        std::string vars_not_found;
        for (unsigned int i = 0; i < not_found_indexes.size(); i++)
        {
          vars_not_found += output_names_[not_found_indexes[i]];
          if (i != not_found_indexes.size() - 1)
            vars_not_found += ", ";
        }

        std::string error_str(
            "The following variables was not found by the controller: [" + vars_not_found +
            "],\n see which variables are supported by your PolyScope version here: \n"
            "https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/");
        throw std::runtime_error(error_str);
      }
      break;
    }

    case RTDE_CONTROL_PACKAGE_START:
    {
      char success = data.at(0);
      DEBUG("success: " << static_cast<bool>(success));
      auto rtde_success = static_cast<bool>(success);
      if (rtde_success)
      {
        conn_state_ = ConnectionState::STARTED;
        if (verbose_)
          std::cout << "RTDE synchronization started" << std::endl;
      }
      else
        std::cerr << "Unable to start synchronization" << std::endl;
      break;
    }

    case RTDE_CONTROL_PACKAGE_PAUSE:
    {
      char success = data.at(0);
      auto pause_success = static_cast<bool>(success);
      DEBUG("success: " << pause_success);
      if (pause_success)
      {
        conn_state_ = ConnectionState::PAUSED;
        DEBUG("RTDE synchronization paused!");
      }
      else
        std::cerr << "Unable to pause synchronization" << std::endl;
      break;
    }

      // TODO: Handle NOT_FOUND case

    default:
      DEBUG("Unknown Command: " << static_cast<int>(msg_cmd));
      break;
  }
}

template <typename AsyncReadStream, typename MutableBufferSequence>
std::size_t RTDE::async_read_some(AsyncReadStream &s, const MutableBufferSequence &buffers,
                                  boost::system::error_code &ec, int timeout_ms)
{
  if (timeout_ms < 0)
  {
    timeout_ms = 2500;
  }

  // Set a deadline for the asynchronous operation. Since this function uses
  // a composed operation (async_read_until), the deadline applies to the
  // entire operation, rather than individual reads from the socket.
  deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));

  // Set up the variable that receives the result of the asynchronous
  // operation. The error code is set to would_block to signal that the
  // operation is incomplete. Asio guarantees that its asynchronous
  // operations will never fail with would_block, so any other value in
  // ec indicates completion.
  ec = boost::asio::error::would_block;
  size_t bytes_received = 0;

  // Start the asynchronous operation itself. The boost::lambda function
  // object is used as a callback and will update the ec variable when the
  // operation completes.
  s.async_read_some(buffers,
                    [&](const boost::system::error_code &error, std::size_t bytes_transferred)
                    {
                      ec = error;
                      bytes_received = bytes_transferred;
                    });

  // Block until the asynchronous operation has completed.
  do
    io_service_.run_one();
  while (ec == boost::asio::error::would_block);
  if (ec)
  {
    throw boost::system::system_error(ec);
  }

  return bytes_received;
}

bool RTDE::isDataAvailable()
{
  if (socket_->available() > 0)
    return true;
  else
    return false;
}

boost::system::error_code RTDE::receiveData(std::shared_ptr<RobotState> &robot_state)
{
  boost::system::error_code error;
  uint32_t message_offset = 0;
  uint32_t packet_data_offset = 0;

  // Prepare buffer of 4096 bytes
  std::vector<char> data(4096);
  size_t data_len = 0;

  data_len = async_read_some(*socket_, boost::asio::buffer(data), error);
  if (error)
    return error;

  // Add data to the buffer
  buffer_.insert(buffer_.end(), data.begin(), data.begin() + data_len);

  while (buffer_.size() >= HEADER_SIZE)
  {
    message_offset = 0;
    // Read RTDEControlHeader
    RTDEControlHeader packet_header = RTDEUtility::readRTDEHeader(buffer_, message_offset);
    // std::cout << "RTDEControlHeader: " << std::endl;
    // std::cout << "size is: " << packet_header.msg_size << std::endl;
    // std::cout << "command is: " << static_cast<int>(packet_header.msg_cmd) << std::endl;

    if (buffer_.size() >= packet_header.msg_size)
    {
      // Read data package and adjust buffer
      std::vector<char> packet(buffer_.begin() + HEADER_SIZE, buffer_.begin() + packet_header.msg_size);
      buffer_.erase(buffer_.begin(), buffer_.begin() + packet_header.msg_size);

      if (buffer_.size() >= HEADER_SIZE && packet_header.msg_cmd == RTDE_DATA_PACKAGE)
      {
        RTDEControlHeader next_packet_header = RTDEUtility::readRTDEHeader(buffer_, message_offset);
        if (next_packet_header.msg_cmd == RTDE_DATA_PACKAGE)
        {
          if (verbose_)
            std::cout << "skipping package(1)" << std::endl;
          continue;
        }
      }

      if (packet_header.msg_cmd == RTDE_DATA_PACKAGE)
      {
        packet_data_offset = 0;
        RTDEUtility::getUChar(packet, packet_data_offset);

        robot_state->lockUpdateStateMutex();

        // Read all the variables specified by the user.
        for (const auto &output_name : output_names_)
        {
          if (robot_state->state_types_.find(output_name) != robot_state->state_types_.end())
          {
            rtde_type_variant_ entry = robot_state->state_types_[output_name];
            if (entry.type() == typeid(std::vector<double>))
            {
              std::vector<double> parsed_data;
              if (output_name == "actual_tool_accelerometer" || output_name == "payload_cog" ||
                  output_name == "elbow_position" || output_name == "elbow_velocity")
                parsed_data = RTDEUtility::unpackVector3d(packet, packet_data_offset);
              else
                parsed_data = RTDEUtility::unpackVector6d(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
            else if (entry.type() == typeid(double))
            {
              double parsed_data = RTDEUtility::getDouble(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
            else if (entry.type() == typeid(int32_t))
            {
              int32_t parsed_data = RTDEUtility::getInt32(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
            else if (entry.type() == typeid(uint32_t))
            {
              uint32_t parsed_data = RTDEUtility::getUInt32(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
            else if (entry.type() == typeid(uint64_t))
            {
              uint64_t parsed_data = RTDEUtility::getUInt64(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
            else if (entry.type() == typeid(std::vector<int32_t>))
            {
              std::vector<int32_t> parsed_data = RTDEUtility::unpackVector6Int32(packet, packet_data_offset);
              robot_state->setStateData(output_name, parsed_data);
            }
          }
          else
          {
            DEBUG("Unknown variable name: " << output_name << " please verify the output setup!");
          }
        }

        if (!robot_state->getFirstStateReceived())
          robot_state->setFirstStateReceived(true);

        robot_state->unlockUpdateStateMutex();
      }
      else
      {
        if (verbose_)
          std::cout << "skipping package(2)" << std::endl;
      }
    }
    else
    {
      break;
    }
  }
  return error;
}

std::tuple<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t> RTDE::getControllerVersion()
{
  std::uint8_t cmd = RTDE_GET_URCONTROL_VERSION;
  sendAll(cmd, "");
  DEBUG("Done sending RTDE_GET_URCONTROL_VERSION");
  std::vector<char> data(HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));
  uint32_t message_offset = 0;
  uint16_t msg_size = RTDEUtility::getUInt16(data, message_offset);
  uint8_t msg_cmd = data.at(2);
  // Read Body
  data.resize(msg_size - HEADER_SIZE);
  boost::asio::read(*socket_, boost::asio::buffer(data));

  if (msg_cmd == RTDE_GET_URCONTROL_VERSION)
  {
    message_offset = 0;
    std::uint32_t v_major = RTDEUtility::getUInt32(data, message_offset);
    std::uint32_t v_minor = RTDEUtility::getUInt32(data, message_offset);
    std::uint32_t v_bugfix = RTDEUtility::getUInt32(data, message_offset);
    std::uint32_t v_build = RTDEUtility::getUInt32(data, message_offset);
    DEBUG(v_major << "." << v_minor << "." << v_bugfix << "." << v_build);
    return std::make_tuple(v_major, v_minor, v_bugfix, v_build);
  }
  else
  {
    std::uint32_t v_major = 0;
    std::uint32_t v_minor = 0;
    std::uint32_t v_bugfix = 0;
    std::uint32_t v_build = 0;
    return std::make_tuple(v_major, v_minor, v_bugfix, v_build);
  }
}

void RTDE::check_deadline()
{
  // Check whether the deadline has passed. We compare the deadline against
  // the current time since a new asynchronous operation may have moved the
  // deadline before this actor had a chance to run.
  if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
  {
    // The deadline has passed. The socket is closed so that any outstanding
    // asynchronous operations are cancelled. This allows the blocked
    // connect(), read_line() or write_line() functions to return.
    boost::system::error_code ignored_ec;
    socket_->close(ignored_ec);
    conn_state_ = ConnectionState::DISCONNECTED;
    socket_.reset();

    // There is no longer an active deadline. The expiry is set to positive
    // infinity so that the actor takes no action until a new deadline is set.
    deadline_.expires_at(boost::posix_time::pos_infin);
  }

  // Put the actor back to sleep.
  deadline_.async_wait(boost::bind(&RTDE::check_deadline, this));
}

}  // namespace ur_rtde
