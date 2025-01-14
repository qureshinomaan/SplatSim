#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_utility.h>

#include <boost/array.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/socket_base.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind/bind.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <cstring>
#include <iostream>
#include <memory>
#include <regex>

using boost::asio::ip::tcp;
using boost::lambda::var;
using namespace boost::placeholders;

namespace ur_rtde
{
DashboardClient::DashboardClient(std::string hostname, int port, bool verbose)
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

DashboardClient::~DashboardClient() = default;

void DashboardClient::connect(uint32_t timeout_ms)
{
  socket_.reset(new boost::asio::ip::tcp::socket(io_service_));
  socket_->open(boost::asio::ip::tcp::v4());
  boost::asio::ip::tcp::no_delay no_delay_option(true);
  boost::asio::socket_base::reuse_address sol_reuse_option(true);
  socket_->set_option(no_delay_option);
  socket_->set_option(sol_reuse_option);
  resolver_ = std::make_shared<tcp::resolver>(io_service_);
  tcp::resolver::query query(hostname_, std::to_string(port_));

  if (verbose_)
    std::cout << "Connecting to UR dashboard server..." << std::endl;
  deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
  boost::system::error_code ec = boost::asio::error::would_block;
  boost::asio::async_connect(*socket_, resolver_->resolve(query), var(ec) = boost::lambda::_1);
  do
  {
    io_service_.run_one();
  } while (ec == boost::asio::error::would_block);
  if (ec || !socket_->is_open())
  {
    throw std::runtime_error("Timeout connecting to UR dashboard server.");
  }
  conn_state_ = ConnectionState::CONNECTED;
  receive();
  if (verbose_)
    std::cout << "Connected successfully to UR dashboard server: " << hostname_ << " at " << port_ << std::endl;
}

bool DashboardClient::isConnected()
{
  return conn_state_ == ConnectionState::CONNECTED;
}

void DashboardClient::disconnect()
{
  /* We use reset() to safely close the socket,
   * see: https://stackoverflow.com/questions/3062803/how-do-i-cleanly-reconnect-a-boostsocket-following-a-disconnect
   */
  socket_.reset();
  conn_state_ = ConnectionState::DISCONNECTED;
  if (verbose_)
    std::cout << "Dashboard Client - Socket disconnected" << std::endl;
}

void DashboardClient::send(const std::string &str)
{
  boost::asio::write(*socket_, boost::asio::buffer(str));
}

void DashboardClient::loadURP(const std::string &urp_name)
{
  std::string load_urp = "load " + urp_name + "\n";
  send(load_urp);
  auto result = receive();
  if (strstr(result.c_str(), "Loading program:") == nullptr)
  {
    throw std::runtime_error(result);
  }
}

void DashboardClient::play()
{
  std::string play = "play\n";
  send(play);
  auto result = receive();
  if (result != "Starting program")
  {
    throw std::runtime_error(result);
  }
}

void DashboardClient::stop()
{
  std::string stop = "stop\n";
  send(stop);
  auto result = receive();
  if (result != "Stopped")
  {
    throw std::runtime_error(result);
  }
}

void DashboardClient::pause()
{
  std::string pause = "pause\n";
  send(pause);
  auto result = receive();
  if (result != "Pausing program")
  {
    throw std::runtime_error(result);
  }
}

void DashboardClient::quit()
{
  std::string quit = "quit\n";
  send(quit);
  receive();
}
void DashboardClient::shutdown()
{
  std::string shutdown = "shutdown\n";
  send(shutdown);
  receive();
}

bool DashboardClient::running()
{
  std::string message = "running\n";
  send(message);
  auto str = receive();
  std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::tolower(c); });
  if (strstr(str.c_str(), "true") != nullptr)
    return true;
  return false;
}

void DashboardClient::popup(const std::string &message)
{
  std::string popup = "popup " + message + "\n";
  send(popup);
  receive();
}

void DashboardClient::closePopup()
{
  std::string close_popup = "close popup\n";
  send(close_popup);
  receive();
}

std::string DashboardClient::polyscopeVersion()
{
  std::string polyscope_version = "PolyscopeVersion\n";
  send(polyscope_version);
  auto str = receive();
  const std::regex base_regex("\\d+.\\d+.\\d+.\\d+");
  std::smatch base_match;
  std::regex_search(str, base_match, base_regex);
  if (!base_match.empty())
    return std::string(base_match[0]);
  else
    return str;
}

std::string DashboardClient::programState()
{
  std::string program_state = "programState\n";
  send(program_state);
  auto state_str = receive();
  return state_str;
}

void DashboardClient::powerOn()
{
  std::string power_on = "power on\n";
  send(power_on);
  receive();
}

void DashboardClient::powerOff()
{
  std::string power_off = "power off\n";
  send(power_off);
  receive();
}

void DashboardClient::brakeRelease()
{
  std::string brake_release = "brake release\n";
  send(brake_release);
  receive();
}

void DashboardClient::unlockProtectiveStop()
{
  std::string unlock_p_stop = "unlock protective stop\n";
  send(unlock_p_stop);
  auto result = receive();
  if (result != "Protective stop releasing")
  {
    throw std::logic_error("Unlock protective stop failure: " + result);
  }
}

template <typename AsyncReadStream>
std::string DashboardClient::async_readline(AsyncReadStream &s, int timeout_ms)
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
  boost::system::error_code ec = boost::asio::error::would_block;

  // Start the asynchronous operation itself. The boost::lambda function
  // object is used as a callback and will update the ec variable when the
  // operation completes.
  // boost::asio::async_read(s, buffers, boost::lambda::var(ec) = boost::lambda::_1);
  boost::asio::async_read_until(s, input_buffer_, '\n', var(ec) = boost::lambda::_1);

  // Block until the asynchronous operation has completed.
  do
    io_service_.run_one();
  while (ec == boost::asio::error::would_block);
  if (ec)
  {
    throw boost::system::system_error(ec);
  }

  std::string line;
  std::istream is(&input_buffer_);
  std::getline(is, line);
  return line;
}

std::string DashboardClient::receive()
{
  return async_readline(*socket_);
}

std::string DashboardClient::robotmode()
{
  std::string robotmode = "robotmode\n";
  send(robotmode);
  auto state_str = receive();
  return state_str;
}

std::string DashboardClient::getRobotModel()
{
  std::string robot_model = "get robot model\n";
  send(robot_model);
  auto state_str = receive();
  return state_str;
}

std::string DashboardClient::getLoadedProgram()
{
  std::string get_loaded_program = "get loaded program\n";
  send(get_loaded_program);
  auto state_str = receive();
  return state_str;
}

void DashboardClient::addToLog(const std::string &message)
{
  std::string add_to_lof = "addToLog " + message + "\n";
  send(add_to_lof);
  receive();
}

bool DashboardClient::isProgramSaved()
{
  std::string is_program_saved = "isProgramSaved\n";
  send(is_program_saved);
  auto str = receive();
  if (strstr(str.c_str(), "True") != nullptr)
    return true;
  return false;
}

bool DashboardClient::isInRemoteControl()
{
  PolyScopeVersion polyscope_version(polyscopeVersion());
  if (polyscope_version.major == 5 && polyscope_version.minor > 5)
  {
    std::string is_in_remote_control = "is in remote control\n";
    send(is_in_remote_control);
    auto str = receive();
    if (strstr(str.c_str(), "true") != nullptr)
      return true;
    return false;
  }
  else
  {
    std::cerr << "Warning! isInRemoteControl() function is not supported on the dashboard server for PolyScope "
                 "versions less than 5.6.0"
              << std::endl;
    return false;
  }
}

void DashboardClient::setUserRole(const UserRole &role)
{
  std::string message;
  switch (role)
  {
    case UserRole::LOCKED:
      message = "locked";
    case UserRole::PROGRAMMER:
      message = "programmer";
    case UserRole::OPERATOR:
      message = "operator";
    case UserRole::NONE:
      message = "none";
    case UserRole::RESTRICTED:
      message = "restricted";
  }
  send("setUserRole " + message + "\n");
  receive();
}

std::string DashboardClient::safetymode()
{
  std::string safetymode = "safetymode\n";
  send(safetymode);
  return receive();
}

std::string DashboardClient::safetystatus()
{
  std::string safetystatus = "safetystatus\n";
  send(safetystatus);
  return receive();
}

void DashboardClient::closeSafetyPopup()
{
  std::string str = "close safety popup\n";
  send(str);
  receive();
}

void DashboardClient::restartSafety()
{
  std::string str = "restart safety\n";
  send(str);
  receive();
}

std::string DashboardClient::getSerialNumber()
{
  PolyScopeVersion polyscope_version(polyscopeVersion());
  if (polyscope_version.major == 5 && polyscope_version.minor > 5)
  {
    std::string get_serial_str = "get serial number\n";
    send(get_serial_str);
    auto result_str = receive();
    if (ur_rtde::RTDEUtility::isNumber(result_str))
    {
      return result_str;
    }
    else
    {
      throw std::runtime_error("getSerialNumber() function did not return a number. The following was returned: " +
                               result_str);
    }
  }
  else
  {
    throw std::runtime_error(
        "getSerialNumber() function is not supported on the dashboard server for PolyScope "
        "versions less than 5.6.0");
  }
}

void DashboardClient::check_deadline()
{
  // Check whether the deadline has passed. We compare the deadline against
  // the current time since a new asynchronous operation may have moved the
  // deadline before this actor had a chance to run.
  if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
  {
    std::cout << "Dashboard client deadline expired" << std::endl;
    // The deadline has passed. The socket is closed so that any outstanding
    // asynchronous operations are cancelled. This allows the blocked
    // connect(), read_line() or write_line() functions to return.
    boost::system::error_code ignored_ec;
    socket_->close(ignored_ec);
    conn_state_ = ConnectionState::DISCONNECTED;

    // There is no longer an active deadline. The expiry is set to positive
    // infinity so that the actor takes no action until a new deadline is set.
    deadline_.expires_at(boost::posix_time::pos_infin);
  }

  // Put the actor back to sleep.
  deadline_.async_wait(boost::bind(&DashboardClient::check_deadline, this));
}

}  // namespace ur_rtde
