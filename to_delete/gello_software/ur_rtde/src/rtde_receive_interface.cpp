#include <ur_rtde/robot_state.h>
#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_utility.h>

#include <bitset>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <fstream>

namespace ur_rtde
{
RTDEReceiveInterface::RTDEReceiveInterface(std::string hostname, double frequency, std::vector<std::string> variables,
                                           bool verbose, bool use_upper_range_registers, int rt_priority)
    : hostname_(std::move(hostname)),
      frequency_(frequency),
      variables_(std::move(variables)),
      verbose_(verbose),
      use_upper_range_registers_(use_upper_range_registers),
      rt_priority_(rt_priority)
{
  // Check if realtime kernel is available and set realtime priority for the interface.
  if (RTDEUtility::isRealtimeKernelAvailable())
  {
    if (!RTDEUtility::setRealtimePriority(rt_priority_))
    {
      std::cerr << "RTDEReceiveInterface: Warning! Failed to set realtime priority even though a realtime kernel is "
                   "available." << std::endl;
    }
    else
    {
      if (verbose_)
      {
        std::cout << "RTDEReceiveInterface: realtime priority set successfully!" << std::endl;
      }
    }
  }
  else
  {
    if (verbose_)
    {
      std::cout << "RTDEReceiveInterface: realtime kernel not found, consider using a realtime kernel for better "
                   "performance."
                << std::endl;
    }
  }

  port_ = 30004;
  rtde_ = std::make_shared<RTDE>(hostname_, port_, verbose_);
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  auto controller_version = rtde_->getControllerVersion();
  uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

  if (frequency_ < 0) // frequency not specified, set it based on controller version.
  {
    frequency_ = 125;
    // If e-Series Robot set frequency to 500Hz
    if (major_version > CB3_MAJOR_VERSION)
      frequency_ = 500;
  }

  // Set delta time to be used by recordCallback
  delta_time_ = 1 / frequency_;

  // Init pausing state
  pausing_state_ = PausingState::RUNNING;
  pausing_ramp_up_increment_ = 0.01;

  if (use_upper_range_registers_)
    register_offset_ = 24;
  else
    register_offset_ = 0;

  // Setup recipes
  setupRecipes(frequency_);

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>(variables_);

  // Start RTDE data synchronization
  rtde_->sendStart();

  // Start executing receiveCallback
  th_ = std::make_shared<boost::thread>(boost::bind(&RTDEReceiveInterface::receiveCallback, this));

  // Wait until the first robot state has been received
  while (!robot_state_->getFirstStateReceived())
  {
    // Wait for first state to be fully received
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

RTDEReceiveInterface::~RTDEReceiveInterface()
{
  disconnect();
}

void RTDEReceiveInterface::disconnect()
{
  // Stop the receive callback function
  stop_receive_thread = true;
  th_->interrupt();
  th_->join();

  if (rtde_ != nullptr)
  {
    if (rtde_->isConnected())
      rtde_->disconnect();
  }

  // Wait until everything has disconnected
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

bool RTDEReceiveInterface::setupRecipes(const double& frequency)
{
  if (variables_.empty())
  {
    // Assume all variables
    variables_ = {"timestamp",
                  "target_q",
                  "target_qd",
                  "target_qdd",
                  "target_current",
                  "target_moment",
                  "actual_q",
                  "actual_qd",
                  "actual_current",
                  "joint_control_output",
                  "actual_TCP_pose",
                  "actual_TCP_speed",
                  "actual_TCP_force",
                  "target_TCP_pose",
                  "target_TCP_speed",
                  "actual_digital_input_bits",
                  "joint_temperatures",
                  "actual_execution_time",
                  "robot_mode",
                  "joint_mode",
                  "safety_mode",
                  "actual_tool_accelerometer",
                  "speed_scaling",
                  "target_speed_fraction",
                  "actual_momentum",
                  "actual_main_voltage",
                  "actual_robot_voltage",
                  "actual_robot_current",
                  "actual_joint_voltage",
                  "actual_digital_output_bits",
                  "runtime_state",
                  "standard_analog_input0",
                  "standard_analog_input1",
                  "standard_analog_output0",
                  "standard_analog_output1",
                  "robot_status_bits",
                  "safety_status_bits"};

    auto controller_version = rtde_->getControllerVersion();
    uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);
    uint32_t minor_version = std::get<MINOR_VERSION>(controller_version);
    uint32_t bugfix_version = std::get<BUGFIX_VERSION>(controller_version);

    // Some RTDE variables depends on a minimum PolyScope version, check is performed here
    if (major_version == 5 && minor_version >= 9)
      variables_.emplace_back("ft_raw_wrench");

    if ((major_version == 3 && minor_version >= 11) ||
        (major_version == 5 && minor_version >= 5 && bugfix_version >= 1))
    {
      variables_.emplace_back("payload");
      variables_.emplace_back("payload_cog");
    }

    if ((major_version == 3 && minor_version >= 15) || (major_version == 5 && minor_version >= 11))
      variables_.emplace_back("payload_inertia");

    if (use_upper_range_registers_)
    {
      if ((major_version == 3 && minor_version >= 9) || (major_version == 5 && minor_version >= 3))
      {
        variables_.emplace_back(outIntReg(2));
        for (int i = 12; i <= 19; i++)
          variables_.emplace_back(outIntReg(i));
        for (int i = 12; i <= 19; i++)
          variables_.emplace_back(outDoubleReg(i));
      }
      else
      {
        std::cerr << "Warning! The upper range of the double output registers are only available on PolyScope versions "
                     ">3.9 or >5.3"
                  << std::endl;
      }
    }
    else
    {
      if (major_version >= 3 && minor_version >= 4)
      {
        variables_.emplace_back(outIntReg(2));
        for (int i = 12; i <= 19; i++)
          variables_.emplace_back(outIntReg(i));
        for (int i = 12; i <= 19; i++)
          variables_.emplace_back(outDoubleReg(i));
      }
      else
      {
        std::cerr
            << "Warning! The lower range of the double output registers are only available on PolyScope versions >3.4"
            << std::endl;
      }
    }
  }

  // Setup output
  rtde_->sendOutputSetup(variables_, frequency);
  return true;
}

void RTDEReceiveInterface::receiveCallback()
{
  while (!stop_receive_thread)
  {
    // Receive and update the robot state
    try
    {
      if (rtde_->isDataAvailable())
      {
        no_bytes_avail_cnt_ = 0;
        boost::system::error_code ec = rtde_->receiveData(robot_state_);
        if(ec)
        {
          if(ec == boost::asio::error::eof)
          {
            std::cerr << "RTDEReceiveInterface: Robot closed the connection!" << std::endl;
          }
          throw std::system_error(ec);
        }
      }
      else
      {
        // Register that data was not available in this cycle
        no_bytes_avail_cnt_++;
        // If at least 2ms has passed without data available, try to read data again to detect de-synchronization.
        if (no_bytes_avail_cnt_ > 20)
        {
          boost::system::error_code ec = rtde_->receiveData(robot_state_);
          if(ec)
          {
            if(ec == boost::asio::error::eof)
            {
              std::cerr << "RTDEReceiveInterface: Robot closed the connection!" << std::endl;
            }
            throw std::system_error(ec);
          }
          no_bytes_avail_cnt_ = 0;
        }

#if defined(__linux__) || defined(__APPLE__)
        // Data not available on socket yet, yield to other threads and sleep before trying to receive data again.
        std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
#endif
      }     
    }
    catch (std::exception& e)
    {
      std::cerr << "RTDEReceiveInterface Exception: " << e.what() << std::endl;
      if (rtde_->isConnected())
        rtde_->disconnect();
      stop_receive_thread = true;
      stop_record_thread = true;
    }
  }
}

bool RTDEReceiveInterface::reconnect()
{
  if (rtde_ != nullptr)
  {
    no_bytes_avail_cnt_ = 0;
    rtde_->connect();
    rtde_->negotiateProtocolVersion();
    auto controller_version = rtde_->getControllerVersion();
    uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

    frequency_ = 125;
    // If e-Series Robot set frequency to 500Hz
    if (major_version > CB3_MAJOR_VERSION)
      frequency_ = 500;

    // Set delta time to be used by recordCallback
    delta_time_ = 1 / frequency_;

    // Setup recipes
    setupRecipes(frequency_);

    // Init Robot state
    robot_state_ = std::make_shared<RobotState>(variables_);

    // Start RTDE data synchronization
    rtde_->sendStart();

    stop_receive_thread = false;
    stop_record_thread = false;

    // Start executing receiveCallback
    th_ = std::make_shared<boost::thread>(boost::bind(&RTDEReceiveInterface::receiveCallback, this));

    // Wait until the first robot state has been received
    while (!robot_state_->getFirstStateReceived())
    {
      // Wait for first state to be fully received
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  return RTDEReceiveInterface::isConnected();
}

std::chrono::steady_clock::time_point RTDEReceiveInterface::initPeriod()
{
  return std::chrono::steady_clock::now();
}

void RTDEReceiveInterface::waitPeriod(const std::chrono::steady_clock::time_point &t_cycle_start)
{
  RTDEUtility::waitPeriod(t_cycle_start, delta_time_);
}

bool RTDEReceiveInterface::startFileRecording(const std::string &filename, const std::vector<std::string> &variables)
{
  // Init file recording
  file_recording_ = std::make_shared<std::ofstream>(filename);
  *file_recording_ << std::fixed << std::setprecision(6);

  // Write header
  if (!variables.empty())
  {
    record_variables_ = variables;
  }
  else
  {
    record_variables_ = variables_;
  }

  for (size_t i=0; i < record_variables_.size() ; i++)
  {
      uint16_t entry_size = robot_state_->getStateEntrySize(record_variables_[i]);
      if (entry_size > 1)
      {
        for (int j = 0; j < entry_size; j++)
        {
          *file_recording_ << record_variables_[i] + '_' + std::to_string(j);
          if (i != record_variables_.size() - 1)  // No comma at the end of line
            *file_recording_ << ",";
          else
          {
            if (j != entry_size - 1)
              *file_recording_ << ",";
          }
        }
      }
      else
      {
        *file_recording_ << record_variables_[i];
        if (i != record_variables_.size() - 1)  // No comma at the end of line
          *file_recording_ << ",";
      }
  }
  // End the header line
  *file_recording_ << std::endl;

  // Start recorder thread
  record_thrd_ = std::make_shared<boost::thread>(boost::bind(&RTDEReceiveInterface::recordCallback, this));
  return true;
}

bool RTDEReceiveInterface::stopFileRecording()
{
  stop_record_thread = true;
  record_thrd_->join();

  // Close the file
  if (file_recording_ != nullptr)
    file_recording_->close();

  return true;
}

void RTDEReceiveInterface::recordCallback()
{
  while (!stop_record_thread)
  {
    auto t_start = std::chrono::steady_clock::now();
    for (size_t i=0; i < record_variables_.size() ; i++)
    {
      std::string entry_str = robot_state_->getStateEntryString(record_variables_[i]);
      *file_recording_ << entry_str;
      if (i != record_variables_.size() - 1)  // No comma at the end of line
        *file_recording_ << ",";
    }
    *file_recording_ << std::endl;  // End row
    waitPeriod(t_start);
  }
}

bool RTDEReceiveInterface::isConnected()
{
  return rtde_->isConnected();
}

double RTDEReceiveInterface::getTimestamp()
{
  double timestamp;
  if (robot_state_->getStateData("timestamp", timestamp))
    return timestamp;
  else
    throw std::runtime_error("unable to get state data for specified key: timestamp");
}

std::vector<double> RTDEReceiveInterface::getTargetQ()
{
  std::vector<double> target_q;
  if (robot_state_->getStateData("target_q", target_q))
    return target_q;
  else
    throw std::runtime_error("unable to get state data for specified key: target_q");
}

std::vector<double> RTDEReceiveInterface::getTargetQd()
{
  std::vector<double> target_qd;
  if (robot_state_->getStateData("target_qd", target_qd))
    return target_qd;
  else
    throw std::runtime_error("unable to get state data for specified key: target_qd");
}

std::vector<double> RTDEReceiveInterface::getTargetQdd()
{
  std::vector<double> target_qdd;
  if (robot_state_->getStateData("target_qdd", target_qdd))
    return target_qdd;
  else
    throw std::runtime_error("unable to get state data for specified key: target_qdd");
}

std::vector<double> RTDEReceiveInterface::getTargetCurrent()
{
  std::vector<double> target_current;
  if (robot_state_->getStateData("target_current", target_current))
    return target_current;
  else
    throw std::runtime_error("unable to get state data for specified key: target_current");
}

std::vector<double> RTDEReceiveInterface::getTargetMoment()
{
  std::vector<double> target_moment;
  if (robot_state_->getStateData("target_moment", target_moment))
    return target_moment;
  else
    throw std::runtime_error("unable to get state data for specified key: target_moment");
}

std::vector<double> RTDEReceiveInterface::getActualQ()
{
  std::vector<double> actual_q;
  if (robot_state_->getStateData("actual_q", actual_q))
    return actual_q;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_q");
}

std::vector<double> RTDEReceiveInterface::getActualQd()
{
  std::vector<double> actual_qd;
  if (robot_state_->getStateData("actual_qd", actual_qd))
    return actual_qd;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_qd");
}

std::vector<double> RTDEReceiveInterface::getActualCurrent()
{
  std::vector<double> actual_current;
  if (robot_state_->getStateData("actual_current", actual_current))
    return actual_current;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_current");
}

std::vector<double> RTDEReceiveInterface::getJointControlOutput()
{
  std::vector<double> joint_control_output;
  if (robot_state_->getStateData("joint_control_output", joint_control_output))
    return joint_control_output;
  else
    throw std::runtime_error("unable to get state data for specified key: joint_control_output");
}

std::vector<double> RTDEReceiveInterface::getActualTCPPose()
{
  std::vector<double> actual_tcp_pose;
  if (robot_state_->getStateData("actual_TCP_pose", actual_tcp_pose))
    return actual_tcp_pose;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_TCP_pose");
}

std::vector<double> RTDEReceiveInterface::getActualTCPSpeed()
{
  std::vector<double> actual_tcp_speed;
  if (robot_state_->getStateData("actual_TCP_speed", actual_tcp_speed))
    return actual_tcp_speed;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_TCP_speed");
}

std::vector<double> RTDEReceiveInterface::getActualTCPForce()
{
  std::vector<double> actual_tcp_force;
  if (robot_state_->getStateData("actual_TCP_force", actual_tcp_force))
    return actual_tcp_force;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_TCP_force");
}

std::vector<double> RTDEReceiveInterface::getTargetTCPPose()
{
  std::vector<double> target_tcp_pose;
  if (robot_state_->getStateData("target_TCP_pose", target_tcp_pose))
    return target_tcp_pose;
  else
    throw std::runtime_error("unable to get state data for specified key: target_TCP_pose");
}

std::vector<double> RTDEReceiveInterface::getTargetTCPSpeed()
{
  std::vector<double> target_tcp_speed;
  if (robot_state_->getStateData("target_TCP_speed", target_tcp_speed))
    return target_tcp_speed;
  else
    throw std::runtime_error("unable to get state data for specified key: target_TCP_speed");
}

uint64_t RTDEReceiveInterface::getActualDigitalInputBits()
{
  uint64_t actual_digital_input_bits;
  if (robot_state_->getStateData("actual_digital_input_bits", actual_digital_input_bits))
    return actual_digital_input_bits;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_digital_input_bits");
}

bool RTDEReceiveInterface::getDigitalInState(std::uint8_t input_id)
{
  uint64_t input_bits = getActualDigitalInputBits();
  std::bitset<std::numeric_limits<uint64_t>::digits> input_bitset(input_bits);
  return input_bitset.test(input_id);
}

std::vector<double> RTDEReceiveInterface::getJointTemperatures()
{
  std::vector<double> joint_temperatures;
  if (robot_state_->getStateData("joint_temperatures", joint_temperatures))
    return joint_temperatures;
  else
    throw std::runtime_error("unable to get state data for specified key: joint_temperatures");
}

double RTDEReceiveInterface::getActualExecutionTime()
{
  double actual_execution_time;
  if (robot_state_->getStateData("actual_execution_time", actual_execution_time))
    return actual_execution_time;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_execution_time");
}

int32_t RTDEReceiveInterface::getRobotMode()
{
  int32_t robot_mode;
  if (robot_state_->getStateData("robot_mode", robot_mode))
    return robot_mode;
  else
    throw std::runtime_error("unable to get state data for specified key: robot_mode");
}

uint32_t RTDEReceiveInterface::getRobotStatus()
{
  uint32_t robot_status;
  if (robot_state_->getStateData("robot_status_bits", robot_status))
    return robot_status;
  else
    throw std::runtime_error("unable to get state data for specified key: robot_status");
}

std::vector<int32_t> RTDEReceiveInterface::getJointMode()
{
  std::vector<int32_t> joint_mode;
  if (robot_state_->getStateData("joint_mode", joint_mode))
    return joint_mode;
  else
    throw std::runtime_error("unable to get state data for specified key: joint_mode");
}

int32_t RTDEReceiveInterface::getSafetyMode()
{
  int32_t safety_mode;
  if (robot_state_->getStateData("safety_mode", safety_mode))
    return safety_mode;
  else
    throw std::runtime_error("unable to get state data for specified key: safety_mode");
}

uint32_t RTDEReceiveInterface::getSafetyStatusBits()
{
  uint32_t safety_status_bits;
  if (robot_state_->getStateData("safety_status_bits", safety_status_bits))
    return safety_status_bits;
  else
    throw std::runtime_error("unable to get state data for specified key: safety_status_bits");
}

std::vector<double> RTDEReceiveInterface::getActualToolAccelerometer()
{
  std::vector<double> actual_tool_accelerometer;
  if (robot_state_->getStateData("actual_tool_accelerometer", actual_tool_accelerometer))
    return actual_tool_accelerometer;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_tool_accelerometer");
}

double RTDEReceiveInterface::getSpeedScaling()
{
  double speed_scaling;
  if (robot_state_->getStateData("speed_scaling", speed_scaling))
    return speed_scaling;
  else
    throw std::runtime_error("unable to get state data for specified key: speed_scaling");
}

double RTDEReceiveInterface::getTargetSpeedFraction()
{
  double target_speed_fraction;
  if (robot_state_->getStateData("target_speed_fraction", target_speed_fraction))
    return target_speed_fraction;
  else
    throw std::runtime_error("unable to get state data for specified key: target_speed_fraction");
}

double RTDEReceiveInterface::getActualMomentum()
{
  double actual_momentum;
  if (robot_state_->getStateData("actual_momentum", actual_momentum))
    return actual_momentum;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_momentum");
}

double RTDEReceiveInterface::getActualMainVoltage()
{
  double actual_main_voltage;
  if (robot_state_->getStateData("actual_main_voltage", actual_main_voltage))
    return actual_main_voltage;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_main_voltage");
}

double RTDEReceiveInterface::getActualRobotVoltage()
{
  double actual_robot_voltage;
  if (robot_state_->getStateData("actual_robot_voltage", actual_robot_voltage))
    return actual_robot_voltage;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_robot_voltage");
}

double RTDEReceiveInterface::getActualRobotCurrent()
{
  double actual_robot_current;
  if (robot_state_->getStateData("actual_robot_current", actual_robot_current))
    return actual_robot_current;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_robot_current");
}

std::vector<double> RTDEReceiveInterface::getActualJointVoltage()
{
  std::vector<double> actual_joint_voltage;
  if (robot_state_->getStateData("actual_joint_voltage", actual_joint_voltage))
    return actual_joint_voltage;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_joint_voltage");
}

uint64_t RTDEReceiveInterface::getActualDigitalOutputBits()
{
  uint64_t actual_digital_output_bits;
  if (robot_state_->getStateData("actual_digital_output_bits", actual_digital_output_bits))
    return actual_digital_output_bits;
  else
    throw std::runtime_error("unable to get state data for specified key: actual_digital_output_bits");
}

bool RTDEReceiveInterface::getDigitalOutState(std::uint8_t output_id)
{
  uint64_t output_bits = getActualDigitalOutputBits();
  std::bitset<std::numeric_limits<uint64_t>::digits> output_bitset(output_bits);
  return output_bitset.test(output_id);
}

uint32_t RTDEReceiveInterface::getRuntimeState()
{
  uint32_t runtime_state;
  if (robot_state_->getStateData("runtime_state", runtime_state))
    return runtime_state;
  else
    throw std::runtime_error("unable to get state data for specified key: runtime_state");
}

double RTDEReceiveInterface::getStandardAnalogInput0()
{
  double standard_analog_input0;
  if (robot_state_->getStateData("standard_analog_input0", standard_analog_input0))
    return standard_analog_input0;
  else
    throw std::runtime_error("unable to get state data for specified key: standard_analog_input_0");
}

double RTDEReceiveInterface::getStandardAnalogInput1()
{
  double standard_analog_input1;
  if (robot_state_->getStateData("standard_analog_input1", standard_analog_input1))
    return standard_analog_input1;
  else
    throw std::runtime_error("unable to get state data for specified key: standard_analog_input_1");
}

double RTDEReceiveInterface::getStandardAnalogOutput0()
{
  double standard_analog_output0;
  if (robot_state_->getStateData("standard_analog_output0", standard_analog_output0))
    return standard_analog_output0;
  else
    throw std::runtime_error("unable to get state data for specified key: standard_analog_output_0");
}

double RTDEReceiveInterface::getStandardAnalogOutput1()
{
  double standard_analog_output1;
  if (robot_state_->getStateData("standard_analog_output1", standard_analog_output1))
    return standard_analog_output1;
  else
    throw std::runtime_error("unable to get state data for specified key: standard_analog_output_1");
}

double RTDEReceiveInterface::getPayload()
{
  double payload;
  if (robot_state_->getStateData("payload", payload))
    return payload;
  else
    throw std::runtime_error("unable to get state data for specified key: payload");
}

std::vector<double> RTDEReceiveInterface::getPayloadCog()
{
  std::vector<double> payload_cog;
  if (robot_state_->getStateData("payload_cog", payload_cog))
    return payload_cog;
  else
    throw std::runtime_error("unable to get state data for specified key: payload_cog");
}

std::vector<double> RTDEReceiveInterface::getPayloadInertia()
{
  std::vector<double> payload_inertia;
  if (robot_state_->getStateData("payload_inertia", payload_inertia))
    return payload_inertia;
  else
    throw std::runtime_error("unable to get state data for specified key: payload_inertia");
}

bool RTDEReceiveInterface::isProtectiveStopped()
{
  if (robot_state_ != nullptr)
  {
    std::bitset<32> safety_status_bits(getSafetyStatusBits());
    return safety_status_bits.test(SafetyStatus::IS_PROTECTIVE_STOPPED);
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTDEReceiveInterface::isEmergencyStopped()
{
  if (robot_state_ != nullptr)
  {
    std::bitset<32> safety_status_bits(getSafetyStatusBits());
    return safety_status_bits.test(SafetyStatus::IS_EMERGENCY_STOPPED);
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

int RTDEReceiveInterface::getOutputIntRegister(int output_id)
{
  if (use_upper_range_registers_)
  {
    if (!isWithinBounds(output_id, 36, 43))
    {
      throw std::range_error(
          "The supported range of getOutputIntRegister() is [36-43], when using upper range, you specified: " +
          std::to_string(output_id));
    }
  }
  else
  {
    if (!isWithinBounds(output_id, 12, 19))
    {
      throw std::range_error(
          "The supported range of getOutputIntRegister() is [12-19], when using lower range you specified: " +
          std::to_string(output_id));
    }
  }
  std::string output_int_register_key = "output_int_register_" + std::to_string(output_id);
  int32_t output_int_register_val;
  if (robot_state_->getStateData(output_int_register_key, output_int_register_val))
    return output_int_register_val;
  else
    throw std::runtime_error("unable to get state data for specified key: "+output_int_register_key);
}

double RTDEReceiveInterface::getOutputDoubleRegister(int output_id)
{
  if (use_upper_range_registers_)
  {
    if (!isWithinBounds(output_id, 36, 43))
    {
      throw std::range_error(
          "The supported range of getOutputDoubleRegister() is [36-43], when using upper range, you specified: " +
          std::to_string(output_id));
    }
  }
  else
  {
    if (!isWithinBounds(output_id, 12, 19))
    {
      throw std::range_error(
          "The supported range of getOutputDoubleRegister() is [12-19], when using lower range you specified: " +
          std::to_string(output_id));
    }
  }

  std::string output_double_register_key = "output_double_register_" + std::to_string(output_id);
  double output_double_register_val;
  if (robot_state_->getStateData(output_double_register_key, output_double_register_val))
    return output_double_register_val;
  else
    throw std::runtime_error("unable to get state data for specified key: " + output_double_register_key);
}

std::vector<double> RTDEReceiveInterface::getFtRawWrench()
{
  std::vector<double> ft_raw_wrench;
  if (robot_state_->getStateData("ft_raw_wrench", ft_raw_wrench))
  {
    if (!ft_raw_wrench.empty())
      return ft_raw_wrench;
    else
      throw std::runtime_error("getFtRawWrench is only supported on PolyScope versions >= 5.9.0");
  }
  else
    throw std::runtime_error("unable to get state data for specified key: ft_raw_wrench");
}

double RTDEReceiveInterface::getSpeedScalingCombined()
{
  uint32_t runtime_state = getRuntimeState();

  if (runtime_state == RuntimeState::PAUSED)
  {
    pausing_state_ = PausingState::PAUSED;
  }
  else if (runtime_state == RuntimeState::PLAYING && pausing_state_ == PausingState::PAUSED)
  {
    speed_scaling_combined_ = 0.0;
    pausing_state_ = PausingState::RAMPUP;
  }

  if (pausing_state_ == PausingState::RAMPUP)
  {
    double speed_scaling_ramp = speed_scaling_combined_ + pausing_ramp_up_increment_;
    speed_scaling_combined_ = std::min(speed_scaling_ramp, getSpeedScaling() * getTargetSpeedFraction());

    if (speed_scaling_ramp > getSpeedScaling() * getTargetSpeedFraction())
    {
      pausing_state_ = PausingState::RUNNING;
    }
  }
  else if (runtime_state == RuntimeState::RESUMING)
  {
    speed_scaling_combined_ = 0.0;
  }
  else
  {
    speed_scaling_combined_ = getSpeedScaling() * getTargetSpeedFraction();
  }

  return speed_scaling_combined_;
}

}  // namespace ur_rtde
