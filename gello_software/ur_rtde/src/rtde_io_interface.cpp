#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_utility.h>

#include <bitset>
#include <chrono>
#include <iostream>
#include <thread>

namespace ur_rtde
{
RTDEIOInterface::RTDEIOInterface(std::string hostname, bool verbose, bool use_upper_range_registers, int rt_priority)
    : hostname_(std::move(hostname)), verbose_(verbose), use_upper_range_registers_(use_upper_range_registers),
      rt_priority_(rt_priority)
{
  // Check if realtime kernel is available and set realtime priority for the interface.
  if (RTDEUtility::isRealtimeKernelAvailable())
  {
    if (!RTDEUtility::setRealtimePriority(rt_priority_))
    {
      std::cerr << "RTDEIOInterface: Warning! Failed to set realtime priority even though a realtime kernel is available." << std::endl;
    }
    else
    {
      if (verbose_)
      {
        std::cout << "RTDEIOInterface: realtime priority set successfully!" << std::endl;
      }
    }
  }
  else
  {
    if (verbose_)
    {
      std::cout << "RTDEIOInterface: realtime kernel not found, consider using a realtime kernel for better performance"
                << std::endl;
    }
  }

  port_ = 30004;
  rtde_ = std::make_shared<RTDE>(hostname_, port_, verbose_);
  rtde_->connect();
  rtde_->negotiateProtocolVersion();

  if(use_upper_range_registers_)
    register_offset_ = 24;
  else
    register_offset_ = 0;

  // Setup recipes
  setupRecipes();

  // Wait for connection to be fully established before returning
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

RTDEIOInterface::~RTDEIOInterface()
{
  if (rtde_ != nullptr)
  {
    if (rtde_->isConnected())
      rtde_->disconnect();
  }
}

void RTDEIOInterface::disconnect()
{
  if (rtde_ != nullptr)
  {
    if (rtde_->isConnected())
      rtde_->disconnect();
  }
}

bool RTDEIOInterface::reconnect()
{
  rtde_->connect();
  rtde_->negotiateProtocolVersion();

  // Setup recipes
  setupRecipes();

  // Wait for connection to be fully established before returning
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return true;
}

bool RTDEIOInterface::setupRecipes()
{
  // Recipe 1
  std::vector<std::string> no_cmd_input = {inIntReg(23)};
  rtde_->sendInputSetup(no_cmd_input);

  // Recipe 2
  std::vector<std::string> set_std_digital_out_input = {inIntReg(23), "standard_digital_output_mask",
                                                        "standard_digital_output"};
  rtde_->sendInputSetup(set_std_digital_out_input);

  // Recipe 3
  std::vector<std::string> set_tool_digital_out_input = {inIntReg(23), "tool_digital_output_mask",
                                                         "tool_digital_output"};
  rtde_->sendInputSetup(set_tool_digital_out_input);

  // Recipe 4
  std::vector<std::string> set_speed_slider = {inIntReg(23), "speed_slider_mask", "speed_slider_fraction"};
  rtde_->sendInputSetup(set_speed_slider);

  // Recipe 5
  std::vector<std::string> set_std_analog_output = {inIntReg(23), "standard_analog_output_mask",
                                                    "standard_analog_output_type", "standard_analog_output_0",
                                                    "standard_analog_output_1"};
  rtde_->sendInputSetup(set_std_analog_output);

  // Recipe 6
  std::vector<std::string> set_conf_digital_out_input = {inIntReg(23), "configurable_digital_output_mask",
                                                         "configurable_digital_output"};
  rtde_->sendInputSetup(set_conf_digital_out_input);

  // Recipe 7
  std::vector<std::string> set_input_int_reg_0_input = {inIntReg(23), inIntReg(18)};
  rtde_->sendInputSetup(set_input_int_reg_0_input);

  // Recipe 8
  std::vector<std::string> set_input_int_reg_1_input = {inIntReg(23), inIntReg(19)};
  rtde_->sendInputSetup(set_input_int_reg_1_input);

  // Recipe 9
  std::vector<std::string> set_input_int_reg_2_input = {inIntReg(23), inIntReg(20)};
  rtde_->sendInputSetup(set_input_int_reg_2_input);

  // Recipe 10
  std::vector<std::string> set_input_int_reg_3_input = {inIntReg(23), inIntReg(21)};
  rtde_->sendInputSetup(set_input_int_reg_3_input);

  // Recipe 11
  std::vector<std::string> set_input_int_reg_4_input = {inIntReg(23), inIntReg(22)};
  rtde_->sendInputSetup(set_input_int_reg_4_input);

  // Recipe 12
  std::vector<std::string> set_input_double_reg_0_input = {inIntReg(23), inDoubleReg(18)};
  rtde_->sendInputSetup(set_input_double_reg_0_input);

  // Recipe 13
  std::vector<std::string> set_input_double_reg_1_input = {inIntReg(23), inDoubleReg(19)};
  rtde_->sendInputSetup(set_input_double_reg_1_input);

  // Recipe 14
  std::vector<std::string> set_input_double_reg_2_input = {inIntReg(23), inDoubleReg(20)};
  rtde_->sendInputSetup(set_input_double_reg_2_input);

  // Recipe 15
  std::vector<std::string> set_input_double_reg_3_input = {inIntReg(23), inDoubleReg(21)};
  rtde_->sendInputSetup(set_input_double_reg_3_input);

  // Recipe 16
  std::vector<std::string> set_input_double_reg_4_input = {inIntReg(23), inDoubleReg(22)};
  rtde_->sendInputSetup(set_input_double_reg_4_input);
  return true;
}

void RTDEIOInterface::verifyValueIsWithin(const double &value, const double &min, const double &max)
{
  if (std::isnan(min) || std::isnan(max))
  {
    throw std::invalid_argument("Make sure both min and max are not NaN's");
  }
  else if (std::isnan(value))
  {
    throw std::invalid_argument("The value is considered NaN");
  }
  else if (!(std::isgreaterequal(value, min) && std::islessequal(value, max)))
  {
    std::ostringstream oss;
    oss << "The value is not within [" << min << ";" << max << "]";
    throw std::range_error(oss.str());
  }
}

bool RTDEIOInterface::setConfigurableDigitalOut(std::uint8_t output_id, bool signal_level)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_CONF_DIGITAL_OUT;
  robot_cmd.recipe_id_ = 6;

  if (signal_level)
  {
    robot_cmd.configurable_digital_out_mask_ = static_cast<uint8_t>(1u << output_id);
    robot_cmd.configurable_digital_out_ = static_cast<uint8_t>(1u << output_id);
  }
  else
  {
    robot_cmd.configurable_digital_out_mask_ = static_cast<uint8_t>(1u << output_id);
    robot_cmd.configurable_digital_out_ = 0;
  }

  return sendCommand(robot_cmd);
}

bool RTDEIOInterface::setStandardDigitalOut(std::uint8_t output_id, bool signal_level)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_STD_DIGITAL_OUT;
  robot_cmd.recipe_id_ = 2;

  if (signal_level)
  {
    robot_cmd.std_digital_out_mask_ = static_cast<uint8_t>(1u << output_id);
    robot_cmd.std_digital_out_ = static_cast<uint8_t>(1u << output_id);
  }
  else
  {
    robot_cmd.std_digital_out_mask_ = static_cast<uint8_t>(1u << output_id);
    robot_cmd.std_digital_out_ = 0;
  }

  return sendCommand(robot_cmd);
}

bool RTDEIOInterface::setToolDigitalOut(std::uint8_t output_id, bool signal_level)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_TOOL_DIGITAL_OUT;
  robot_cmd.recipe_id_ = 3;

  if (signal_level)
  {
    robot_cmd.std_tool_out_mask_ = static_cast<uint8_t>(1u << output_id);
    robot_cmd.std_tool_out_ = static_cast<uint8_t>(1u << output_id);
  }
  else
  {
    robot_cmd.std_tool_out_mask_ = static_cast<uint8_t>(1u << output_id);
    robot_cmd.std_tool_out_ = 0;
  }

  return sendCommand(robot_cmd);
}

bool RTDEIOInterface::setSpeedSlider(double speed)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_SPEED_SLIDER;
  robot_cmd.recipe_id_ = 4;
  robot_cmd.speed_slider_mask_ = 1;  // use speed_slider_fraction to set speed slider value
  robot_cmd.speed_slider_fraction_ = speed;
  return sendCommand(robot_cmd);
}

bool RTDEIOInterface::setAnalogOutputVoltage(std::uint8_t output_id, double voltage_ratio)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_STD_ANALOG_OUT;
  robot_cmd.recipe_id_ = 5;
  robot_cmd.std_analog_output_mask_ = static_cast<uint8_t>(1u << output_id);
  robot_cmd.std_analog_output_type_ = 1;  // set output type to voltage
  if (output_id == 0)
    robot_cmd.std_analog_output_0_ = voltage_ratio;
  else if (output_id == 1)
    robot_cmd.std_analog_output_1_ = voltage_ratio;
  return sendCommand(robot_cmd);
}

bool RTDEIOInterface::setAnalogOutputCurrent(std::uint8_t output_id, double current_ratio)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_STD_ANALOG_OUT;
  robot_cmd.recipe_id_ = 5;
  robot_cmd.std_analog_output_mask_ = static_cast<uint8_t>(1u << output_id);
  robot_cmd.std_analog_output_type_ = 0;  // set output type to current
  if (output_id == 0)
    robot_cmd.std_analog_output_0_ = current_ratio;
  else if (output_id == 1)
    robot_cmd.std_analog_output_1_ = current_ratio;
  return sendCommand(robot_cmd);
}

bool RTDEIOInterface::setInputIntRegister(int input_id, int value)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_INPUT_INT_REGISTER;
  if (use_upper_range_registers_)
  {
    if (input_id == 42)
      robot_cmd.recipe_id_ = 7;
    else if (input_id == 43)
      robot_cmd.recipe_id_ = 8;
    else if (input_id == 44)
      robot_cmd.recipe_id_ = 9;
    else if (input_id == 45)
      robot_cmd.recipe_id_ = 10;
    else if (input_id == 46)
      robot_cmd.recipe_id_ = 11;
    else
      throw std::range_error(
          "The supported range of setInputIntRegister() is [42-46], when using upper range, you specified: " +
          std::to_string(input_id));

    robot_cmd.reg_int_val_ = value;
    return sendCommand(robot_cmd);
  }
  else
  {
    if (input_id == 18)
      robot_cmd.recipe_id_ = 7;
    else if (input_id == 19)
      robot_cmd.recipe_id_ = 8;
    else if (input_id == 20)
      robot_cmd.recipe_id_ = 9;
    else if (input_id == 21)
      robot_cmd.recipe_id_ = 10;
    else if (input_id == 22)
      robot_cmd.recipe_id_ = 11;
    else
      throw std::range_error(
          "The supported range of setInputIntRegister() is [18-22], when using lower range, you specified: " +
          std::to_string(input_id));

    robot_cmd.reg_int_val_ = value;
    return sendCommand(robot_cmd);
  }
}

bool RTDEIOInterface::setInputDoubleRegister(int input_id, double value)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_INPUT_DOUBLE_REGISTER;
  if (use_upper_range_registers_)
  {
    if (input_id == 42)
      robot_cmd.recipe_id_ = 12;
    else if (input_id == 43)
      robot_cmd.recipe_id_ = 13;
    else if (input_id == 44)
      robot_cmd.recipe_id_ = 14;
    else if (input_id == 45)
      robot_cmd.recipe_id_ = 15;
    else if (input_id == 46)
      robot_cmd.recipe_id_ = 16;
    else
      throw std::range_error(
          "The supported range of setInputDoubleRegister() is [42-46], when using upper range, you specified: " +
          std::to_string(input_id));

    robot_cmd.reg_double_val_ = value;
    return sendCommand(robot_cmd);
  }
  else
  {
    if (input_id == 18)
      robot_cmd.recipe_id_ = 12;
    else if (input_id == 19)
      robot_cmd.recipe_id_ = 13;
    else if (input_id == 20)
      robot_cmd.recipe_id_ = 14;
    else if (input_id == 21)
      robot_cmd.recipe_id_ = 15;
    else if (input_id == 22)
      robot_cmd.recipe_id_ = 16;
    else
      throw std::range_error(
          "The supported range of setInputDoubleRegister() is [18-22], when using lower range, you specified: " +
          std::to_string(input_id));

    robot_cmd.reg_double_val_ = value;
    return sendCommand(robot_cmd);
  }
}

std::string RTDEIOInterface::inDoubleReg(int reg) const
{
  return "input_double_register_" + std::to_string(register_offset_+reg);
}

std::string RTDEIOInterface::inIntReg(int reg) const
{
  return "input_int_register_" + std::to_string(register_offset_+reg);
}

bool RTDEIOInterface::sendCommand(const RTDE::RobotCommand &cmd)
{
  try
  {
    // Send command to the controller
    rtde_->send(cmd);
    return true;
  }
  catch (std::exception &e)
  {
    std::cout << "RTDEIOInterface: Lost connection to robot..." << std::endl;
    std::cerr << e.what() << std::endl;
    if (rtde_ != nullptr)
    {
      if (rtde_->isConnected())
        rtde_->disconnect();
    }
  }

  if (!rtde_->isConnected())
  {
    std::cout << "RTDEIOInterface: Robot is disconnected, reconnecting..." << std::endl;
    reconnect();
    return sendCommand(cmd);
  }
  return false;
}

}  // namespace ur_rtde