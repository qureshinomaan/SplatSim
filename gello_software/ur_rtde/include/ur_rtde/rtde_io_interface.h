#pragma once
#ifndef RTDE_IO_INTERFACE_H
#define RTDE_IO_INTERFACE_H

#include <ur_rtde/rtde_export.h>
#include <ur_rtde/rtde.h>
#include <memory>
#include <string>

#define MAJOR_VERSION 0
#define CB3_MAJOR_VERSION 3
#define RT_PRIORITY_UNDEFINED 0

namespace ur_rtde
{
class RTDEIOInterface
{
 public:
  RTDE_EXPORT explicit RTDEIOInterface(std::string hostname, bool verbose = false,
                                       bool use_upper_range_registers = false, int rt_priority = RT_PRIORITY_UNDEFINED);

  RTDE_EXPORT virtual ~RTDEIOInterface();

  enum RobotStatus
  {
    ROBOT_STATUS_POWER_ON = 0,
    ROBOT_STATUS_PROGRAM_RUNNING = 1,
    ROBOT_STATUS_TEACH_BUTTON_PRESSED = 2,
    ROBOT_STATUS_POWER_BUTTON_PRESSED = 3
  };

  /**
    * @brief Can be used to disconnect the RTDE IO client.
   */
  RTDE_EXPORT void disconnect();

  /**
    * @brief Can be used to reconnect to the robot after a lost connection.
    */
  RTDE_EXPORT bool reconnect();

  /**
    * @brief Set standard digital output signal level
    * @param output_id The number (id) of the output, integer: [0:7]
    * @param signal_level The signal level. (boolean)
    */
  RTDE_EXPORT bool setStandardDigitalOut(std::uint8_t output_id, bool signal_level);

  /**
    * @brief Set configurable digital output signal level
    * @param output_id The number (id) of the output, integer: [0:7]
    * @param signal_level The signal level. (boolean)
    */
  RTDE_EXPORT bool setConfigurableDigitalOut(std::uint8_t output_id, bool signal_level);

  /**
    * @brief Set tool digital output signal level
    * @param output_id The number (id) of the output, integer: [0:1]
    * @param signal_level The signal level. (boolean)
    */
  RTDE_EXPORT bool setToolDigitalOut(std::uint8_t output_id, bool signal_level);

  /**
    * @brief Set the speed slider on the controller
    * @param speed set the speed slider on the controller as a fraction value between 0 and 1 (1 is 100%)
    */
  RTDE_EXPORT bool setSpeedSlider(double speed);

  /**
    * @brief Set Analog output voltage
    * @param output_id The number (id) of the output, integer: [0:1]
    * @param voltage_ratio voltage set as a (ratio) of the voltage span [0..1], 1 means full voltage.
    */
  RTDE_EXPORT bool setAnalogOutputVoltage(std::uint8_t output_id, double voltage_ratio);

  /**
    * @brief Set Analog output current
    * @param output_id The number (id) of the output, integer: [0:1]
    * @param current_ratio current set as a (ratio) of the current span [0..1], 1 means full current.
    */
  RTDE_EXPORT bool setAnalogOutputCurrent(std::uint8_t output_id, double current_ratio);

  /**
   * @brief Set the specified input integer register in either lower range [18-22] or upper range [42-46].
   *
   * @param input_id the id of the register to set, current supported range is: [18-22] or [42-46], this can
   * be adjusted by changing the RTDEControlInterface input recipes and by using the use_upper_range_registers
   * constructor flag to switch between lower and upper range.
   * @param value the desired integer value
   *
   * @returns true if the register is successfully set, false otherwise.
   */
  RTDE_EXPORT bool setInputIntRegister(int input_id, int value);

  /**
   * @brief Set the specified input double register in either lower range [18-22] or upper range [42-46].
   *
   * @param input_id the id of the register to set, current supported range is: [18-22] or [42-46], this can
   * be adjusted by changing the RTDEControlInterface input recipes and by using the use_upper_range_registers
   * constructor flag to switch between lower and upper range.
   * @param value the desired double value
   *
   * @returns true if the register is successfully set, false otherwise.
   */
  RTDE_EXPORT bool setInputDoubleRegister(int input_id, double value);

 private:
  bool setupRecipes();

  std::string inDoubleReg(int reg) const;

  std::string inIntReg(int reg) const;

  bool sendCommand(const RTDE::RobotCommand &cmd);

  void verifyValueIsWithin(const double &value, const double &min, const double &max);

 private:
  std::string hostname_;
  int port_;
  bool verbose_;
  bool use_upper_range_registers_;
  int rt_priority_;
  int register_offset_;
  std::shared_ptr<RTDE> rtde_;
};

}  // namespace ur_rtde

#endif  // RTDE_IO_INTERFACE_H
