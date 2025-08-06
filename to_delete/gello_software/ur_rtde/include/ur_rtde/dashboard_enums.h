#pragma once
#ifndef RTDE_DASHBOARD_ENUMS_H
#define RTDE_DASHBOARD_ENUMS_H
#include <iostream>

namespace ur_rtde
{
class PolyScopeVersion
{
 public:
  PolyScopeVersion(const std::string &str)
  {
    parse(str);
  }

  int major;
  int minor;
  int patch;
  int build;

  std::string toString();
  void parse(const std::string &str);
};

enum class ProgramState
{
  STOPPED,
  PLAYING,
  PAUSED
};

ProgramState parseProgramState(const std::string &state_str);
std::string toString(const ProgramState &mode);

enum class UserRole
{
  PROGRAMMER,
  OPERATOR,
  NONE,
  LOCKED,
  RESTRICTED
};
enum class SafetyMode
{
  NORMAL,
  REDUCED,
  PROTECTIVE_STOP,
  RECOVERY,
  SAFEGUARD_STOP,
  SYSTEM_EMERGENCY_STOP,
  ROBOT_EMERGENCY_STOP,
  VIOLATION,
  FAULT
};

SafetyMode parseSafetyMode(const std::string &state_str);
std::string toString(const SafetyMode &mode);

enum class SafetyStatus
{
  NORMAL,
  REDUCED,
  PROTECTIVE_STOP,
  RECOVERY,
  SAFEGUARD_STOP,
  SYSTEM_EMERGENCY_STOP,
  ROBOT_EMERGENCY_STOP,
  VIOLATION,
  FAULT,
  AUTOMATIC_MODE_SAFEGUARD_STOP,
  SYSTEM_THREE_POSITION_ENABLING_STOP
};

SafetyStatus parseSafetyStatus(const std::string &state_str);
std::string toString(const SafetyStatus &mode);

enum class RobotMode
{
  NO_CONTROLLER,
  DISCONNECTED,
  CONFIRM_SAFETY,
  BOOTING,
  POWER_OFF,
  POWER_ON,
  IDLE,
  BACKDRIVE,
  RUNNING
};

RobotMode parseRobotMode(const std::string &state_str);
std::string toString(const RobotMode &mode);

}  // namespace ur_rtde

#endif  // RTDE_DASHBOARD_ENUMS_H