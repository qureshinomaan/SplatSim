#include <ur_rtde/dashboard_enums.h>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <regex>

namespace ur_rtde
{
SafetyStatus parseSafetyStatus(const std::string &state_str)
{
  if (strstr(state_str.c_str(), "NORMAL") != nullptr)
    return SafetyStatus::NORMAL;
  else if (strstr(state_str.c_str(), "REDUCED") != nullptr)
    return SafetyStatus::REDUCED;
  else if (strstr(state_str.c_str(), "PROTECTIVE_STOP") != nullptr)
    return SafetyStatus::PROTECTIVE_STOP;
  else if (strstr(state_str.c_str(), "RECOVERY") != nullptr)
    return SafetyStatus::RECOVERY;
  else if (strstr(state_str.c_str(), "SAFEGUARD_STOP") != nullptr)
    return SafetyStatus::SAFEGUARD_STOP;
  else if (strstr(state_str.c_str(), "SYSTEM_EMERGENCY_STOP") != nullptr)
    return SafetyStatus::SYSTEM_EMERGENCY_STOP;
  else if (strstr(state_str.c_str(), "ROBOT_EMERGENCY_STOP") != nullptr)
    return SafetyStatus::ROBOT_EMERGENCY_STOP;
  else if (strstr(state_str.c_str(), "VIOLATION") != nullptr)
    return SafetyStatus::VIOLATION;
  else if (strstr(state_str.c_str(), "FAULT") != nullptr)
    return SafetyStatus::FAULT;
  else if (strstr(state_str.c_str(), "AUTOMATIC_MODE_SAFEGUARD_STOP") != nullptr)
    return SafetyStatus::AUTOMATIC_MODE_SAFEGUARD_STOP;
  else if (strstr(state_str.c_str(), "SYSTEM_THREE_POSITION_ENABLING_STOP") != nullptr)
    return SafetyStatus::SYSTEM_THREE_POSITION_ENABLING_STOP;
  else
    throw std::runtime_error("Parsing Failed");
}

std::string toString(const SafetyStatus &mode)
{
  switch (mode)
  {
    case SafetyStatus::AUTOMATIC_MODE_SAFEGUARD_STOP:
      return "AUTOMATIC_MODE_SAFEGUARD_STOP";
    case SafetyStatus::NORMAL:
      return "NORMAL";
    case SafetyStatus::REDUCED:
      return "REDUCED";
    case SafetyStatus::PROTECTIVE_STOP:
      return "PROTECTIVE_STOP";
    case SafetyStatus::RECOVERY:
      return "RECOVERY";
    case SafetyStatus::SAFEGUARD_STOP:
      return "SAFEGUARD_STOP";
    case SafetyStatus::SYSTEM_EMERGENCY_STOP:
      return "SYSTEM_EMERGENCY_STOP";
    case SafetyStatus::ROBOT_EMERGENCY_STOP:
      return "ROBOT_EMERGENCY_STOP";
    case SafetyStatus::VIOLATION:
      return "VIOLATION";
    case SafetyStatus::FAULT:
      return "FAULT";
    case SafetyStatus::SYSTEM_THREE_POSITION_ENABLING_STOP:
      return "SYSTEM_THREE_POSITION_ENABLING_STOP";
  }
  return "NORMAL";
}

ProgramState parseProgramState(const std::string &state_str)
{
  if (strstr(state_str.c_str(), "STOPPED") != nullptr)
    return ProgramState::STOPPED;
  else if (strstr(state_str.c_str(), "PLAYING") != nullptr)
    return ProgramState::PLAYING;
  else if (strstr(state_str.c_str(), "PAUSED") != nullptr)
    return ProgramState::PAUSED;
  else
    throw std::runtime_error("Parsing Failed");
}

std::string toString(const ProgramState &mode)
{
  switch (mode)
  {
    case ProgramState::STOPPED:
      return "STOPPED";
    case ProgramState::PLAYING:
      return "PLAYING";
    case ProgramState::PAUSED:
      return "PAUSED";
  }
  return "STOPPED";
}

RobotMode parseRobotMode(const std::string &state_str)
{
  if (strstr(state_str.c_str(), "NO_CONTROLLER") != nullptr)
    return RobotMode::NO_CONTROLLER;
  else if (strstr(state_str.c_str(), "DISCONNECTED") != nullptr)
    return RobotMode::DISCONNECTED;
  else if (strstr(state_str.c_str(), "CONFIRM_SAFETY") != nullptr)
    return RobotMode::CONFIRM_SAFETY;
  else if (strstr(state_str.c_str(), "BOOTING") != nullptr)
    return RobotMode::BOOTING;
  else if (strstr(state_str.c_str(), "POWER_OFF") != nullptr)
    return RobotMode::POWER_OFF;
  else if (strstr(state_str.c_str(), "POWER_ON") != nullptr)
    return RobotMode::POWER_ON;
  else if (strstr(state_str.c_str(), "IDLE") != nullptr)
    return RobotMode::IDLE;
  else if (strstr(state_str.c_str(), "BACKDRIVE") != nullptr)
    return RobotMode::BACKDRIVE;
  else if (strstr(state_str.c_str(), "RUNNING") != nullptr)
    return RobotMode::RUNNING;
  throw std::runtime_error("Parsing Failed");
}

std::string toString(const RobotMode &mode)
{
  switch (mode)
  {
    case RobotMode::BACKDRIVE:
      return "BACKDRIVE";
    case RobotMode::NO_CONTROLLER:
      return "NO_CONTROLLER";
    case RobotMode::DISCONNECTED:
      return "DISCONNECTED";
    case RobotMode::CONFIRM_SAFETY:
      return "CONFIRM_SAFETY";
    case RobotMode::BOOTING:
      return "BOOTING";
    case RobotMode::POWER_OFF:
      return "POWER_OFF";
    case RobotMode::POWER_ON:
      return "POWER_ON";
    case RobotMode::IDLE:
      return "IDLE";
    case RobotMode::RUNNING:
      return "RUNNING";
  }
  return "RUNNING";
}

SafetyMode parseSafetyMode(const std::string &state_str)
{
  if (strstr(state_str.c_str(), "NORMAL") != nullptr)
    return SafetyMode::NORMAL;
  else if (strstr(state_str.c_str(), "REDUCED") != nullptr)
    return SafetyMode::REDUCED;
  else if (strstr(state_str.c_str(), "PROTECTIVE_STOP") != nullptr)
    return SafetyMode::PROTECTIVE_STOP;
  else if (strstr(state_str.c_str(), "RECOVERY") != nullptr)
    return SafetyMode::RECOVERY;
  else if (strstr(state_str.c_str(), "SAFEGUARD_STOP") != nullptr)
    return SafetyMode::SAFEGUARD_STOP;
  else if (strstr(state_str.c_str(), "SYSTEM_EMERGENCY_STOP") != nullptr)
    return SafetyMode::SYSTEM_EMERGENCY_STOP;
  else if (strstr(state_str.c_str(), "ROBOT_EMERGENCY_STOP") != nullptr)
    return SafetyMode::ROBOT_EMERGENCY_STOP;
  else if (strstr(state_str.c_str(), "VIOLATION") != nullptr)
    return SafetyMode::VIOLATION;
  else if (strstr(state_str.c_str(), "FAULT") != nullptr)
    return SafetyMode::FAULT;
  throw std::runtime_error("Parsing Failed");
}

std::string toString(const SafetyMode &mode)
{
  switch (mode)
  {
    case SafetyMode::FAULT:
      return "FAULT";
    case SafetyMode::NORMAL:
      return "NORMAL";
    case SafetyMode::REDUCED:
      return "REDUCED";
    case SafetyMode::PROTECTIVE_STOP:
      return "PROTECTIVE_STOP";
    case SafetyMode::RECOVERY:
      return "RECOVERY";
    case SafetyMode::SAFEGUARD_STOP:
      return "SAFEGUARD_STOP";
    case SafetyMode::SYSTEM_EMERGENCY_STOP:
      return "SYSTEM_EMERGENCY_STOP";
    case SafetyMode::ROBOT_EMERGENCY_STOP:
      return "ROBOT_EMERGENCY_STOP";
    case SafetyMode::VIOLATION:
      return "VIOLATION";
  }
  return "FAULT";
}

void PolyScopeVersion::parse(const std::string &str)
{
  std::string version;
  {
    const std::regex base_regex("\\d+.\\d+.\\d+.\\d+");
    std::smatch base_match;
    std::regex_search(str, base_match, base_regex);
    if (base_match.empty())
      throw std::runtime_error("PolyScopeVersion::parse failed");
    version = str;
  }

  const std::regex base_regex("\\d+");
  std::smatch base_match;

  if (std::regex_search(version, base_match, base_regex))
  {
    major = std::atoi(base_match[0].str().c_str());
    version = base_match.suffix();
  }
  else
  {
    throw std::runtime_error("PolyScopeVersion::parse failed");
  }

  if (std::regex_search(version, base_match, base_regex))
  {
    minor = std::atoi(base_match[0].str().c_str());
    version = base_match.suffix();
  }
  else
  {
    throw std::runtime_error("PolyScopeVersion::parse failed");
  }

  if (std::regex_search(version, base_match, base_regex))
  {
    patch = std::atoi(base_match[0].str().c_str());
    version = base_match.suffix();
  }
  else
  {
    throw std::runtime_error("PolyScopeVersion::parse failed");
  }

  if (std::regex_search(version, base_match, base_regex))
  {
    build = std::atoi(base_match[0].str().c_str());
    version = base_match.suffix();
  }
  else
  {
    throw std::runtime_error("PolyScopeVersion::parse failed");
  }
}

std::string PolyScopeVersion::toString()
{
  return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch) + "." +
         std::to_string(build);
}

}  // namespace ur_rtde