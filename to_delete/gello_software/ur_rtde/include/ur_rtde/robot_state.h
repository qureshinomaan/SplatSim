#pragma once
#ifndef RTDE_ROBOT_STATE_H
#define RTDE_ROBOT_STATE_H

#include <ur_rtde/rtde_export.h>
#include <ur_rtde/rtde_utility.h>
#include <boost/variant.hpp>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <cstdint>
#include <iomanip>
#include <mutex>
#include <iterator>



namespace ur_rtde
{
using rtde_type_variant_ = boost::variant<uint32_t, uint64_t, int32_t, double, std::vector<double>,
    std::vector<int32_t>>;

class RobotState
{
 public:
  RTDE_EXPORT explicit RobotState(const std::vector<std::string> &variables);

  RTDE_EXPORT virtual ~RobotState();

  RTDE_EXPORT bool lockUpdateStateMutex();

  RTDE_EXPORT bool unlockUpdateStateMutex();

  RTDE_EXPORT void setFirstStateReceived(bool val);

  RTDE_EXPORT bool getFirstStateReceived();

  RTDE_EXPORT void initRobotState(const std::vector<std::string> &variables);

  uint16_t getStateEntrySize(const std::string& name)
  {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    std::lock_guard<std::mutex> lock(update_state_mutex_);
#else
    std::lock_guard<PriorityInheritanceMutex> lock(update_state_mutex_);
#endif
    if (state_data_.find(name) != state_data_.end())
    {
      uint16_t entry_size = boost::apply_visitor(RobotState::SizeVisitor(), state_data_[name]);
      return entry_size;
    }
    else
    {
      throw std::runtime_error("unable to get state entry size for specified key: " + name);
    }
  };

  std::string getStateEntryString(const std::string& name)
  {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    std::lock_guard<std::mutex> lock(update_state_mutex_);
#else
    std::lock_guard<PriorityInheritanceMutex> lock(update_state_mutex_);
#endif
    if (state_data_.find(name) != state_data_.end())
    {
      std::string entry_str = boost::apply_visitor(RobotState::StringVisitor(), state_data_[name]);
      return entry_str;
    }
    else
    {
      throw std::runtime_error("unable to get state entry as string for specified key: " + name);
    }
  };

  template <typename T> bool
  getStateData(const std::string& name, T& val)
  {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    std::lock_guard<std::mutex> lock(update_state_mutex_);
#else
    std::lock_guard<PriorityInheritanceMutex> lock(update_state_mutex_);
#endif
    if (state_data_.find(name) != state_data_.end())
    {
      val = boost::strict_get<T>(state_data_[name]);
    }
    else
    {
      return false;
    }
    return true;
  };

  template <typename T>
  bool setStateData(const std::string& name, T& val)
  {
    if (state_data_.find(name) != state_data_.end())
    {
      state_data_[name] = val;
    }
    else
    {
      return false;
    }
    return true;
  };

  struct StringVisitor : public boost::static_visitor<std::string>
  {
    std::string operator()(uint32_t int_val) const
    {
      std::stringstream ss;
      ss << int_val;
      return ss.str();
    }
    std::string operator()(uint64_t int_val) const
    {
      std::stringstream ss;
      ss << int_val;
      return ss.str();
    }
    std::string operator()(int32_t int_val) const
    {
      std::stringstream ss;
      ss << int_val;
      return ss.str();
    }
    std::string operator()(double double_val) const
    {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(6) << double_val;
      return ss.str();
    }
    std::string operator()(std::vector<double> vec_double_val) const
    {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(6);
      std::copy(vec_double_val.begin(),  std::prev(vec_double_val.end()), std::ostream_iterator<double>(ss, ","));
      ss << vec_double_val.back();
      return ss.str();
    }
    std::string operator()(std::vector<int32_t> vec_int_val) const
    {
      std::stringstream ss;
      std::copy(vec_int_val.begin(), std::prev(vec_int_val.end()), std::ostream_iterator<int32_t>(ss, ","));
      ss << vec_int_val.back();
      return ss.str();
    }
  };

  struct SizeVisitor : public boost::static_visitor<uint16_t>
  {
    uint16_t operator()(uint32_t int_val) const
    {
      return 1;
    }

    uint16_t operator()(uint64_t int_val) const
    {
      return 1;
    }

    uint16_t operator()(int32_t int_val) const
    {
      return 1;
    }

    uint16_t operator()(double double_val) const
    {
      return 1;
    }

    uint16_t operator()(std::vector<double> vec_double_val) const
    {
      return vec_double_val.size();
    }

    uint16_t operator()(std::vector<int32_t> vec_int_val) const
    {
      return vec_int_val.size();
    }
  };

 public:
  static std::unordered_map<std::string, rtde_type_variant_> state_types_;

 private:
  std::unordered_map<std::string, rtde_type_variant_> state_data_;
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  std::mutex update_state_mutex_;
#else
  PriorityInheritanceMutex update_state_mutex_;
#endif
  bool first_state_received_;
};

}  // namespace ur_rtde

#endif  // RTDE_ROBOT_STATE_H
