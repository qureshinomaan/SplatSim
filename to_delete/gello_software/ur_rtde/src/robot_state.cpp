#include <ur_rtde/robot_state.h>
#include <unordered_map>

namespace ur_rtde
{
std::unordered_map<std::string, rtde_type_variant_> RobotState::state_types_ {
    { "timestamp", double() },
    { "target_q", std::vector<double>() },
    { "target_qd", std::vector<double>() },
    { "target_qdd", std::vector<double>() },
    { "target_current", std::vector<double>() },
    { "target_moment", std::vector<double>() },
    { "actual_q", std::vector<double>() },
    { "actual_qd", std::vector<double>() },
    { "actual_qdd", std::vector<double>() },
    { "actual_current", std::vector<double>() },
    { "actual_moment", std::vector<double>() },
    { "joint_control_output", std::vector<double>() },
    { "actual_TCP_pose", std::vector<double>() },
    { "actual_TCP_speed", std::vector<double>() },
    { "actual_TCP_force", std::vector<double>() },
    { "target_TCP_pose", std::vector<double>() },
    { "target_TCP_speed", std::vector<double>() },
    { "actual_digital_input_bits", uint64_t() },
    { "joint_temperatures", std::vector<double>() },
    { "actual_execution_time", double() },
    { "robot_mode", int32_t() },
    { "joint_mode", std::vector<int32_t>() },
    { "safety_mode", int32_t() },
    { "actual_tool_accelerometer", std::vector<double>() },
    { "speed_scaling", double() },
    { "target_speed_fraction", double() },
    { "actual_momentum", double() },
    { "actual_main_voltage", double() },
    { "actual_robot_voltage", double() },
    { "actual_robot_current", double() },
    { "actual_joint_voltage", std::vector<double>() },
    { "actual_digital_output_bits", uint64_t() },
    { "runtime_state", uint32_t() },
    { "robot_status_bits", uint32_t() },
    { "safety_status_bits", uint32_t() },
    { "standard_analog_input0", double() },
    { "standard_analog_input1", double() },
    { "standard_analog_output0", double() },
    { "standard_analog_output1", double() },
    { "ft_raw_wrench", std::vector<double>() },
    { "payload", double() },
    { "payload_cog", std::vector<double>() },
    { "payload_inertia", std::vector<double>() },
    { "output_bit_registers0_to_31", uint32_t() },
    { "output_bit_registers32_to_63", uint32_t() },
    { "output_int_register_0", int32_t() },
    { "output_int_register_1", int32_t() },
    { "output_int_register_2", int32_t() },
    { "output_int_register_3", int32_t() },
    { "output_int_register_4", int32_t() },
    { "output_int_register_5", int32_t() },
    { "output_int_register_6", int32_t() },
    { "output_int_register_7", int32_t() },
    { "output_int_register_8", int32_t() },
    { "output_int_register_9", int32_t() },
    { "output_int_register_10", int32_t() },
    { "output_int_register_11", int32_t() },
    { "output_int_register_12", int32_t() },
    { "output_int_register_13", int32_t() },
    { "output_int_register_14", int32_t() },
    { "output_int_register_15", int32_t() },
    { "output_int_register_16", int32_t() },
    { "output_int_register_17", int32_t() },
    { "output_int_register_18", int32_t() },
    { "output_int_register_19", int32_t() },
    { "output_int_register_20", int32_t() },
    { "output_int_register_21", int32_t() },
    { "output_int_register_22", int32_t() },
    { "output_int_register_23", int32_t() },
    { "output_int_register_24", int32_t() },
    { "output_int_register_25", int32_t() },
    { "output_int_register_26", int32_t() },
    { "output_int_register_27", int32_t() },
    { "output_int_register_28", int32_t() },
    { "output_int_register_29", int32_t() },
    { "output_int_register_30", int32_t() },
    { "output_int_register_31", int32_t() },
    { "output_int_register_32", int32_t() },
    { "output_int_register_33", int32_t() },
    { "output_int_register_34", int32_t() },
    { "output_int_register_35", int32_t() },
    { "output_int_register_36", int32_t() },
    { "output_int_register_37", int32_t() },
    { "output_int_register_38", int32_t() },
    { "output_int_register_39", int32_t() },
    { "output_int_register_40", int32_t() },
    { "output_int_register_41", int32_t() },
    { "output_int_register_42", int32_t() },
    { "output_int_register_43", int32_t() },
    { "output_int_register_44", int32_t() },
    { "output_int_register_45", int32_t() },
    { "output_int_register_46", int32_t() },
    { "output_int_register_47", int32_t() },
    { "output_double_register_0", double() },
    { "output_double_register_1", double() },
    { "output_double_register_2", double() },
    { "output_double_register_3", double() },
    { "output_double_register_4", double() },
    { "output_double_register_5", double() },
    { "output_double_register_6", double() },
    { "output_double_register_7", double() },
    { "output_double_register_8", double() },
    { "output_double_register_9", double() },
    { "output_double_register_10", double() },
    { "output_double_register_11", double() },
    { "output_double_register_12", double() },
    { "output_double_register_13", double() },
    { "output_double_register_14", double() },
    { "output_double_register_15", double() },
    { "output_double_register_16", double() },
    { "output_double_register_17", double() },
    { "output_double_register_18", double() },
    { "output_double_register_19", double() },
    { "output_double_register_20", double() },
    { "output_double_register_21", double() },
    { "output_double_register_22", double() },
    { "output_double_register_23", double() },
    { "output_double_register_24", double() },
    { "output_double_register_25", double() },
    { "output_double_register_26", double() },
    { "output_double_register_27", double() },
    { "output_double_register_28", double() },
    { "output_double_register_29", double() },
    { "output_double_register_30", double() },
    { "output_double_register_31", double() },
    { "output_double_register_32", double() },
    { "output_double_register_33", double() },
    { "output_double_register_34", double() },
    { "output_double_register_35", double() },
    { "output_double_register_36", double() },
    { "output_double_register_37", double() },
    { "output_double_register_38", double() },
    { "output_double_register_39", double() },
    { "output_double_register_40", double() },
    { "output_double_register_41", double() },
    { "output_double_register_42", double() },
    { "output_double_register_43", double() },
    { "output_double_register_44", double() },
    { "output_double_register_45", double() },
    { "output_double_register_46", double() },
    { "output_double_register_47", double() }
};

RobotState::RobotState(const std::vector<std::string> &variables)
{
  initRobotState(variables);
}

RobotState::~RobotState() = default;

bool RobotState::lockUpdateStateMutex()
{
  update_state_mutex_.lock();
  return true;
}

bool RobotState::unlockUpdateStateMutex()
{
  update_state_mutex_.unlock();
  return true;
}

void RobotState::setFirstStateReceived(bool val)
{
  first_state_received_ = val;
}

bool RobotState::getFirstStateReceived()
{
  return first_state_received_;
}

void RobotState::initRobotState(const std::vector<std::string> &variables)
{
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  std::lock_guard<std::mutex> lock(update_state_mutex_);
#else
  std::lock_guard<PriorityInheritanceMutex> lock(update_state_mutex_);
#endif
  for (auto& item : variables)
  {
    if (state_types_.find(item) != state_types_.end())
    {
      rtde_type_variant_ entry = state_types_[item];
      state_data_[item] = entry;
    }
  }
  first_state_received_ = false;
}

}  // namespace ur_rtde
