#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <thread>
#include <chrono>
#include <csignal>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace ur_rtde;
using namespace std::chrono;

// interrupt flag
bool running = true;
void raiseFlag(int param)
{
  running = false;
}

std::vector<double> getCircleTarget(const std::vector<double> &pose, double timestep, double radius=0.075, double freq=1.0)
{
  std::vector<double> circ_target = pose;
  circ_target[0] = pose[0] + radius * cos((2 * M_PI * freq * timestep));
  circ_target[1] = pose[1] + radius * sin((2 * M_PI * freq * timestep));
  return circ_target;
}

int main(int argc, char* argv[])
{
  // Setup parameters
  std::string robot_ip = "localhost";
  double rtde_frequency = 500.0; // Hz
  double dt = 1.0 / rtde_frequency; // 2ms
  uint16_t flags = RTDEControlInterface::FLAG_VERBOSE | RTDEControlInterface::FLAG_UPLOAD_SCRIPT;
  int ur_cap_port = 50002;

  // ur_rtde realtime priorities
  int rt_receive_priority = 90;
  int rt_control_priority = 85;

  RTDEControlInterface rtde_control(robot_ip, rtde_frequency, flags, ur_cap_port, rt_control_priority);
  RTDEReceiveInterface rtde_receive(robot_ip, rtde_frequency, {}, true, false, rt_receive_priority);

  // Set application realtime priority
  RTDEUtility::setRealtimePriority(80);

  // Move parameters
  double vel = 0.5;
  double acc = 0.5;

  // Servo control parameters
  double lookahead_time = 0.1;
  double gain = 600;

  signal(SIGINT, raiseFlag);

  double time_counter = 0.0;

  // Move to init position using moveL
  std::vector<double> actual_tcp_pose = rtde_receive.getActualTCPPose();
  std::vector<double> init_pose = getCircleTarget(actual_tcp_pose, time_counter);
  rtde_control.moveL(init_pose, vel, acc);

  try
  {
    while (running)
    {
      steady_clock::time_point t_start = rtde_control.initPeriod();
      std::vector<double> servo_target = getCircleTarget(actual_tcp_pose, time_counter);
      rtde_control.servoL(servo_target, vel, acc, dt, lookahead_time, gain);
      rtde_control.waitPeriod(t_start);
      time_counter += dt;
    }
    std::cout << "Control interrupted!" << std::endl;
    rtde_control.servoStop();
    rtde_control.stopScript();
  }
  catch(std::exception& e)
  {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  catch(...)
  {
    std::cerr << "Exception of unknown type!\n";
  }
  return 0;
}
