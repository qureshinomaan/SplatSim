#include <ur_rtde/rtde_control_interface.h>
#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono; 

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("127.0.0.1");

  // Parameters
  double acceleration = 0.5;
  double dt = 1.0/500; // 2ms
  std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};
  std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Move to initial joint position with a regular moveJ
  rtde_control.moveJ(joint_q);

  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<1000; i++)
  {
    steady_clock::time_point t_start = rtde_control.initPeriod();
    rtde_control.speedJ(joint_speed, acceleration, dt);
    joint_speed[0] += 0.0005;
    joint_speed[1] += 0.0005;
    rtde_control.waitPeriod(t_start);
  }

  rtde_control.speedStop();
  rtde_control.stopScript();

  return 0;
}
