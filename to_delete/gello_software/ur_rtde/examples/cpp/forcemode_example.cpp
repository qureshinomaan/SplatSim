#include <ur_rtde/rtde_control_interface.h>
#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono; 

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("127.0.0.1");

  // Parameters
  std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
  std::vector<int> selection_vector = {0, 0, 1, 0, 0, 0};
  std::vector<double> wrench_down = {0, 0, -10, 0, 0, 0};
  std::vector<double> wrench_up = {0, 0, 10, 0, 0, 0};
  int force_type = 2;
  std::vector<double> limits = {2, 2, 1.5, 1, 1, 1};
  std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

  // Move to initial joint position with a regular moveJ
  rtde_control.moveJ(joint_q);

  // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<2000; i++)
  {
    steady_clock::time_point t_start = rtde_control.initPeriod();
    // First we move the robot down for 2 seconds, then up for 2 seconds
    if (i > 1000)
      rtde_control.forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
    else
      rtde_control.forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
    rtde_control.waitPeriod(t_start);
  }

  rtde_control.forceModeStop();
  rtde_control.stopScript();

  return 0;
}
