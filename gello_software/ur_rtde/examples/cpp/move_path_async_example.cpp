#include <ur_rtde/rtde_control_interface.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace ur_rtde;
using namespace std::chrono;

int main(int argc, char* argv[])
{
  std::string hostname = "127.0.0.1";
  RTDEControlInterface rtde_control(hostname);

  ur_rtde::Path path;
  double velocity = 0.5;
  double acceleration = 4;
  path.addEntry({PathEntry::MoveJ,
                 PathEntry::PositionTcpPose,
                 {-0.140, -0.400, 0.100, 0, 3.14, 0, velocity, acceleration,
                  0}});  // move to initial position using movej with inverse kinematics
  path.addEntry({PathEntry::MoveL,
                 PathEntry::PositionTcpPose,
                 {-0.140, -0.400, 0.300, 0, 3.14, 0, velocity, acceleration, 0.099}});
  path.addEntry({PathEntry::MoveL,
                 PathEntry::PositionTcpPose,
                 {-0.140, -0.600, 0.300, 0, 3.14, 0, velocity, acceleration, 0.099}});
  path.addEntry({PathEntry::MoveL,
                 PathEntry::PositionTcpPose,
                 {-0.140, -0.600, 0.100, 0, 3.14, 0, velocity, acceleration, 0.099}});
  path.addEntry({PathEntry::MoveL,
                 PathEntry::PositionTcpPose,
                 {-0.140, -0.400, 0.100, 0, 3.14, 0, velocity, acceleration, 0}});

  // First move given path synchronously
  std::cout << "Move path synchronously..." << std::endl;
  rtde_control.movePath(path, false);
  std::cout << "Path finished...\n\n" << std::endl;

  // Now move given path asynchronously
  std::cout << "Move path asynchronously with progress feedback..." << std::endl;
  rtde_control.movePath(path, true);
  // Wait for start of asynchronous operation
  while (rtde_control.getAsyncOperationProgress() < 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  std::cout << "Async path started.. " << std::endl;
  // Wait for end of asynchronous operation
  int waypoint = -1;
  while (rtde_control.getAsyncOperationProgress() >= 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int new_waypoint = rtde_control.getAsyncOperationProgress();
    if (new_waypoint != waypoint)
    {
      waypoint = new_waypoint;
      std::cout << "Moving to path waypoint " << waypoint << std::endl;
    }
  }
  std::cout << "Async path finished...\n\n" << std::endl;

  rtde_control.stopScript();
  return 0;
}
