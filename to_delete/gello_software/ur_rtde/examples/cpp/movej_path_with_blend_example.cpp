#include <ur_rtde/rtde_control_interface.h>

using namespace ur_rtde;

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("127.0.0.1");

  double velocity = 0.5;
  double acceleration = 0.5;
  double blend_1 = 0.0;
  double blend_2 = 0.02;
  double blend_3 = 0.0;
  std::vector<double> path_pose1 = {-0.143, -0.435, 0.20, -0.001, 3.12, 0.04, velocity, acceleration, blend_1};
  std::vector<double> path_pose2 = {-0.143, -0.51, 0.21, -0.001, 3.12, 0.04, velocity, acceleration, blend_2};
  std::vector<double> path_pose3 = {-0.32, -0.61, 0.31, -0.001, 3.12, 0.04, velocity, acceleration, blend_3};

  std::vector<std::vector<double>> path;
  path.push_back(path_pose1);
  path.push_back(path_pose2);
  path.push_back(path_pose3);

  // Send a linear path with blending in between - (currently uses separate script)
  rtde_control.moveL(path);
  rtde_control.stopScript();

  return 0;
}
