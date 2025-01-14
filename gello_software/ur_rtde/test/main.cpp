#define DOCTEST_CONFIG_IMPLEMENT
#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <chrono>
#include <thread>
#include <vector>

#include "doctest.h"

using namespace ur_rtde;
using namespace std::chrono;

// Declare ur_rtde interfaces
std::shared_ptr<DashboardClient> db_client;
std::shared_ptr<RTDEControlInterface> rtde_control;
std::shared_ptr<RTDEReceiveInterface> rtde_receive;
std::shared_ptr<RTDEIOInterface> rtde_io;

// Declare initial values
std::vector<double> init_q;
std::vector<double> init_pose;

int main(int argc, char** argv)
{
  doctest::Context context;

  // Initialize doctest C++ testing framework
  context.setOption("abort-after", 5);  // stop test execution after 5 failed assertions
  context.applyCommandLine(argc, argv);
  context.setOption("no-breaks", true);  // don't break in the debugger when assertions fail

  // Power and brake release the robot through dashboard client
  db_client = std::make_shared<DashboardClient>("192.168.56.101", 29999, true);
  db_client->connect(5000);
  db_client->brakeRelease();

  // Wait for the brakes to release
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // Initialize RTDE
  rtde_control = std::make_shared<RTDEControlInterface>("192.168.56.101");
  rtde_receive = std::make_shared<RTDEReceiveInterface>("192.168.56.101");
  rtde_io = std::make_shared<RTDEIOInterface>("192.168.56.101");
  init_q = rtde_receive->getActualQ();
  init_pose = rtde_receive->getActualTCPPose();

  // Activate doctest C++ testing framework and run all test cases
  int res = context.run();  // run test cases unless with --no-run

  if (context.shouldExit())  // query flags (and --exit) rely on the user doing this
    return res;              // propagate the result of the tests

  return res;  // the result from doctest is propagated here as well

  // Stop the RTDE control script
  rtde_control->stopScript();
}

SCENARIO("Move robot in joint space (moveJ)")
{
  GIVEN("A target joint configuration")
  {
    // Move to initial pose, securing that former test don't leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    // Target is Pi / 6 in the robot base joint
    std::vector<double> target_q = init_q;
    target_q[0] += 0.5235;  // ~ Pi / 6

    WHEN("Robot is done moving")
    {
      REQUIRE(rtde_control->moveJ(target_q, 1.05, 1.4));

      THEN("Robot must be at target")
      {
        std::vector<double> actual_q = rtde_receive->getActualQ();

        for (unsigned int i = 0; i < actual_q.size(); i++)
        {
          REQUIRE(actual_q[i] == doctest::Approx(target_q[i]).epsilon(0.005));
        }
      }
    }
  }
}

SCENARIO("Move robot in tool space (moveL)")
{
  GIVEN("A cartesian target pose")
  {
    // Move to initial pose, securing that former test don't leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    // Target 10 cm up in the Z-Axis of the TCP
    std::vector<double> target_pose = init_pose;
    target_pose[2] += 0.10;

    WHEN("Robot is done moving")
    {
      REQUIRE(rtde_control->moveL(target_pose, 0.25, 0.5));

      THEN("Robot must be at target")
      {
        std::vector<double> actual_tcp_pose = rtde_receive->getActualTCPPose();

        for (unsigned int i = 0; i < actual_tcp_pose.size(); i++)
        {
          REQUIRE(actual_tcp_pose[i] == doctest::Approx(target_pose[i]).epsilon(0.005));
        }
      }
    }
  }
}

SCENARIO("Move robot in tool space using a predefined path")
{
  GIVEN("A cartesian target pose")
  {
    // Move to initial pose, securing that former test don't leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    // Target is defined in this vector
    std::vector<double> target_pose{0.280, -0.400, 0.100, 0, 3.14, 0};

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
                   {0.140, -0.600, 0.100, 0, 3.14, 0, velocity, acceleration, 0.099}});
    path.addEntry(
        {PathEntry::MoveL, PathEntry::PositionTcpPose, {0.280, -0.400, 0.100, 0, 3.14, 0, velocity, acceleration, 0}});

    WHEN("Robot is done moving")
    {
      REQUIRE(rtde_control->movePath(path, false));

      THEN("Robot must be at target")
      {
        std::vector<double> actual_tcp_pose = rtde_receive->getActualTCPPose();
        for (unsigned int i = 0; i < actual_tcp_pose.size(); i++)
        {
          REQUIRE(actual_tcp_pose[i] == doctest::Approx(target_pose[i]).epsilon(0.005));
        }
      }
    }
  }
}

SCENARIO("Move robot in Forcemode (forceMode)")
{
  GIVEN("Move robot in cartesian space using fixed force")
  {
    // Parameters
    std::vector<double> task_frame = {0, 0, 0, 0, 0, 0};
    std::vector<int> selection_vector = {0, 0, 1, 0, 0, 0};
    std::vector<double> wrench_down = {0, 0, -10, 0, 0, 0};
    std::vector<double> wrench_up = {0, 0, 10, 0, 0, 0};
    int force_type = 2;
    double dt = 1.0 / 500;  // 2ms
    std::vector<double> limits = {2, 2, 1.5, 1, 1, 1};
    std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

    // This target is defined specifically for the test
    std::vector<double> target_pose{-0.117025, -0.435893, 0.150533, -0.0124127, 3.11199, -0.0180484};

    // Move to initial pose, securing that former test don't leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    std::vector<double> start_pose = rtde_receive->getActualTCPPose();

    WHEN("Robot is still moving")
    {
      // Move to initial joint position with a regular moveJ
      REQUIRE(rtde_control->moveJ(joint_q));

      // Execute 500Hz control loop for a total of 4 seconds, each cycle is ~2ms
      for (unsigned int i = 0; i < 2000; i++)
      {
        auto t_start = rtde_control->initPeriod();
        // First we move the robot down for 2 seconds, then up for 2 seconds
        if (i > 1000)
          rtde_control->forceMode(task_frame, selection_vector, wrench_up, force_type, limits);
        else
          rtde_control->forceMode(task_frame, selection_vector, wrench_down, force_type, limits);
        rtde_control->waitPeriod(t_start);
      }
      rtde_control->forceModeStop();

      THEN("Robot must be at different place than at start")
      {
        std::vector<double> actual_tcp_pose = rtde_receive->getActualTCPPose();
        for (unsigned int i = 0; i < actual_tcp_pose.size(); i++)
        {
          CHECK(actual_tcp_pose[i] == doctest::Approx(target_pose[i]).epsilon(0.05));
        }
      }
    }
  }
}

SCENARIO("Move robot using MoveL Path With Blending")
{
  GIVEN("A cartesian target pose")
  {
    // Move to initial pose, securing that former test dont leave robot in a strange state.
    rtde_control->moveL(init_pose, 3, 3);

    // Parameters
    double velocity = 0.5;
    double acceleration = 0.5;
    double blend_1 = 0.0;
    double blend_2 = 0.02;
    double blend_3 = 0.0;
    std::vector<double> path_pose1 = {-0.143, -0.435, 0.20, -0.001, 3.12, 0.04, velocity, acceleration, blend_1};
    std::vector<double> path_pose2 = {-0.143, -0.51, 0.21, -0.001, 3.12, 0.04, velocity, acceleration, blend_2};
    std::vector<double> path_pose3 = {-0.32, -0.61, 0.31, -0.001, 3.12, 0.04, velocity, acceleration, blend_3};

    // Target is defined as a subset of path_pose3
    std::vector<double> target_pose(&path_pose3[0], &path_pose3[6]);

    std::vector<std::vector<double>> path;
    path.push_back(path_pose1);
    path.push_back(path_pose2);
    path.push_back(path_pose3);

    WHEN("Robot is done moving")
    {
      REQUIRE(rtde_control->moveL(path));

      THEN("Robot must be at target")
      {
        std::vector<double> actual_tcp_pose = rtde_receive->getActualTCPPose();

        for (unsigned int i = 0; i < actual_tcp_pose.size(); i++)
        {
          REQUIRE(actual_tcp_pose[i] == doctest::Approx(target_pose[i]).epsilon(0.005));
        }
      }
    }
  }
}

SCENARIO("Controlling the IO of the robot")
{
  GIVEN("Set output to a specified value")
  {
    // Set StandardDigitalOut
    for (unsigned int i = 0; i < 8; i++)
    {
      rtde_io->setStandardDigitalOut(i, true);
    }
    for (unsigned int i = 0; i < 8; i++)

    // Set ConfigurableDigitalOut
    {
      rtde_io->setConfigurableDigitalOut(i, true);
    }

    // Set ToolDigitalOut
    rtde_io->setToolDigitalOut(0, true);
    rtde_io->setToolDigitalOut(1, true);

    WHEN("Waiting for output to be actuated")
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      THEN("Reading values from IO, must be true")
      {
        // Reading StandardDigitalOut Output, 0-7
        for (unsigned int i = 0; i < 8; i++)
        {
          REQUIRE(rtde_receive->getDigitalOutState(i));
        }

        // Reading ConfigurableDigitalOut, 8-15
        for (unsigned int i = 8; i < 16; i++)
        {
          REQUIRE(rtde_receive->getDigitalOutState(i));
        }

        // Reading ToolDigitalOut, 16-17
        REQUIRE(rtde_receive->getDigitalOutState(16));
        REQUIRE(rtde_receive->getDigitalOutState(17));
      }
    }
  }
}
