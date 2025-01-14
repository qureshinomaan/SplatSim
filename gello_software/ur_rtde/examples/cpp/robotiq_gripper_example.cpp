#include <ur_rtde/robotiq_gripper.h>
#include <chrono>
#include <iostream>
#include <thread>

using namespace std;
using namespace ur_rtde;

/**
 * Print object detection status of gripper
 */
void printStatus(int Status)
{
  switch (Status)
  {
    case RobotiqGripper::MOVING:
      std::cout << "moving";
      break;
    case RobotiqGripper::STOPPED_OUTER_OBJECT:
      std::cout << "outer object detected";
      break;
    case RobotiqGripper::STOPPED_INNER_OBJECT:
      std::cout << "inner object detected";
      break;
    case RobotiqGripper::AT_DEST:
      std::cout << "at destination";
      break;
  }

  std::cout << std::endl;
}

int main(int argc, char* argv[])
{
  std::cout << "Gripper test" << std::endl;
  ur_rtde::RobotiqGripper gripper("127.0.0.1", 63352, true);
  gripper.connect();

  // Test emergency release functionality
  if (!gripper.isActive())
  {
    gripper.emergencyRelease(RobotiqGripper::OPEN);
  }
  std::cout << "Fault status: 0x" << std::hex << gripper.faultStatus() << std::dec << std::endl;
  std::cout << "activating gripper" << std::endl;
  gripper.activate();

  // Test setting of position units and conversion of position values
  gripper.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_DEVICE);
  std::cout << "OpenPosition: " << gripper.getOpenPosition() << "  ClosedPosition: " << gripper.getClosedPosition()
            << std::endl;
  gripper.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_NORMALIZED);
  std::cout << "OpenPosition: " << gripper.getOpenPosition() << "  ClosedPosition: " << gripper.getClosedPosition()
            << std::endl;

  // Test of move functionality with normalized values (0.0 - 1.0)
  int status = gripper.move(1, 1, 0, RobotiqGripper::WAIT_FINISHED);
  printStatus(status);
  status = gripper.move(0, 1, 0, RobotiqGripper::WAIT_FINISHED);
  printStatus(status);

  // We preset force and and speed so we don't need to pass it to the following move functions
  gripper.setForce(0.0);
  gripper.setSpeed(0.5);

  // We switch the position unit the mm and define the position range of our gripper
  gripper.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_MM);
  gripper.setPositionRange_mm(50);
  std::cout << "OpenPosition: " << gripper.getOpenPosition() << "  ClosedPosition: " << gripper.getClosedPosition()
            << std::endl;
  gripper.move(50);
  status = gripper.waitForMotionComplete();
  printStatus(status);

  gripper.move(10);
  status = gripper.waitForMotionComplete();
  printStatus(status);

  std::cout << "moving to open position" << std::endl;
  status = gripper.open();
  status = gripper.waitForMotionComplete();
  printStatus(status);

  // Test async move - start move and then wait for completion
  gripper.close(0.02, 0, RobotiqGripper::START_MOVE);
  status = gripper.waitForMotionComplete();
  printStatus(status);

  status = gripper.open(1.0, 0.0, RobotiqGripper::WAIT_FINISHED);
  printStatus(status);

  gripper.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_DEVICE);
  gripper.setUnit(RobotiqGripper::SPEED, RobotiqGripper::UNIT_DEVICE);
  gripper.setUnit(RobotiqGripper::FORCE, RobotiqGripper::UNIT_DEVICE);

  std::cout << "OpenPosition: " << gripper.getOpenPosition() << "  ClosedPosition: " << gripper.getClosedPosition()
            << std::endl;

  gripper.move(255, 5, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  while (RobotiqGripper::MOVING == gripper.objectDetectionStatus())
  {
    std::cout << "waiting..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  printStatus(gripper.objectDetectionStatus());

  std::cout << "disconnecting" << std::endl;
  gripper.disconnect();
}
