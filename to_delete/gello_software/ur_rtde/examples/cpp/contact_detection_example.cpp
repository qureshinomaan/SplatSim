#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
using namespace ur_rtde;

#include <thread>
#include <chrono>

int main(int argc, char* argv[])
{
  std::string hostname = "127.0.0.1";
  std::cout << "Connecting to robot..." << std::endl;
  RTDEControlInterface rtde_control(hostname);
  RTDEReceiveInterface rtde_receive(hostname);

  std::vector<double> home_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

  // Move to initial joint position with a regular moveJ
  std::cout << "Moving to initial position..." << std::endl;
  rtde_control.moveJ(home_q);

  // Target 20 cm down in the Z-Axis of the TCP
  std::vector<double> target = rtde_receive.getActualTCPPose();
  target[2] -= 0.20;

  // first start async move 20 cm down in Z-Axis and the start contact detection
  rtde_control.moveL(target, 0.05, 0.5, true);
  rtde_control.startContactDetection(); // detect contact in direction of TCP movement - no parameter
  std::cout << "Contact detection started " << std::endl;
  bool contact_detected = false;
  int Timeout_s = 10;

  for (int i = 0; i < Timeout_s; ++i)
  {
	  std::cout << "Detecting contact..." << std::endl;
	  contact_detected = rtde_control.readContactDetection();
	  auto AsyncProgress = rtde_control.getAsyncOperationProgressEx();

	  // If contact has been detected, then the move has finished and we can
	  // leave
	  if (contact_detected)
	  {
		  std::cout << "Contact detected" << std::endl;
		  break;
	  }

	  // If the async operation has been finished, the robot has reach its
	  // target position and finished the move - we can leave
	  if ( !AsyncProgress.isAsyncOperationRunning())
	  {
		  std::cout << "Move finished" << std::endl;
		  break;
	  }
	  std::this_thread::sleep_for( std::chrono::milliseconds(1000));
  }

  // If no contact has been detected yet, then robot is still moving. Stop
  // robot move and contact detection.
  if (!contact_detected)
  {
	  std::cout << "No contact detected" << std::endl;
	  rtde_control.stopContactDetection();
	  rtde_control.stopL();
  }
  rtde_control.stopScript();
  return 0;
}
