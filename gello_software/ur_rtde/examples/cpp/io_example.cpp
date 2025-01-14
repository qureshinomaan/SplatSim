#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>
#include <thread>

using namespace ur_rtde;

int main(int argc, char* argv[])
{
  RTDEIOInterface rtde_io("127.0.0.1");
  RTDEReceiveInterface rtde_receive("127.0.0.1");

  /** How-to set and get standard and tool digital outputs. Notice that we need the
    * RTDEIOInterface for setting an output and RTDEReceiveInterface for getting the state
    * of an output.
    */

  if (rtde_receive.getDigitalOutState(7))
    std::cout << "Standard digital out (7) is HIGH" << std::endl;
  else
    std::cout << "Standard digital out (7) is LOW" << std::endl;

  if (rtde_receive.getDigitalOutState(16))
    std::cout << "Tool digital out (16) is HIGH" << std::endl;
  else
    std::cout << "Tool digital out (16) is LOW" << std::endl;

  rtde_io.setStandardDigitalOut(7, true);
  rtde_io.setToolDigitalOut(0, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  if (rtde_receive.getDigitalOutState(7))
    std::cout << "Standard digital out (7) is HIGH" << std::endl;
  else
    std::cout << "Standard digital out (7) is LOW" << std::endl;

  if (rtde_receive.getDigitalOutState(16))
    std::cout << "Tool digital out (16) is HIGH" << std::endl;
  else
    std::cout << "Tool digital out (16) is LOW" << std::endl;

  // How to set a analog output with a specified current ratio
  rtde_io.setAnalogOutputCurrent(1, 0.25);

  return 0;
}