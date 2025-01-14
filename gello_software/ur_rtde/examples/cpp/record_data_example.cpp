#include <ur_rtde/rtde_receive_interface.h>
#include <boost/program_options.hpp>
#include <thread>
#include <chrono>
#include <csignal>
#include <string>
#include <iostream>

using namespace ur_rtde;
using namespace std::chrono;
namespace po = boost::program_options;

// Interrupt flag
bool running = true;
void raiseFlag(int param)
{
  running = false;
}

int main(int argc, char* argv[])
{
  try {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Record robot data to a (.csv) file")
        ("robot_ip", po::value<std::string>()->default_value("localhost"),
             "the IP address of the robot")
        ("frequency", po::value<double>()->default_value(500.0),
                 "the frequency at which the data is recorded (default is 500Hz)")
        ("output", po::value<std::string>()->default_value("robot_data.csv"),
                     "data output (.csv) file to write to (default is \"robot_data.csv\"")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }

    signal(SIGINT, raiseFlag);
    double frequency = vm["frequency"].as<double>();
    RTDEReceiveInterface rtde_receive(vm["robot_ip"].as<std::string>(), frequency, {}, true, false);

    rtde_receive.startFileRecording(vm["output"].as<std::string>());
    std::cout << "Data recording started. press [Ctrl-C] to end recording." << std::endl;
    int i=0;
    while (running)
    {
      steady_clock::time_point t_start = rtde_receive.initPeriod();
      if (i % 10 == 0)
      {
        std::cout << '\r';
        printf("%.3d samples.", i);
        std::cout << std::flush;
      }
      rtde_receive.waitPeriod(t_start);
      i++;
    }

    // Stop data recording.
    rtde_receive.stopFileRecording();
    std::cout << "\nData recording stopped." << std::endl;
  }
  catch(std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  catch(...) {
    std::cerr << "Exception of unknown type!\n";
  }
  return 0;
}
