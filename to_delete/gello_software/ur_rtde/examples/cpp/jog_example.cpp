#include <ur_rtde/rtde_control_interface.h>
#include <ncurses.h>
#include <chrono>
#include <iostream>
#include <thread>

using namespace ur_rtde;
using namespace std::chrono;

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("127.0.0.1");

  // Curses Initialisations
  initscr();
  raw();
  keypad(stdscr, TRUE);
  noecho();
  timeout(10);

  // Parameters
  double speed_magnitude = 0.15;
  std::vector<double> speed_vector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  rtde_control.jogStart(speed_vector, RTDEControlInterface::FEATURE_TOOL);

  std::string instructions("[ Use arrow keys to control the robot, to exit press 'q' ]");
  int c, row, col;
  getmaxyx(stdscr, row, col);
  mvprintw(row / 2, (col-strlen(instructions.c_str())) / 2, "%s", instructions.c_str());

  while ((c = getch()) != 'q')
  {
    steady_clock::time_point t_start = rtde_control.initPeriod();
    c = getch();
    switch (c)
    {
      case KEY_UP:
        speed_vector = {0.0, 0.0, -speed_magnitude, 0.0, 0.0, 0.0};
        rtde_control.jogStart(speed_vector, RTDEControlInterface::FEATURE_TOOL);
        break;
      case KEY_DOWN:
        speed_vector = {0.0, 0.0, speed_magnitude, 0.0, 0.0, 0.0};
        rtde_control.jogStart(speed_vector, RTDEControlInterface::FEATURE_TOOL);
        break;
      case KEY_LEFT:
        speed_vector = {speed_magnitude, 0.0, 0.0, 0.0, 0.0, 0.0};
        rtde_control.jogStart(speed_vector, RTDEControlInterface::FEATURE_TOOL);
        break;
      case KEY_RIGHT:
        speed_vector = {-speed_magnitude, 0.0, 0.0, 0.0, 0.0, 0.0};
        rtde_control.jogStart(speed_vector, RTDEControlInterface::FEATURE_TOOL);
        break;
      default:
        speed_vector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        rtde_control.jogStart(speed_vector, RTDEControlInterface::FEATURE_TOOL);
        break;
    }
    rtde_control.waitPeriod(t_start);
  }

  endwin();
  rtde_control.jogStop();
  rtde_control.stopScript();

  return 0;
}
