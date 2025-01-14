#pragma once
#ifndef RTDE_DASHBOARD_CLIENT_H
#define RTDE_DASHBOARD_CLIENT_H

#include <ur_rtde/rtde_export.h>
#include <ur_rtde/dashboard_enums.h>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/streambuf.hpp>
#include <memory>
#include <string>

namespace ur_rtde
{
/**
 * This class provides the interface for access to the UR dashboard server.
 */
class DashboardClient
{
 public:

  RTDE_EXPORT explicit DashboardClient(std::string hostname, int port = 29999, bool verbose = false);

  RTDE_EXPORT virtual ~DashboardClient();

  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
  };

 public:
  /**
   * Connects to the dashboard server with the given timeout value.
   */
  RTDE_EXPORT void connect(uint32_t timeout_ms = 2000);

  /**
   * @brief Returns true if the dashboard client is connected to the server.
   */
  RTDE_EXPORT bool isConnected();
  RTDE_EXPORT void disconnect();
  RTDE_EXPORT void send(const std::string &str);
  RTDE_EXPORT std::string receive();

  /**
   * Returns when both program and associated installation has loaded..
   * The load command fails if the associated installation requires confirmation
   * of safety. In this case an exception with an error message is thrown.
   */
  RTDE_EXPORT void loadURP(const std::string &urp_name);

  /**
   * Throws exception if program fails to start.
   */
  RTDE_EXPORT void play();

  /**
   * Throws exception if the program fails to stop.
   */
  RTDE_EXPORT void stop();

  /**
   * Throws exception if the program fails to pause.
   */
  RTDE_EXPORT void pause();

  /**
   * Closes connection
   */
  RTDE_EXPORT void quit();

  /**
   * Shuts down and turns off robot and controller
   */
  RTDE_EXPORT void shutdown();

  /**
   * Execution state enquiry.
   * @return Returns true if program is running.
   */
  RTDE_EXPORT bool running();

  /**
   * The popup-text will be translated to the selected language, if the text
   * exists in the language file
   */
  RTDE_EXPORT void popup(const std::string &message);

  /**
   * Closes the popup
   */
  RTDE_EXPORT void closePopup();

  /**
   * Closes a safety popup
   */
  RTDE_EXPORT void closeSafetyPopup();

  /**
   * Powers on the robot arm
   */
  RTDE_EXPORT void powerOn();

  /**
   * Powers off the robot arm
   */
  RTDE_EXPORT void powerOff();

  /**
   * Powers off the robot arm
   */
  RTDE_EXPORT void brakeRelease();

  /**
   * @brief Closes the current popup and unlocks protective stop.
   * The unlock protective stop command fails with an exception if less than 5
   * seconds has passed since the protective stop occurred.
   */
  RTDE_EXPORT void unlockProtectiveStop();

  /**
   * @brief Use this when robot gets a safety fault or violation to restart the safety.
   * After safety has been rebooted the robot will be in Power Off.
   * \attention You should always ensure it is okay to restart the system.
   * It is highly recommended to check the error log before using this
   * command (either via PolyScope or e.g. ssh connection).
   */
  RTDE_EXPORT void restartSafety();
  RTDE_EXPORT std::string polyscopeVersion();
  RTDE_EXPORT std::string programState();
  RTDE_EXPORT std::string robotmode();
  RTDE_EXPORT std::string getRobotModel();
  RTDE_EXPORT std::string getLoadedProgram();

  /**
   * @brief Safety mode inquiry.
   * A Safeguard Stop resulting from any type of safeguard I/O or a
   * configurable I/O three position enabling device result in SAFEGUARD_STOP.
   * This function is deprecated. Instead, use safetystatus().
   */
  RTDE_EXPORT std::string safetymode();

  /**
   * @brief Safety status inquiry.
   * This differs from 'safetymode' by specifying if a given Safeguard Stop
   * was caused by the permanent safeguard I/O stop, a configurable I/O
   * automatic mode safeguard stop or a configurable I/O three position
   * enabling device stop. Thus, this is strictly more detailed than safetymode()..
   */
  RTDE_EXPORT std::string safetystatus();

  /**
   * Adds log-message to the Log history
   */
  RTDE_EXPORT void addToLog(const std::string &message);

  /**
   * @brief Returns the save state of the active program.
   */
  RTDE_EXPORT bool isProgramSaved();

  /**
   * @brief Returns the remote control status of the robot.
   * If the robot is in remote control it returns false and if remote control
   * is disabled or robot is in local control it returns false.
   */
  RTDE_EXPORT bool isInRemoteControl();

  RTDE_EXPORT void setUserRole(const UserRole &role);

  /**
   *  @brief Returns serial number of the robot. (Serial number like "20175599999")
   *  @returns serial number as a std::string
   */
  RTDE_EXPORT std::string getSerialNumber();

 private:
  /**
   * Reads a single line (until newline) with timeout
   * If no timeout is given (value < 0) then an internal default timeout
   * values is used
   * \return The received data string without newline
   */
  template <typename AsyncReadStream>
  std::string async_readline(AsyncReadStream& s, int timeout_ms = -1);

  /**
   * For socket timeouts
   */
  void check_deadline();

  std::string hostname_;
  int port_;
  bool verbose_;
  ConnectionState conn_state_;
  boost::asio::io_service io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
  boost::asio::deadline_timer deadline_;
  boost::asio::streambuf input_buffer_;
};

}  // namespace ur_rtde

#endif  // RTDE_DASHBOARD_CLIENT_H
