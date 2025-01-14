#pragma once
#ifndef RTDE_SCRIPT_CLIENT_H
#define RTDE_SCRIPT_CLIENT_H

#include <ur_rtde/rtde_export.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <memory>
#include <string>
#include <vector>

namespace ur_rtde
{
struct ScriptInjectItem
{
  std::string search_string;
  std::string inject_string;
  ScriptInjectItem(const std::string& search, const std::string& inject) : search_string(search), inject_string(inject)
  {
  }
};

class ScriptClient
{
 public:
  RTDE_EXPORT explicit ScriptClient(std::string hostname, uint32_t major_control_version,
                                    uint32_t minor_control_version, int port = 30003, bool verbose = false);

  RTDE_EXPORT virtual ~ScriptClient();

  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
  };

 public:
  RTDE_EXPORT void connect();
  RTDE_EXPORT void disconnect();
  RTDE_EXPORT bool isConnected();

  /**
   * Assign a custom script file that will be sent to device if the
   * sendScript() function is called.
   * Setting an empty file_name will disable the custom script loading
   * This eases debugging when modifying the control
   * script because it does not require to recompile the whole library
   */
  RTDE_EXPORT void setScriptFile(const std::string& file_name);

  /**
   * Send the internal control script that is compiled into the library
   * or the assigned control script file
   */
  RTDE_EXPORT bool sendScript();

  /**
   * Send the script file with the given file_name
   */
  RTDE_EXPORT bool sendScript(const std::string& file_name);

  RTDE_EXPORT bool sendScriptCommand(const std::string& cmd_str);

  RTDE_EXPORT void setScriptInjection(const std::string& search_string, const std::string& inject_string);

  /**
   * Get the corrected rtde_control script as a std::string
   */
  RTDE_EXPORT std::string getScript();

 private:
  RTDE_EXPORT bool removeUnsupportedFunctions(std::string& ur_script);
  RTDE_EXPORT bool scanAndInjectAdditionalScriptCode(std::string& ur_script);

 private:
  std::string hostname_;
  uint32_t major_control_version_;
  uint32_t minor_control_version_;
  int port_;
  bool verbose_;
  ConnectionState conn_state_;
  std::string script_file_name_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
  std::vector<ScriptInjectItem> script_injections_;
};

}  // namespace ur_rtde

#endif  // RTDE_SCRIPT_CLIENT_H
