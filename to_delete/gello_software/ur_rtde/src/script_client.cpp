#include <ur_rtde/script_client.h>

#include <algorithm>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/socket_base.hpp>
#include <boost/asio/write.hpp>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

#include "ur_rtde/rtde_control_script.h"

using boost::asio::ip::tcp;

namespace ur_rtde
{
ScriptClient::ScriptClient(std::string hostname, uint32_t major_control_version, uint32_t minor_control_version,
                           int port, bool verbose)
    : hostname_(std::move(hostname)),
      major_control_version_(major_control_version),
      minor_control_version_(minor_control_version),
      port_(port),
      verbose_(verbose),
      conn_state_(ConnectionState::DISCONNECTED)
{
}

ScriptClient::~ScriptClient() = default;

void ScriptClient::connect()
{
  io_service_ = std::make_shared<boost::asio::io_service>();
  socket_.reset(new boost::asio::ip::tcp::socket(*io_service_));
  socket_->open(boost::asio::ip::tcp::v4());
  boost::asio::ip::tcp::no_delay no_delay_option(true);
  boost::asio::socket_base::reuse_address sol_reuse_option(true);
  socket_->set_option(no_delay_option);
  socket_->set_option(sol_reuse_option);
  resolver_ = std::make_shared<tcp::resolver>(*io_service_);
  tcp::resolver::query query(hostname_, std::to_string(port_));
  boost::asio::connect(*socket_, resolver_->resolve(query));
  conn_state_ = ConnectionState::CONNECTED;
  if (verbose_)
    std::cout << "Connected successfully to UR script server: " << hostname_ << " at " << port_ << std::endl;
}

bool ScriptClient::isConnected()
{
  return conn_state_ == ConnectionState::CONNECTED;
}

void ScriptClient::disconnect()
{
  /* We use reset() to safely close the socket,
   * see: https://stackoverflow.com/questions/3062803/how-do-i-cleanly-reconnect-a-boostsocket-following-a-disconnect
   */
  socket_.reset();
  conn_state_ = ConnectionState::DISCONNECTED;
  if (verbose_)
    std::cout << "Script Client - Socket disconnected" << std::endl;
}

bool ScriptClient::sendScriptCommand(const std::string& cmd_str)
{
  if (isConnected() && !cmd_str.empty())
  {
    boost::asio::write(*socket_, boost::asio::buffer(cmd_str));
  }
  else
  {
    std::cerr << "Please connect to the controller before calling sendScriptCommand()" << std::endl;
    return false;
  }

  return true;
}

void ScriptClient::setScriptFile(const std::string& file_name)
{
  script_file_name_ = file_name;
}

/**
 * Internal private helper function to load a script to avoid duplicated code
 */
static bool loadScript(const std::string& file_name, std::string& str)
{
  // Read in the UR script file
  // Notice! We use this method as it allocates the memory up front, strictly for performance.
  std::ifstream file(file_name.c_str());
  if (file)
  {
    file.seekg(0, std::ios::end);
    str.reserve(file.tellg());
    file.seekg(0, std::ios::beg);
    // Do not remove the redundant parentheses, this is to avoid the most vexing parse!
    str.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return true;
  }
  else
  {
    std::cerr << "There was an error reading the provided script file: " << file_name << std::endl;
    return false;
  }
}

bool ScriptClient::removeUnsupportedFunctions(std::string& ur_script)
{
  // Remove lines not fitting for the specific version of the controller
  auto n = ur_script.find("$");

  while (n != std::string::npos)
  {
    const std::string version_str = ur_script.substr(n+1, 9);
    const std::string major_str(1, version_str.at(0));
    const std::string minor_str = version_str.substr(2, 4);
    const std::string extra_major_str(1, version_str.at(5));
    const std::string extra_minor_str = version_str.substr(7, 2);

    if (!major_str.empty() && !minor_str.empty() && major_str != " " && minor_str != " ")
    {
      uint32_t major_version_needed = uint32_t(std::stoi(major_str));
      uint32_t minor_version_needed = uint32_t(std::stoi(minor_str));
      uint32_t extra_major_version_needed = 0;
      uint32_t extra_minor_version_needed = 0;
      bool additional_version_specified = false;

      if (version_str.at(4) == '|') // Look for an additionally supported CB version.
      {
        if (!extra_major_str.empty())
          extra_major_version_needed = uint32_t(std::stoi(extra_major_str));
        if (!extra_minor_str.empty())
          extra_minor_version_needed = uint32_t(std::stoi(extra_minor_str));
        additional_version_specified = true;
      }

      if ((major_control_version_ > major_version_needed) ||
          (major_control_version_ == major_version_needed && minor_control_version_ >= minor_version_needed) ||
          (major_control_version_ == extra_major_version_needed && minor_control_version_ >= extra_minor_version_needed))
      {
        // Keep the line
        if (additional_version_specified)
        {
          ur_script.erase(n, 10);
          ur_script.insert(n, "          ");
        }
        else
        {
          ur_script.erase(n, 5);
          ur_script.insert(n, "     ");
        }
      }
      else
      {
        // Erase the line
        ur_script.erase(n, ur_script.find("\n", n) - n + 1);
      }
    }
    else
    {
      std::cerr << "Could not read the control version required from the control script!" << std::endl;
      return false;
    }

    n = ur_script.find("$");
  }
  return true;
}

bool ScriptClient::scanAndInjectAdditionalScriptCode(std::string& ur_script)
{
  unsigned long n = 0;
  // Now scan the script for injection points where we can inject additional
  // script code
  for (const auto& script_injection : script_injections_)
  {
    n = ur_script.find(script_injection.search_string);
    if (std::string::npos == n)
    {
      if (verbose_)
        std::cout << "script_injection [" << script_injection.search_string << "] not found in script" << std::endl;
      continue;
    }

    // Now inject custom script code into the script
    ur_script.insert(n + script_injection.search_string.length(), script_injection.inject_string);
    if (verbose_)
    {
      std::cout << "script_injection [" << script_injection.search_string << "] found at pos " << n << std::endl;
      std::cout << ur_script.substr(n - 100, n + script_injection.search_string.length() +
                                                 script_injection.inject_string.length() + 100)
                << std::endl;
    }
  }
  return true;
}

bool ScriptClient::sendScript()
{
  std::string ur_script;
  // If the user assigned a custom control script, then we use this one instead
  // of the internal compiled one.
  if (!script_file_name_.empty())
  {
    // If loading fails, we fall back to the default script file
    if (!loadScript(script_file_name_, ur_script))
    {
      std::cerr << "Error loading custom script file. Falling back to internal script file." << std::endl;
      ur_script = std::string();
    }
  }

  if (ur_script.empty())
  {
    ur_script = "def rtde_control():\n";
    ur_script += UR_SCRIPT;
    ur_script += "end\n";
  }

  // Remove if any, functions not supported on this version of the controller
  if (!removeUnsupportedFunctions(ur_script))
    return false;
  // Scan the script for injection points where additional script code can be injected.
  scanAndInjectAdditionalScriptCode(ur_script);

  if (isConnected() && !ur_script.empty())
  {
    std::size_t bytes_transferred = 0;
    bytes_transferred = boost::asio::write(*socket_, boost::asio::buffer(ur_script));
    if (ur_script.size() == bytes_transferred)
      return true;
    else
      return false;
  }
  else
  {
    std::cerr << "Please connect to the controller before calling sendScript()" << std::endl;
    return false;
  }

  return true;
}

bool ScriptClient::sendScript(const std::string& file_name)
{
  std::string str;
  if (!loadScript(file_name, str))
  {
    return false;
  }

  if (isConnected() && !str.empty())
  {
    boost::asio::write(*socket_, boost::asio::buffer(str));
  }
  else
  {
    std::cerr << "Please connect to the controller before calling sendScript()" << std::endl;
    return false;
  }

  return true;
}

void ScriptClient::setScriptInjection(const std::string& search_string, const std::string& inject_string)
{
  auto it = std::find_if(script_injections_.begin(), script_injections_.end(),
                         [&](const ScriptInjectItem& val) { return search_string == val.search_string; });
  if (it != script_injections_.end())
  {
    it->inject_string = inject_string;
  }
  else
  {
    script_injections_.emplace_back(search_string, inject_string);
  }
}

std::string ScriptClient::getScript()
{
  std::string ur_script;
  // If the user assigned a custom control script, then we use this one instead
  // of the internal compiled one.
  if (!script_file_name_.empty())
  {
    // If loading fails, we fall back to the default script file
    if (!loadScript(script_file_name_, ur_script))
    {
      std::cerr << "Error loading custom script file. Falling back to internal script file." << std::endl;
      ur_script = std::string();
    }
  }

  if (ur_script.empty())
  {
    ur_script = UR_SCRIPT;
  }

  // Remove if any, functions not supported on this version of the controller
  if (!removeUnsupportedFunctions(ur_script))
    std::cerr << "Error removing unsupported functions from control script!" << std::endl;
  // Scan the script for injection points where additional script code can be injected.
  scanAndInjectAdditionalScriptCode(ur_script);

  return ur_script;
}

}  // namespace ur_rtde
