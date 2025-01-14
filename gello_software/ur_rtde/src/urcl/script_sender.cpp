/*
* Copyright 2019 FZI Forschungszentrum Informatik - Tristan Schnell schnell@fzi.de (original)
*
* Created on behalf of Universal Robots A/S
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <urcl/script_sender.h>

namespace urcl
{
namespace control
{
ScriptSender::ScriptSender(uint32_t port, const std::string& program)
  : server_(port), script_thread_(), program_(program)
{
  server_.setMessageCallback(
      std::bind(&ScriptSender::messageCallback, this, std::placeholders::_1, std::placeholders::_2));
  server_.setConnectCallback(std::bind(&ScriptSender::connectionCallback, this, std::placeholders::_1));
  server_.setDisconnectCallback(std::bind(&ScriptSender::disconnectionCallback, this, std::placeholders::_1));
  server_.start();
}

void ScriptSender::connectionCallback(const int filedescriptor)
{
  UR_RTDE_LOG_DEBUG("New client connected at FD %d.", filedescriptor);
}

void ScriptSender::disconnectionCallback(const int filedescriptor)
{
  UR_RTDE_LOG_DEBUG("Client at FD %d disconnected.", filedescriptor);
}

void ScriptSender::messageCallback(const int filedescriptor, char* buffer)
{
  if (std::string(buffer) == PROGRAM_REQUEST_)
  {
    UR_RTDE_LOG_INFO("Robot requested program");
    sendProgram(filedescriptor);
  }
}

void ScriptSender::sendProgram(const int filedescriptor)
{
  size_t len = program_.size();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(program_.c_str());
  size_t written;

  if (server_.write(filedescriptor, data, len, written))
  {
    UR_RTDE_LOG_INFO("Sent program to robot");
  }
  else
  {
    UR_RTDE_LOG_ERROR("Could not send program to robot");
  }
}

}  // namespace control
}  // namespace urcl
