// Copyright 2024 Moritz Schmidt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"

#include "soar_ros/Publisher.hpp"
#include "soar_ros/Subscriber.hpp"
#include "soar_ros/Client.hpp"
#include "soar_ros/Service.hpp"
#include "soar_ros/SoarRunner.hpp"

namespace soar_ros
{
void SoarRunner::processOutputLinkChanges(sml::Agent * agent)
{
  // Go through all the commands we've received (if any) since we last ran
  // Soar.
  if (agent == nullptr) {
    return;
  }

  int numberCommands = agent->GetNumberCommands();
  for (int i = 0; i < numberCommands; i++) {
    sml::Identifier * pId = agent->GetCommand(i);

    std::string name = pId->GetCommandName();
    auto outputs_it = outputs_by_agent.find(agent);
    if (outputs_it == outputs_by_agent.end()) {
      RCLCPP_ERROR(this->get_logger(), "No output mappings registered for agent: %s",
          agent->GetAgentName());
      continue;
    }

    auto it = outputs_it->second.find(name);
    if (it == outputs_it->second.end()) {
      RCLCPP_ERROR(this->get_logger(), "No mapping for topic: %s found!", name.c_str());
      continue;
    }
    it->second.get()->process_s2r(pId);
    pId->AddStatusComplete();
  }
}

void SoarRunner::processInput(sml::Agent * agent)
{
  if (agent == nullptr) {
    return;
  }

  auto inputs_it = inputs_by_agent.find(agent);
  if (inputs_it != inputs_by_agent.end()) {
    for (const auto & input : inputs_it->second) {
      input->process_r2s();
    }
  }
  agent->Commit();
}

std::string SoarRunner::getSoarLogFilePath()
{
  auto currentTime = std::chrono::system_clock::now();
  std::time_t currentTime_t = std::chrono::system_clock::to_time_t(currentTime);
  std::tm * currentTime_tm = std::gmtime(&currentTime_t);
  std::stringstream ss;
  ss << std::put_time(currentTime_tm, "%Y_%m_%d-%H_%M_%S");

  std::string timestamp = ss.str();

  std::string root_file_path = "";

  const char * ros_log_dir = std::getenv("ROS_LOG_DIR");
  const char * ros_home = std::getenv("ROS_HOME");
  const char * home = std::getenv("HOME");
  if (ros_log_dir != nullptr) {
    root_file_path = std::getenv("ROS_LOG_DIR");
    RCLCPP_INFO_STREAM(this->get_logger(), "ROS_LOG_DIR defined: Log directory:" << root_file_path);
  } else if (ros_home != nullptr) {
    std::string ros_home_path(ros_home);
    root_file_path = ros_home_path + "/log/";
    RCLCPP_INFO_STREAM(this->get_logger(), "ROS_HOME defined: Log directory: " << root_file_path);
  } else {
    if (home == nullptr) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "HOME ENV not found/ defined. Log file might be "
                          "relative to Soar Java debugger.");
    } else {
      std::string home_path(home);
      root_file_path = home_path + "/.ros/log/";
      RCLCPP_WARN_STREAM(this->get_logger(),
          "No logging directory via ENV defined; Defaulting to: " << root_file_path);
    }
  }
  return root_file_path;
}

SoarRunner::SoarRunner()
: rclcpp::Node("SoarRunner"), isRunning(false)
{
  // Sometimes the error binding the listener socket to its port number (Soar)
  // occurs from: Origin:
  // https://github.com/SoarGroup/Soar/blob/9530289cc5452fc3a4c75cf5b5f4ca25cb46fde4/Core/ConnectionSML/src/sock_ListenerSocket.cpp#L146C1-L146C93
  // Potential reason: Soar Java Debugger is openened and has a kernel running
  // blocking the port.

  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "Enable debug mode.";
  this->declare_parameter("debug", false, param_desc);
  m_debug = this->get_parameter("debug").as_bool();

  pKernel = sml::Kernel::CreateKernelInNewThread();
  if (pKernel->HadError()) {
    std::string err_msg = pKernel->GetLastErrorDescription();
    RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
    throw std::runtime_error(err_msg);
  }

  pKernel->RegisterForUpdateEvent(sml::smlEVENT_AFTER_ALL_OUTPUT_PHASES, updateEventHandler, this);

  m_getSoarKernelStatus = this->create_service<std_srvs::srv::Trigger>(
      "/soar_ros/kernel/status",
      std::bind(&SoarRunner::getSoarKernelStatus, this, std::placeholders::_1,
                                           std::placeholders::_2, std::placeholders::_3));

  m_kernelRun = this->create_service<std_srvs::srv::Trigger>(
      "/soar_ros/kernel/run",
      std::bind(&SoarRunner::runSoarKernel, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  m_kernelStop = this->create_service<std_srvs::srv::Trigger>(
      "/soar_ros/kernel/stop", std::bind(&SoarRunner::stopSoarKernel, this, std::placeholders::_1,
                                         std::placeholders::_2, std::placeholders::_3));

  m_debuggerLaunch = this->create_service<std_srvs::srv::Trigger>(
      "/soar_ros/debugger/launch",
      std::bind(&SoarRunner::debuggerLaunch, this, std::placeholders::_1,
                                             std::placeholders::_2, std::placeholders::_3));
}

void SoarRunner::debuggerLaunch(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
  [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  stopThread();
  if (agents.empty()) {
    std::string err_msg = "No agents available to spawn debugger.";
    RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
    response->success = false;
    response->message = err_msg;
    return;
  }

  bool all_spawned = true;
  std::string failed_agents;
  for (auto * agent : agents) {
    if (agent == nullptr) {
      continue;
    }
    if (!agent->SpawnDebugger()) {
      all_spawned = false;
      if (!failed_agents.empty()) {
        failed_agents += ", ";
      }
      failed_agents += agent->GetAgentName();
    }
  }

  if (!all_spawned) {
    std::string err_msg =
      "Unable to open Soar Java debugger for agents: " + failed_agents +
      ". Is Java and the Soar debugger installed?";
    RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
    response->success = false;
    response->message = err_msg;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Spawning Soar debugger for all agents.");
    response->success = true;
  }
}

void SoarRunner::getSoarKernelStatus(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
  [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (isRunning.load()) {
    response->message = "Soar kernel isRunning: true";
  } else {
    response->message = "Soar kernel isRunnings: false";
  }
  response->success = true;
}

void SoarRunner::runSoarKernel(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
  [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Attempting to run Soar kernel from ROS.");
  startThread();
  if (isRunning.load()) {
    response->message = "Soar kernel isRunning: true";
    response->success = true;
  } else {
    response->message = "Soar kernel isRunnings: false";
    response->success = false;
  }
}

void SoarRunner::stopSoarKernel(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
  [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Attempting to stop Soar kernel from ROS.");
  stopThread();
  if (isRunning.load()) {
    response->message = "Soar kernel isRunning: true";
    response->success = false;
  } else {
    response->message = "Soar kernel isRunnings: false";
    response->success = true;
  }
}

sml::Agent * SoarRunner::addAgent(
  const std::string & agent_name,
  const std::string & path_productions)
{
  sml::Agent * pAgent = pKernel->CreateAgent(agent_name.c_str());
  if (pKernel->HadError()) {
    std::string err_msg = pKernel->GetLastErrorDescription();
    RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
    throw std::runtime_error(err_msg);
  }

  pAgent->LoadProductions(path_productions.c_str());
  if (pAgent->HadError()) {
    std::string err_msg = pKernel->GetLastErrorDescription();
    RCLCPP_ERROR_STREAM(this->get_logger(), err_msg);
    throw std::runtime_error(err_msg);
  }

  pAgent->RegisterForPrintEvent(sml::smlEVENT_PRINT, SoarPrintEventHandler, this);

  std::string filepath = getSoarLogFilePath();
  std::string cmd = "output log " + filepath;
  pAgent->ExecuteCommandLine(cmd.c_str());

  if (m_debug) {
    // TODO(moschmdt): Check if another debug window is still open.
    // Force close it prior to reopening another window.
    if (pAgent->SpawnDebugger() == false) {
      // BUG This is not triggered when another window is already open:
      // Error: Error binding the listener socket to its port number
      // Failed to create the internet listener socket.  Shutting down
      // listener thread.
      RCLCPP_ERROR(this->get_logger(), "Spawning Soar debugger failed.");
    }

    pAgent->ExecuteCommandLine("watch 5");
  }

  agents.push_back(pAgent);

  return pAgent;
}

SoarRunner::~SoarRunner()
{
  stopThread();

  for (auto * agent : agents) {
    if (agent == nullptr) {
      continue;
    }

    agent->ExecuteCommandLine("output log --close");

    if (m_debug) {
      agent->KillDebugger();
    }

    pKernel->DestroyAgent(agent);
  }
  pKernel->Shutdown();
  delete pKernel;
}

void SoarPrintEventHandler(
  [[maybe_unused]] sml::smlPrintEventId id, void * pSoarRunner_, sml::Agent * pAgent,
  char const * pMessage)
{
  SoarRunner * pSoarRunner = static_cast<SoarRunner *>(pSoarRunner_);

  std::string str(pMessage);

  if (str.find("(wait)") != std::string::npos) {
    return;
  }

  if (str.find("O:") != std::string::npos || str.find("==>") != std::string::npos) {
    str.erase(
        std::remove_if(str.begin(), str.end(), [&](char ch) {
        return std::iscntrl(static_cast<unsigned char>(ch));
                                                                                                                 }),
        str.end());
  }

  std::stringstream ss(str);
  std::string line;

  while (std::getline(ss, line, '\n')) {
    std::string message = std::string(pAgent->GetAgentName()) + ": " + str;
    auto logger = pSoarRunner->get_logger();
    if (str.find("System halted.") != std::string::npos) {
      RCLCPP_ERROR(logger, message.c_str());
      pSoarRunner->stopThread();
    } else {
      RCLCPP_INFO(logger, message.c_str());
    }
  }
}

void updateEventHandler(
  [[maybe_unused]] sml::smlUpdateEventId id, void * pSoarRunner_,
  [[maybe_unused]] sml::Kernel * kernel_ptr, [[maybe_unused]] sml::smlRunFlags run_flags)
{
  SoarRunner * pSoarRunner = static_cast<SoarRunner *>(pSoarRunner_);
  pSoarRunner->updateWorld();
}
}  // namespace soar_ros
