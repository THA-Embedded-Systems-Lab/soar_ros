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

#include "soar_ros/SoarRunner.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "sml_Client.h"

namespace soar_ros
{

  // ---------------------------------------------------------------------------
  // Free-function callbacks registered with the Soar kernel
  // ---------------------------------------------------------------------------

  void SoarPrintEventHandler(
      [[maybe_unused]] sml::smlPrintEventId id,
      void *pSoarRunner_,
      sml::Agent *pAgent,
      char const *pMessage)
  {
    SoarRunner *pSoarRunner = static_cast<SoarRunner *>(pSoarRunner_);

    std::string str(pMessage);

    if (str.find("(wait)") != std::string::npos)
    {
      return;
    }

    if (str.find("O:") != std::string::npos || str.find("==>") != std::string::npos)
    {
      str.erase(
          std::remove_if(str.begin(), str.end(), [](char ch)
                         { return std::iscntrl(static_cast<unsigned char>(ch)); }),
          str.end());
    }

    std::stringstream ss(str);
    std::string line;
    while (std::getline(ss, line, '\n'))
    {
      std::string message = std::string(pAgent->GetAgentName()) + ": " + line;
      auto logger = pSoarRunner->get_logger();
      if (line.find("System halted.") != std::string::npos)
      {
        RCLCPP_ERROR(logger, "%s", message.c_str());
        pSoarRunner->stopThread();
      }
      else
      {
        RCLCPP_INFO(logger, "%s", message.c_str());
      }
    }
  }

  void updateEventHandler(
      [[maybe_unused]] sml::smlUpdateEventId id,
      void *pSoarRunner_,
      [[maybe_unused]] sml::Kernel *kernel_ptr,
      [[maybe_unused]] sml::smlRunFlags run_flags)
  {
    SoarRunner *pSoarRunner = static_cast<SoarRunner *>(pSoarRunner_);
    pSoarRunner->updateWorld();
  }

  // ---------------------------------------------------------------------------
  // SoarRunner member functions
  // ---------------------------------------------------------------------------

  SoarRunner::SoarRunner(const std::string &node_name)
      : rclcpp::Node(node_name)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Enable debug mode.";
    declare_parameter("debug", false, param_desc);
    m_debug = get_parameter("debug").as_bool();

    m_kernel = sml::Kernel::CreateKernelInNewThread();
    if (!m_kernel || m_kernel->HadError())
    {
      std::string err_msg = m_kernel ? m_kernel->GetLastErrorDescription() : "null kernel";
      throw std::runtime_error("Failed to create Soar kernel: " + err_msg);
    }

    // Register the I/O callback — called after every output phase by the kernel.
    m_kernel->RegisterForUpdateEvent(
        sml::smlEVENT_AFTER_ALL_OUTPUT_PHASES, updateEventHandler, this);

    // ROS2 service interfaces for external control.
    m_getSoarKernelStatus = create_service<std_srvs::srv::Trigger>(
        "/soar_ros/kernel/status",
        std::bind(&SoarRunner::getSoarKernelStatus, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_kernelRun = create_service<std_srvs::srv::Trigger>(
        "/soar_ros/kernel/run",
        std::bind(&SoarRunner::runSoarKernel, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_kernelStop = create_service<std_srvs::srv::Trigger>(
        "/soar_ros/kernel/stop",
        std::bind(&SoarRunner::stopSoarKernel, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    m_debuggerLaunch = create_service<std_srvs::srv::Trigger>(
        "/soar_ros/debugger/launch",
        std::bind(&SoarRunner::debuggerLaunch, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(get_logger(), "Soar kernel created");
  }

  SoarRunner::~SoarRunner()
  {
    stopThread();

    for (auto &soar_agent : m_agents)
    {
      sml::Agent *raw = soar_agent->getSmlAgent();
      if (!raw)
      {
        continue;
      }
      raw->ExecuteCommandLine("output log --close");
      if (m_debug)
      {
        raw->KillDebugger();
      }
      m_kernel->DestroyAgent(raw);
    }
    m_agents.clear();

    m_kernel->Shutdown();
    delete m_kernel;
    m_kernel = nullptr;

    RCLCPP_INFO(get_logger(), "Soar kernel shut down");
  }

  std::shared_ptr<SoarAgent> SoarRunner::addAgent(
      const std::string &agent_name,
      const std::string &source_file,
      bool auto_delete_soar_io_on_complete)
  {
    sml::Agent *raw_agent = m_kernel->CreateAgent(agent_name.c_str());
    if (!raw_agent || m_kernel->HadError())
    {
      std::string err_msg = m_kernel->GetLastErrorDescription();
      RCLCPP_ERROR_STREAM(get_logger(), err_msg);
      throw std::runtime_error("Failed to create Soar agent: " + agent_name + ": " + err_msg);
    }

    if (!source_file.empty())
    {
      raw_agent->LoadProductions(source_file.c_str());
      if (raw_agent->HadError())
      {
        std::string err_msg = raw_agent->GetLastErrorDescription();
        RCLCPP_ERROR_STREAM(get_logger(), err_msg);
        throw std::runtime_error(
            "Failed to load productions for agent " + agent_name + ": " + err_msg);
      }
      RCLCPP_INFO(get_logger(), "Loaded productions for agent '%s'", agent_name.c_str());
    }

    // Register the Soar print event so agent output is routed to ROS logger.
    raw_agent->RegisterForPrintEvent(
        sml::smlEVENT_PRINT, SoarPrintEventHandler, this);

    if (m_debug)
    {
      if (!raw_agent->SpawnDebugger())
      {
        RCLCPP_ERROR(get_logger(), "Spawning Soar debugger failed for agent '%s'.", agent_name.c_str());
      }
      raw_agent->ExecuteCommandLine("watch 5");
    }

    auto node_ptr = std::static_pointer_cast<rclcpp::Node>(shared_from_this());
    auto soar_agent = std::make_shared<SoarAgent>(raw_agent, node_ptr, auto_delete_soar_io_on_complete);

    m_agents.push_back(soar_agent);
    RCLCPP_INFO(get_logger(), "Agent '%s' created", agent_name.c_str());
    return soar_agent;
  }

  void SoarRunner::run()
  {
    startThread();
  }

  void SoarRunner::soarRunLoop()
  {
    RCLCPP_INFO(get_logger(), "Soar run loop started");
    while (m_running.load() && rclcpp::ok())
    {
      m_kernel->RunAllAgents(1);
    }
    RCLCPP_INFO(get_logger(), "Soar run loop stopped");
  }

  // ---------------------------------------------------------------------------
  // ROS2 service handlers
  // ---------------------------------------------------------------------------

  void SoarRunner::debuggerLaunch(
      [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    stopThread();

    if (m_agents.empty())
    {
      std::string err_msg = "No agents available to spawn debugger.";
      RCLCPP_ERROR_STREAM(get_logger(), err_msg);
      response->success = false;
      response->message = err_msg;
      return;
    }

    bool all_spawned = true;
    std::string failed_agents;
    for (auto &soar_agent : m_agents)
    {
      sml::Agent *raw = soar_agent->getSmlAgent();
      if (!raw)
      {
        continue;
      }
      if (!raw->SpawnDebugger())
      {
        all_spawned = false;
        if (!failed_agents.empty())
        {
          failed_agents += ", ";
        }
        failed_agents += raw->GetAgentName();
      }
    }

    if (!all_spawned)
    {
      std::string err_msg =
          "Unable to open Soar Java debugger for agents: " + failed_agents +
          ". Is Java and the Soar debugger installed?";
      RCLCPP_ERROR_STREAM(get_logger(), err_msg);
      response->success = false;
      response->message = err_msg;
    }
    else
    {
      RCLCPP_INFO_STREAM(get_logger(), "Spawning Soar debugger for all agents.");
      response->success = true;
    }
  }

  void SoarRunner::getSoarKernelStatus(
      [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (m_running.load())
    {
      response->message = "Soar kernel isRunning: true";
    }
    else
    {
      response->message = "Soar kernel isRunning: false";
    }
    response->success = true;
  }

  void SoarRunner::runSoarKernel(
      [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Attempting to run Soar kernel from ROS.");
    startThread();
    if (m_running.load())
    {
      response->message = "Soar kernel isRunning: true";
      response->success = true;
    }
    else
    {
      response->message = "Soar kernel isRunning: false";
      response->success = false;
    }
  }

  void SoarRunner::stopSoarKernel(
      [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
      [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Attempting to stop Soar kernel from ROS.");
    stopThread();
    if (m_running.load())
    {
      response->message = "Soar kernel isRunning: true";
      response->success = false;
    }
    else
    {
      response->message = "Soar kernel isRunning: false";
      response->success = true;
    }
  }

} // namespace soar_ros
