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

#ifndef SOAR_ROS__SOAR_RUNNER_HPP_
#define SOAR_ROS__SOAR_RUNNER_HPP_

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "sml_Client.h"

#include "SoarAgent.hpp"

namespace soar_ros
{

    /// @brief Handle the Soar print event via ROS logger.
    /// @param id          sml event id (unused).
    /// @param pSoarRunner_ Pointer to the owning SoarRunner (for logger access).
    /// @param pAgent      Calling agent.
    /// @param pMessage    Message content.
    void SoarPrintEventHandler(
        [[maybe_unused]] sml::smlPrintEventId id,
        void *pSoarRunner_,
        sml::Agent *pAgent,
        char const *pMessage);

    /// @brief Called by the Soar kernel after every output phase to process I/O.
    ///
    /// Delegates to SoarRunner::updateWorld() which in turn calls
    /// SoarAgent::updateWorld() for every registered agent.
    ///
    /// @param id       Unused.
    /// @param pSoarRunner_ Pointer to the owning SoarRunner.
    /// @param kernel_ptr   Unused.
    /// @param run_flags    Unused.
    void updateEventHandler(
        [[maybe_unused]] sml::smlUpdateEventId id,
        void *pSoarRunner_,
        [[maybe_unused]] sml::Kernel *kernel_ptr,
        [[maybe_unused]] sml::smlRunFlags run_flags);

    /// @brief ROS2 node that manages a Soar kernel and its agents.
    ///
    /// Responsibilities:
    ///  - Own the sml::Kernel and the spin thread.
    ///  - Create sml::Agent instances on request and wrap them in SoarAgent.
    ///  - Expose ROS2 service interfaces to start/stop the kernel and launch
    ///    the Soar Java debugger.
    ///
    /// I/O wiring is done on each SoarAgent returned from addAgent(), not here.
    class SoarRunner : public rclcpp::Node
    {
    public:
        explicit SoarRunner(const std::string &node_name);
        ~SoarRunner();

        /// @brief Create a new Soar agent and return a handle for I/O wiring.
        /// @param agent_name  Name passed to sml::Kernel::CreateAgent().
        /// @param source_file Path to the .soar file to load.  Empty = no load.
        /// @return            Per-agent handle; call add*() methods on it.
        std::shared_ptr<SoarAgent> addAgent(
            const std::string &agent_name,
            const std::string &source_file,
            bool auto_delete_soar_io_on_complete = true);

        /// @brief Start the Soar decision-cycle thread.
        void run();

        /// @brief Start the Soar decision-cycle thread (alias for run()).
        void startThread()
        {
            if (m_running.load())
            {
                RCLCPP_WARN(get_logger(), "runThread already running");
                return;
            }
            m_running.store(true);
            RCLCPP_INFO(get_logger(), "Starting runThread");
            m_soar_thread = std::thread(&SoarRunner::soarRunLoop, this);
        }

        /// @brief Stop and join the Soar decision-cycle thread.
        void stopThread()
        {
            if (!m_running.load())
            {
                RCLCPP_DEBUG(get_logger(), "Run thread already stopped!");
                return;
            }
            m_running.store(false);
            RCLCPP_WARN(get_logger(), "Stopping runThread");
            if (m_soar_thread.joinable())
            {
                RCLCPP_DEBUG(get_logger(), "Waiting for run thread to join");
                m_soar_thread.join();
            }
        }

        /// @brief Process all agents' I/O.  Called via updateEventHandler every
        ///        decision cycle (smlEVENT_AFTER_ALL_OUTPUT_PHASES).
        void updateWorld()
        {
            for (auto &agent : m_agents)
            {
                agent->updateWorld();
            }
        }

        // ---- ROS2 service handlers -------------------------------------------

        /// @brief `/soar_ros/debugger/launch` — stop the kernel thread and
        ///        spawn the Soar Java debugger for all agents.
        void debuggerLaunch(
            [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
            [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        /// @brief `/soar_ros/kernel/status` — report whether the kernel thread
        ///        is currently running.
        void getSoarKernelStatus(
            [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
            [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        /// @brief `/soar_ros/kernel/run` — start the kernel thread via ROS.
        void runSoarKernel(
            [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
            [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        /// @brief `/soar_ros/kernel/stop` — stop the kernel thread via ROS.
        void stopSoarKernel(
            [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
            [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    private:
        sml::Kernel *m_kernel{nullptr};
        std::vector<std::shared_ptr<SoarAgent>> m_agents;

        std::atomic<bool> m_running{false};
        std::thread m_soar_thread;
        bool m_debug{false};

        /// @brief ROS2 service: kernel status.
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_getSoarKernelStatus;
        /// @brief ROS2 service: start kernel.
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_kernelRun;
        /// @brief ROS2 service: stop kernel.
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_kernelStop;
        /// @brief ROS2 service: launch Soar Java debugger.
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_debuggerLaunch;

        /// @brief Thread function: calls m_kernel->RunAllAgents(1) in a loop.
        void soarRunLoop();
    };

} // namespace soar_ros

#endif // SOAR_ROS__SOAR_RUNNER_HPP_
