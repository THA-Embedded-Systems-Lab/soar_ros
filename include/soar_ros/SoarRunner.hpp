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

#ifndef SOAR_ROS__SOARRUNNER_HPP_
#define SOAR_ROS__SOARRUNNER_HPP_

#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <sml_Client.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "Client.hpp"
#include "Interface.hpp"
#include "Publisher.hpp"
#include "Service.hpp"
#include "Subscriber.hpp"
#include "ActionClient.hpp"

namespace soar_ros
{
/// @brief Handle the soar print event via ROS logger
/// @param id  sml event id
/// @param pSoarRunner_ reference to SoarRunner class to access the logger.
/// @param pAgent Calling agent
/// @param pMessage Message content
void SoarPrintEventHandler([[maybe_unused]] sml::smlPrintEventId id, void * pSoarRunner_,
  sml::Agent * pAgent,
  char const * pMessagee);

/// @brief Called on every loop of the Soar kernel to process input and output
/// link changes
///
/// Calls the SoarRunner::updateWorld() function, which in turn processes all
/// outputs via SoarRunner::processOutputLinkChanges() and attaches new elements
/// to the Soar input-link via SoarRunner::processInput().
///
/// @param id Unused.
/// @param pSoarRunner_ Soar only provides C wrappers for their API and the
/// SoarRunner structure must be referenced via static pointer casting.
/// @param kernel_ptr Unused.
/// @param run_flags Unused.
void updateEventHandler([[maybe_unused]] sml::smlUpdateEventId id, void * pSoarRunner_,
  [[maybe_unused]] sml::Kernel * kernel_ptr, [[maybe_unused]] sml::smlRunFlags run_flags);

/// @brief Singelton class to manage the Soar kernel thread, main ROS interface
/// to run/ stop the kernel and to attach interfaces via a builder pattern, e.g.
/// soar_ros::Publisher(), soar_ros::Subscriber(), soar_ros::Service() and
/// soar_ros::Client().
class SoarRunner : public rclcpp::Node
{
private:
  /// @brief All Soar agents managed by the kernel.
  std::vector<sml::Agent *> agents;

  /// @brief Reference to the Soar kernel instaniated in
  /// SoarRunner::SoarRunner()
  sml::Kernel * pKernel;

  /// @brief Reference to the thread running the Soar instance.
  std::thread runThread;

  /// @brief ROS2 Service which provides the current status of the Soar kernel
  /// (run/ stop) via SoarRunner::getKernelStatus()
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_getSoarKernelStatus;

  /// @brief ROS2 Service to start/ Restart the Soar kernel via
  /// SoarRunner::runSoarKernel()
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_kernelRun;

  /// @brief ROS2 Service to stop the Soar kernel if it is running via
  /// SoarRunner::stopSoarKernel()
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_kernelStop;

  /// @brief ROS2 Service to start the Soar debugger from external programs.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_debuggerLaunch;

  /// @brief All output related ROS2 structures grouped per agent.
  ///
  /// The inner key is the topic or command name specified when registering new
  /// outputs via SoarRunner::addPublisher(), SoarRunner::addService() or any
  /// other class that relies on soar_ros::OutputBase.
  ///
  /// Implemented as map since the SoarRunner::processOutputLinkChanges() looks
  /// up the topic or command name to match the output to the Soar WME.
  std::map<sml::Agent *,
    std::map<std::string, std::shared_ptr<soar_ros::OutputBase>>> outputs_by_agent;

  /// @brief All input related ROS2 structures grouped per agent.
  ///
  /// Inputs are iterated and attached to each agent's input-link.
  std::map<sml::Agent *, std::vector<std::shared_ptr<soar_ros::InputBase>>> inputs_by_agent;

  /// @brief Variable set via ROS2 params.
  bool m_debug;

  /// @brief Enable automatic deletion of completed input/output messages.
  bool m_auto_delete_soar_io_on_complete;

  /// @brief Permanent reference to the output link per agent.
  std::map<sml::Agent *, sml::Identifier *> output_links;

  /// @brief Pending input removal ids collected from output-link.
  std::map<sml::Agent *, std::vector<long long>> pending_input_removals;

  /// @brief Per-agent counter for input removal ids.
  std::map<sml::Agent *, long long> input_removal_counters;

  /// @brief Called in SoarRunner::updateWorld()
  void processOutputLinkChanges(sml::Agent * agent);

  /// @brief Read all input queues and call the process function to
  /// attach the structure to the Soar input link.
  ///
  /// Called in SoarRunner::updateWorld()
  void processInput(sml::Agent * agent);

  /// @brief Remove completed input messages from the input-link.
  void removeCompletedInput(sml::Agent * agent);

  /// @brief Initialize runThread and execute pKernel->RunAllAgents(1) in
  /// separate thread.
  void run()
  {
    while (isRunning.load()) {
      pKernel->RunAllAgents(1);
#ifdef BUILD_BENCHMARK
      RCLCPP_INFO(this->get_logger(), "Soar decision cycle executed");
#endif
    }
  }

  /// @brief Compute filepath for the Soar log in ROS2 ecosystem.
  /// @return Filepath with ISO8601 timestamp in UTC
  std::string getSoarLogFilePath();

  /// @brief Flag to start/ stop the SoarRunner::run() function in a separate
  /// thread started and stopped via SoarRunner::startThread() and
  /// SoarRunner::stopThread()
  std::atomic<bool> isRunning;

public:
  /// @brief Instantiates the Soar kernel and generates ROS2 interfaces. Further
  /// setup in SoarRunner::addAgent() for first agent.
  ///
  ///
  /// @param agent_name The agents name.
  /// @param path_productions Filepath to main *.soar file relative to the
  /// package root.
  /// @throw std::runtime_error on sml::Kernel setup error.
  SoarRunner();

  /// @brief Enable external control to start the debugger and stop the run
  /// thread via
  /// @verbatim embed:rst:inline :doc:`std_srvs:interfaces/srv/Trigger`
  /// @endverbatim via `/soar_ros/debugger/launch`
  ///
  /// The run thread is stopped automatically since the debugger is not
  /// responsive if the thread is running. Reason to be investigated.
  ///
  /// The thread running the Soar kernel is not automatically restarted once the
  /// debugger was closed. Manually restart the thread via the provided
  /// SoarRunner::runSoarKernel() interface.
  ///
  /// @note Launching the Soar debugger without the command requires stopping
  /// the kernel via SoarRunner::stopSoarKernel() related topic. Otherwise, the
  /// debugger will remain frozen/ unresponsive.
  ///
  /// @param request_header
  /// @param request
  /// @param response
  void debuggerLaunch([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// @brief ROS2 interface to get current Soar kernel status (run/ stop) via
  /// @verbatim embed:rst:inline :doc:`std_srvs:interfaces/srv/Trigger`
  /// @endverbatim via `/soar_ros/kernel/status`
  ///
  /// @param request_header Unused.
  /// @param request Empty.
  /// @param response "Soar Kernel isRunning: true" if running, otherwise
  /// "Soarkernel isRunning: false".
  ///
  /// @warning This does not catch the state of the Soar kernel correctly, once
  /// the kernel is started/ stopped via the Soar debugger!
  void getSoarKernelStatus([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// @brief ROS2 Service interface to stop Soar kernel if it is currently
  /// running via
  /// @verbatim embed:rst:inline :doc:`std_srvs:interfaces/srv/Trigger`
  /// @endverbatim via `/soar_ros/kernel/run`
  ///
  /// @param request_header Unused.
  /// @param request Empty.
  /// @param response "Soar kernel isRunning: true" if started succesfully,
  /// otherwise "Soar kernel isRunning: false".
  void runSoarKernel([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// @brief ROS2 Service interface to start (run) Soar kernel if it is
  /// currenlty stopped.
  /// @verbatim embed:rst:inline :doc:`std_srvs:interfaces/srv/Trigger`
  /// @endverbatim via `/soar_ros/kernel/stop`
  ///
  /// @param request_header
  /// @param request Empty.
  /// @param response "Soar Kernel isRunning: false" if stopped succesfully,
  /// otherwise "Soarkernel isRunning: true".
  void stopSoarKernel([[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /// @brief Add a new Soar agent and register callbacks related to ROS.
  /// @note Multiple agents share one kernel and are executed in one loop.
  /// @param agent_name The agents name
  /// @param path_productions Path to the Soar source files.
  /// @warning The path might depend on the install location specified in cmake
  /// sctipts
  /// @throw std::runtime_error on sml::Agent::loadProductions() error.
  /// @return
  sml::Agent * addAgent(const std::string & agent_name, const std::string & path_productions);

  /// @brief Stops the Kernel thread and wait until joined, closes Soar Debugger
  /// in debug mode.
  ~SoarRunner();

  /// @brief Returns all agents managed by the SoarRunner.
  const std::vector<sml::Agent *> & getAgents() const
  {
    return agents;
  }

  /// @brief Adds a soar_ros::Publisher() to the SoarRunner.
  ///
  /// The output command is assumed to be the topic name of the Publisher.
  ///
  /// @tparam T ROS2 message type
  /// @param output
  /// @return
  template<typename T>
  bool addPublisher(std::shared_ptr<Publisher<T>> output)
  {
    auto agent = output.get()->getAgent();
    outputs_by_agent[agent][output.get()->getTopic()] = output;
    return true;
  }

  /// @brief Adds a soar_ros::Publisher() to the SoarRunner.
  /// @tparam T
  /// @param output
  /// @param commandName Matching of the Soar output-link command name, e.g.
  /// io.output-link.move matches to commandName "move".
  /// @return
  template<typename T>
  bool addPublisher(std::shared_ptr<Publisher<T>> output, const std::string & commandName)
  {
    auto agent = output.get()->getAgent();
    outputs_by_agent[agent][commandName] = output;
    return true;
  }

  template<typename T>
  bool addSubscriber(std::shared_ptr<Subscriber<T>> input)
  {
    auto agent = input.get()->getAgent();
    inputs_by_agent[agent].push_back(input);
    return true;
  }

  /// @brief Add a new soar_ros::Service. The callback on the output link is the
  /// service's topic name
  /// @tparam T ROS2 service message type definition
  /// @param service
  /// @return
  template<typename T>
  bool addService(std::shared_ptr<Service<T>> service)
  {
    return addService(service, service.get()->getTopic());
  }

  /// @brief Add a new ROS2 Service with a custom callback command
  /// @tparam T
  /// @param service
  /// @param commandName
  /// @return
  template<typename T>
  bool addService(std::shared_ptr<Service<T>> service, const std::string & commandName)
  {
    auto agent = service.get()->getAgent();
    outputs_by_agent[agent][commandName] = std::static_pointer_cast<soar_ros::OutputBase>(service);
    inputs_by_agent[agent].push_back(std::static_pointer_cast<soar_ros::InputBase>(service));
    return true;
  }

  /// @brief Add a new soar_ros::Client. The callback on the output link is the
  /// service's topic name
  /// @tparam T ROS2 Service Message type definition
  /// @param service
  /// @return
  template<typename T>
  bool addClient(std::shared_ptr<Client<T>> client)
  {
    return addClient(client, client.get()->getTopic());
  }

  /// @brief Add a new soar_ros::Client with a custom callback command
  /// @tparam T
  /// @param service
  /// @param commandName
  /// @return
  template<typename T>
  bool addClient(std::shared_ptr<Client<T>> client, const std::string & commandName)
  {
    auto agent = client.get()->getAgent();
    outputs_by_agent[agent][commandName] = std::static_pointer_cast<soar_ros::OutputBase>(client);
    inputs_by_agent[agent].push_back(std::static_pointer_cast<soar_ros::InputBase>(client));
    return true;
  }

  /// @brief Add a new soar_ros::ActionClient with default command name
  /// @tparam T The action type
  /// @param action_client The action client to add
  /// @return true if successful
  template<typename T>
  bool addActionClient(std::shared_ptr<ActionClient<T>> action_client)
  {
    return addActionClient(action_client, action_client.get()->getTopic());
  }

  /// @brief Add a new soar_ros::ActionClient with a custom callback command
  /// @tparam T The action type
  /// @param action_client The action client to add
  /// @param commandName The command name for Soar output-link
  /// @return true if successful
  template<typename T>
  bool addActionClient(
    std::shared_ptr<ActionClient<T>> action_client,
    const std::string & commandName)
  {
    auto agent = action_client.get()->getAgent();
    outputs_by_agent[agent][commandName] =
      std::static_pointer_cast<soar_ros::Output<typename T::Goal::SharedPtr>>(action_client);
    inputs_by_agent[agent].push_back(
        std::static_pointer_cast<soar_ros::Input<typename T::Feedback::SharedPtr>>(action_client));
    inputs_by_agent[agent].push_back(
        std::static_pointer_cast<soar_ros::Input<typename rclcpp_action::ClientGoalHandle<T>::
      WrappedResult>>(
            action_client));
    inputs_by_agent[agent].push_back(std::static_pointer_cast<soar_ros::Input<bool>>(
        action_client));
    return true;
  }

  /// @brief Creates a new thread based on the SoarRunner::run() function that
  /// executes the Soar kernel.
  ///
  /// Soar has a synchronous threading model,
  /// cf. https://soar.eecs.umich.edu/development/soar/ThreadsInSML/
  void startThread()
  {
    // If thread is already running, skip
    if (isRunning.load() == true) {
      RCLCPP_WARN(this->get_logger(), "runThread already running");
      return;
    } else {
      isRunning.store(true);
    }

    RCLCPP_INFO(this->get_logger(), "Starting runThread");
    runThread = std::thread(&SoarRunner::run, this);
  }

  /// @brief Stops and joins SoarRunner::run() thread of Soar kernel via the
  /// SoarRunner::isRunning flag.
  void stopThread()
  {
    if (isRunning.load() == false) {
      RCLCPP_INFO(this->get_logger(), "Run thread already stopped!");
      return;
    }

    isRunning.store(false);
    RCLCPP_WARN(this->get_logger(), "Stopping runThread");

    if (runThread.joinable()) {
      RCLCPP_INFO(this->get_logger(), "Waiting for run thread to join");
      runThread.join();
    }
  }

  /// @brief Soar update World function executed every step in the Soar
  /// execution cycle in SoarRunner::run()
  ///
  /// Processes all outputs via SoarRunner::processOutputLinkChanges() and
  /// attaches new elements to the Soar input-link via
  /// SoarRunner::processInput().
  void updateWorld()
  {
    for (auto * agent : agents) {
      if (agent == nullptr) {
        continue;
      }

      // Read Soar output
      processOutputLinkChanges(agent);

      if (output_links[agent] == nullptr) {
        output_links[agent] = agent->GetOutputLink();
        RCLCPP_DEBUG(this->get_logger(), "Output link reference invalid!");
      }

      if (m_auto_delete_soar_io_on_complete) {
        removeCompletedInput(agent);
      }
      // Write to Soar input-link
      processInput(agent);
    }
  }
};  // class SoarRunner

}  // namespace soar_ros

#endif  // SOAR_ROS__SOARRUNNER_HPP_
