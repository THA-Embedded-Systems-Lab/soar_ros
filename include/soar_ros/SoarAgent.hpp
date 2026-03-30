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

#ifndef SOAR_ROS__SOARAGENT_HPP_
#define SOAR_ROS__SOARAGENT_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sml_Client.h>

#include "ActionClient.hpp"
#include "Client.hpp"
#include "Interface.hpp"
#include "Publisher.hpp"
#include "Service.hpp"
#include "Subscriber.hpp"

namespace soar_ros
{

  /// @brief Per-agent wrapper that owns all ROS I/O wiring for one Soar agent.
  ///
  /// Returned by SoarRunner::addAgent() as a shared_ptr<SoarAgent>.
  /// User code registers interfaces directly on the agent:
  ///
  /// @code
  /// auto agent = node->addAgent("MyAgent", path);
  /// agent->addPublisher(std::make_shared<MyPub>(agent->getSmlAgent(), node, "cmd"));
  /// agent->addSubscriber(std::make_shared<MySub>(agent->getSmlAgent(), node, "topic"));
  /// @endcode
  ///
  /// SoarRunner calls updateWorld() on every SoarAgent each decision cycle via
  /// the smlEVENT_AFTER_ALL_OUTPUT_PHASES callback.
  class SoarAgent
  {
  public:
    /// @param pAgent  Raw sml::Agent owned by the kernel (non-owning).
    /// @param node    Shared ptr to SoarRunner (creates ROS primitives).
    SoarAgent(
        sml::Agent *pAgent,
        rclcpp::Node::SharedPtr node)
        : m_pAgent(pAgent), m_node(node)
    {
    }

    SoarAgent(const SoarAgent &) = delete;
    SoarAgent &operator=(const SoarAgent &) = delete;

    // ---- Accessors -----------------------------------------------------------

    /// @brief Raw sml::Agent pointer (lifetime managed by the Soar kernel).
    sml::Agent *getSmlAgent() const { return m_pAgent; }

    /// @brief The ROS node this agent is attached to.
    rclcpp::Node::SharedPtr getNode() const { return m_node; }

    // ---- add* wiring ---------------------------------------------------------

    /// @brief Register a Publisher; topic name is used as the Soar command name.
    template <typename T>
    bool addPublisher(std::shared_ptr<Publisher<T>> output)
    {
      m_outputs[output->getTopic()] = output;
      return true;
    }

    /// @brief Register a Publisher with an explicit Soar output-link command name.
    template <typename T>
    bool addPublisher(std::shared_ptr<Publisher<T>> output, const std::string &commandName)
    {
      m_outputs[commandName] = output;
      return true;
    }

    /// @brief Register a Subscriber.
    template <typename T>
    bool addSubscriber(std::shared_ptr<Subscriber<T>> input)
    {
      input->subscribe(m_node);
      m_inputs.push_back(input);
      return true;
    }

    /// @brief Register a Service; service name is used as the Soar command name.
    template <typename T>
    bool addService(std::shared_ptr<Service<T>> service)
    {
      return addService(service, service->getTopic());
    }

    /// @brief Register a Service with an explicit Soar command name.
    template <typename T>
    bool addService(std::shared_ptr<Service<T>> service, const std::string &commandName)
    {
      m_outputs[commandName] = std::static_pointer_cast<soar_ros::OutputBase>(service);
      m_inputs.push_back(std::static_pointer_cast<soar_ros::InputBase>(service));
      return true;
    }

    /// @brief Register a Client; client name is used as the Soar command name.
    template <typename T>
    bool addClient(std::shared_ptr<Client<T>> client)
    {
      return addClient(client, client->getTopic());
    }

    /// @brief Register a Client with an explicit Soar command name.
    template <typename T>
    bool addClient(std::shared_ptr<Client<T>> client, const std::string &commandName)
    {
      m_outputs[commandName] = std::static_pointer_cast<soar_ros::OutputBase>(client);
      m_inputs.push_back(std::static_pointer_cast<soar_ros::InputBase>(client));
      return true;
    }

    /// @brief Register an ActionClient; action name is used as the command name.
    template <typename T>
    bool addActionClient(std::shared_ptr<ActionClient<T>> action_client)
    {
      return addActionClient(action_client, action_client->getTopic());
    }

    /// @brief Register an ActionClient with an explicit Soar command name.
    template <typename T>
    bool addActionClient(
        std::shared_ptr<ActionClient<T>> action_client,
        const std::string &commandName)
    {
      m_outputs[commandName] =
          std::static_pointer_cast<soar_ros::Output<typename T::Goal::SharedPtr>>(action_client);
      m_inputs.push_back(
          std::static_pointer_cast<
              soar_ros::Input<typename T::Feedback::SharedPtr>>(action_client));
      m_inputs.push_back(
          std::static_pointer_cast<
              soar_ros::Input<
                  typename rclcpp_action::ClientGoalHandle<T>::WrappedResult>>(action_client));
      m_inputs.push_back(
          std::static_pointer_cast<soar_ros::Input<bool>>(action_client));
      return true;
    }

    // ---- Decision-cycle entry point ------------------------------------------

    /// @brief Process output-link commands then flush all input queues.
    ///        Called by SoarRunner::updateWorld() every decision cycle.
    void updateWorld()
    {
      processOutputLinkChanges();
      processInput();
    }

  private:
    void processOutputLinkChanges()
    {
      if (!m_pAgent)
      {
        return;
      }

      int n = m_pAgent->GetNumberCommands();
      for (int i = 0; i < n; ++i)
      {
        sml::Identifier *pId = m_pAgent->GetCommand(i);
        std::string name = pId->GetCommandName();

        auto it = m_outputs.find(name);
        if (it == m_outputs.end())
        {
          RCLCPP_ERROR(
              m_node->get_logger(),
              "[%s] No output mapping for command: %s",
              m_pAgent->GetAgentName(), name.c_str());
          continue;
        }
        it->second->process_s2r(pId);
        pId->AddStatusComplete();
      }
    }

    void processInput()
    {
      if (!m_pAgent)
      {
        return;
      }
      for (const auto &input : m_inputs)
      {
        input->process_r2s();
      }
      m_pAgent->Commit();
    }

    sml::Agent *m_pAgent;
    rclcpp::Node::SharedPtr m_node;

    /// Soar command name -> output handler
    std::map<std::string, std::shared_ptr<soar_ros::OutputBase>> m_outputs;
    /// Input handlers polled each cycle
    std::vector<std::shared_ptr<soar_ros::InputBase>> m_inputs;
  };

} // namespace soar_ros

#endif // SOAR_ROS__SOARAGENT_HPP_
