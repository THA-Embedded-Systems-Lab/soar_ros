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
#include <unordered_set>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
    /// @param auto_delete_soar_io_on_complete If true, automatically remove input
    ///        messages from the input-link when marked complete by output-link.
    SoarAgent(
        sml::Agent *pAgent,
        rclcpp::Node::SharedPtr node,
        bool auto_delete_soar_io_on_complete = true)
        : m_pAgent(pAgent), m_node(node), m_auto_delete_soar_io_on_complete(auto_delete_soar_io_on_complete)
    {
      if (m_auto_delete_soar_io_on_complete)
      {
        loadProductionsFromShared("soar_ros", "Soar/input-output-deletion.soar");
      }
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

      if (m_auto_delete_soar_io_on_complete)
      {
        removeCompletedInput();
      }

      processInput();
    }

    bool loadProductionsFromShared(const std::string share_name, const std::string filename)
    {
      const std::string share_directory = ament_index_cpp::get_package_share_directory(share_name);
      std::string full_path = share_directory + "/" + filename;
      m_pAgent->LoadProductions(full_path.c_str());

      if (m_pAgent->HadError())
      {
        std::string err_msg = m_pAgent->GetLastErrorDescription();
        RCLCPP_ERROR_STREAM(m_node->get_logger(), "Failed loading productions from: " << full_path << ", error: " << err_msg);
        return false;
      }

      RCLCPP_INFO_STREAM(m_node->get_logger(), "Loaded productions from: " << full_path);
      return true;
    }

  private:
    /// @brief Enable automatic deletion of completed input/output messages.
    bool m_auto_delete_soar_io_on_complete;

    /// @brief Pending input removal ids collected from output-link.
    std::vector<long long> pending_input_removals;

    /// @brief Per-agent counter for assigning soar-input-removal-id WMEs.
    long long m_input_removal_counter{0};

    /// @brief Remove completed input messages from the input-link.
    void removeCompletedInput()
    {
      if (!m_auto_delete_soar_io_on_complete)
      {
        return;
      }

      sml::Identifier *input_link = m_pAgent->GetInputLink();
      if (input_link == nullptr)
      {
        return;
      }

      if (pending_input_removals.empty())
      {
        return;
      }

      std::unordered_set<long long> pending_ids(pending_input_removals.begin(), pending_input_removals.end());
      pending_input_removals.clear();

      int child_count = input_link->GetNumberChildren();
      for (int i = 0; i < child_count; ++i)
      {
        sml::WMElement *child = input_link->GetChild(i);
        if (child == nullptr)
        {
          continue;
        }

        auto *message_id = dynamic_cast<sml::Identifier *>(child);
        if (message_id == nullptr)
        {
          continue;
        }

        int message_child_count = message_id->GetNumberChildren();
        for (int j = 0; j < message_child_count; ++j)
        {
          sml::WMElement *message_child = message_id->GetChild(j);
          if (message_child == nullptr)
          {
            continue;
          }

          const char *attribute = message_child->GetAttribute();
          if (attribute == nullptr || std::string(attribute) != "soar-input-removal-id")
          {
            continue;
          }

          std::string removal_id_value;
          message_child->GetValueAsString(removal_id_value);
          try
          {
            long long removal_id = std::stoll(removal_id_value);
            if (pending_ids.find(removal_id) != pending_ids.end())
            {
              child->DestroyWME();
              break;
            }
          }
          catch (const std::exception &)
          {
            continue;
          }
        }
      }
    }

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

        // Intercept internal removal-id commands when auto-delete is enabled
        if (m_auto_delete_soar_io_on_complete && name == "soar-input-removal-id")
        {
          auto value = pId->GetParameterValue("value");
          pending_input_removals.push_back(std::stoll(value));
          pId->AddStatusComplete();
          continue;
        }

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

      if (m_auto_delete_soar_io_on_complete)
      {
        sml::Identifier *input_link = m_pAgent->GetInputLink();
        if (input_link != nullptr)
        {
          int child_count = input_link->GetNumberChildren();
          for (int i = 0; i < child_count; ++i)
          {
            sml::WMElement *child = input_link->GetChild(i);
            if (child == nullptr)
            {
              continue;
            }

            auto *message_id = dynamic_cast<sml::Identifier *>(child);
            if (message_id == nullptr)
            {
              continue;
            }

            bool has_removal_id = false;
            int message_child_count = message_id->GetNumberChildren();
            for (int j = 0; j < message_child_count; ++j)
            {
              sml::WMElement *message_child = message_id->GetChild(j);
              if (message_child == nullptr)
              {
                continue;
              }
              const char *attribute = message_child->GetAttribute();
              if (attribute != nullptr && std::string(attribute) == "soar-input-removal-id")
              {
                has_removal_id = true;
                break;
              }
            }

            if (!has_removal_id)
            {
              message_id->CreateIntWME("soar-input-removal-id", m_input_removal_counter);
              ++m_input_removal_counter;
            }
          }
        }
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
