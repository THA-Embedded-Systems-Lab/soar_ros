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

#ifndef SOAR_ROS__CLIENT_HPP_
#define SOAR_ROS__CLIENT_HPP_

namespace soar_ros
{

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "Interface.hpp"
#include "SafeQueue.hpp"
#include "sml_Client.h"

template <typename T, typename pRequestType = typename T::Request::SharedPtr,
          typename pResponseType = typename T::Response::SharedPtr>
class Client : public virtual Output<pRequestType>, public virtual Input<pResponseType>, public Interface
{
protected:
  typename rclcpp::Client<T>::SharedPtr m_client;
  std::string m_topic;
  rclcpp::Node::SharedPtr m_node;
  sml::Agent* m_pAgent;
  std::thread m_send_client_requests;
  std::atomic<bool> isRunning;
  rclcpp::CallbackGroup::SharedPtr m_callback_group;
  rclcpp::QoS m_qos;

  /**
   * @brief Periodically check the m_s2rQueue and m_r2sQueue for input and
   * output.
   *
   */
  void run()
  {
    while (isRunning.load())
    {
      auto timeout_duration = std::chrono::seconds(10);
      auto start_time = std::chrono::steady_clock::now();
      while (!m_client->wait_for_service(std::chrono::seconds(1)))
      {
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(this->m_node->get_logger(), "Interrupted while waiting for the service. Exiting.");
          isRunning.store(false);
          break;
        }

        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time >= timeout_duration)
        {
          RCLCPP_ERROR(this->m_node->get_logger(), "Timeout while waiting for service %s", m_topic.c_str());
          // rclcpp::shutdown();
          // break;
        }

        RCLCPP_WARN(this->m_node->get_logger(), "%s service not available, waiting again...", m_topic.c_str());
      }

      auto msg = this->m_s2rQueue.pop();

      auto result = m_client->async_send_request(msg);

      auto response = result.get();
      this->m_r2sQueue.push(response);
    }
  }

public:
  Client(sml::Agent* agent, rclcpp::Node::SharedPtr node, const std::string& client_name,
         rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default)),
         rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
    : m_topic(client_name), m_node(node), m_pAgent(agent), isRunning(true), m_callback_group(callback_group), m_qos(qos)
  {
    if (!m_callback_group)
    {
      m_callback_group = m_node->get_node_base_interface()->get_default_callback_group();
    }
    m_client = m_node.get()->create_client<T>(client_name, qos);
    m_send_client_requests = std::thread(&Client::run, this);
  }

  ~Client()
  {
    isRunning.store(false);
    if (m_send_client_requests.joinable())
    {
      m_send_client_requests.join();
    }
  }

  virtual void parse(pResponseType msg) = 0;
  pRequestType parse(sml::Identifier* id) override = 0;

  std::string getTopic() override
  {
    return m_topic;
  }

  sml::Agent* getAgent() override
  {
    return m_pAgent;
  }
};
}  // namespace soar_ros

#endif  // SOAR_ROS__CLIENT_HPP_
