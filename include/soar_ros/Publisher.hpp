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

#ifndef SOAR_ROS__PUBLISHER_HPP_
#define SOAR_ROS__PUBLISHER_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>

#include "Interface.hpp"
#include "SafeQueue.hpp"

#include "sml_Client.h"

namespace soar_ros
{
template <typename T>
class Publisher : public Output<T>, public Interface
{
protected:
  std::atomic<bool> isRunning;
  std::thread publisher;
  std::string m_topic;
  rclcpp::Node::SharedPtr m_node;
  sml::Agent* m_pAgent;
  rclcpp::CallbackGroup::SharedPtr m_callback_group;

public:
  Publisher(sml::Agent* agent, rclcpp::Node::SharedPtr node, const std::string& topic,
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default)),
            rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
    : Output<T>(), isRunning(true), m_topic(topic), m_node(node), m_pAgent(agent), m_callback_group(callback_group)
  {
    if (!m_callback_group)
    {
      m_callback_group = m_node->get_node_base_interface()->get_default_callback_group();
    }
    rclcpp::PublisherOptions options;
    options.callback_group = m_callback_group;
    pub = m_node->create_publisher<T>(m_topic, qos, options);
    publisher = std::thread(&Publisher<T>::run, this);
  }
  ~Publisher()
  {
    isRunning.store(false);
    if (publisher.joinable())
    {
      publisher.join();
    }
  }
  typename rclcpp::Publisher<T>::SharedPtr pub;

  void run()
  {
    while (isRunning.load())
    {
      auto msg = this->m_s2rQueue.pop();
      RCLCPP_INFO(m_node->get_logger(), "Sending on %s", m_topic.c_str());
      pub->publish(msg);
    }
  }

  T parse(sml::Identifier* id) override = 0;

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

#endif  // SOAR_ROS__PUBLISHER_HPP_
