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

#ifndef SOAR_ROS__SUBSCRIBER_HPP_
#define SOAR_ROS__SUBSCRIBER_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>

#include "Interface.hpp"
#include "SafeQueue.hpp"

#include "sml_Client.h"

namespace soar_ros
{
template<typename T>
class Subscriber : public Input<T>, public Interface
{
protected:
  typename rclcpp::Subscription<T>::SharedPtr sub;
  std::string m_topic;
  rclcpp::Node::SharedPtr m_node;
  sml::Agent * m_pAgent;

public:
  Subscriber(sml::Agent * agent, rclcpp::Node::SharedPtr node, const std::string & topic)
  : Input<T>(), m_topic(topic), m_node(node), m_pAgent(agent)
  {
    sub =
      m_node->create_subscription<T>(
      topic, 10,
      std::bind(&Subscriber::callback, this, std::placeholders::_1));
  }
  ~Subscriber() {}

  void callback(const T & msg)
  {
    RCLCPP_INFO(m_node->get_logger(), "Received subscription msg on %s", m_topic.c_str());
    this->m_r2sQueue.push(msg);
  }

  std::string getTopic() override
  {
    return m_topic;
  }

  sml::Agent * getAgent() override
  {
    return m_pAgent;
  }
};
}  // namespace soar_ros

#endif  // SOAR_ROS__SUBSCRIBER_HPP_
