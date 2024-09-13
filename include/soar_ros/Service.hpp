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

#ifndef SOAR_ROSOS__SERVICE_HPP_
#define SOAR_ROSOS__SERVICE_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "Interface.hpp"
#include "SafeQueue.hpp"

#include "sml_Client.h"

namespace soar_ros
{
template<typename T, typename pRequestType = typename T::Request::SharedPtr,
  typename pResponseType = typename T::Response::SharedPtr>
class Service : public virtual Input<pRequestType>, public virtual Output<pResponseType>,
  public Interface
{
protected:
  typename rclcpp::Service<T>::SharedPtr m_service;
  std::string m_topic;
  rclcpp::Node::SharedPtr m_node;
  sml::Agent * m_pAgent;

  void callback(
    [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
    pRequestType request,
    pResponseType response)
  {
    // send data via createWMEs
    this->m_r2sQueue.push(request);
    // This is a blocking call!
    *response = *(this->m_s2rQueue.pop());
  }

public:
  Service(sml::Agent * agent, rclcpp::Node::SharedPtr node, const std::string & service_name)
  : m_topic(service_name), m_node(node), m_pAgent(agent)
  {
    m_service =
      m_node.get()->create_service<T>(
      m_topic,
      std::bind(
        &Service::callback, this, std::placeholders::_1, std::placeholders::_2,
        std::placeholders::_3));
  }
  ~Service() {}

  virtual void parse(pRequestType msg) = 0;
  pResponseType parse(sml::Identifier * id) override = 0;

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

#endif  // SOAR_ROSOS__SERVICE_HPP_
