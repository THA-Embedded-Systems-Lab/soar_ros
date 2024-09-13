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

#include <rclcpp/rclcpp.hpp>

#include <example_interfaces/srv/add_two_ints.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>

#include "soar_ros/soar_ros.hpp"
#include <sml_Client.h>

class TestOutput : public soar_ros::Publisher<std_msgs::msg::String>
{
public:
  TestOutput(
    sml::Agent * agent, rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Publisher<std_msgs::msg::String>(agent, node, topic) {}
  ~TestOutput() {}
  std_msgs::msg::String parse(sml::Identifier * id) override
  {
    std_msgs::msg::String msg;
    msg.data = id->GetParameterValue("data");
    std::cout << id->GetCommandName() << " " << msg.data << std::endl;
    return msg;
  }
};

class TestInput : public soar_ros::Subscriber<std_msgs::msg::String>
{
public:
  TestInput(
    sml::Agent * agent, rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Subscriber<std_msgs::msg::String>(agent, node, topic) {}
  ~TestInput() {}

  void parse(std_msgs::msg::String msg) override
  {
    sml::Identifier * il = this->m_pAgent->GetInputLink();
    sml::Identifier * pId = il->CreateIdWME(this->m_topic.c_str());
    pId->CreateStringWME("data", msg.data.c_str());
  }
};

class TestService
  : public soar_ros::Service<example_interfaces::srv::AddTwoInts>
{
public:
  TestService(
    sml::Agent * agent, rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Service<example_interfaces::srv::AddTwoInts>(agent, node, topic) {}
  ~TestService() {}

  example_interfaces::srv::AddTwoInts::Response::SharedPtr
  parse(sml::Identifier * id) override
  {
    example_interfaces::srv::AddTwoInts::Response::SharedPtr response =
      std::make_shared<example_interfaces::srv::AddTwoInts::Response>();
    auto sum = id->GetParameterValue("sum");
    int32_t num = std::stoi(sum);
    response.get()->sum = num;
    RCLCPP_INFO(m_node->get_logger(), "Computed: Sum=%ld", response.get()->sum);
    std::cout << "Sum=" << response.get()->sum << std::endl;
    return response;
  }

  void
  parse(example_interfaces::srv::AddTwoInts::Request::SharedPtr msg) override
  {
    sml::Identifier * il = getAgent()->GetInputLink();
    sml::Identifier * pId = il->CreateIdWME("AddTwoInts");
    pId->CreateIntWME("a", msg.get()->a);
    pId->CreateIntWME("b", msg.get()->b);
  }
};

class TestClient
  : public soar_ros::Client<example_interfaces::srv::AddTwoInts>
{
public:
  TestClient(
    sml::Agent * agent, rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Client<example_interfaces::srv::AddTwoInts>(agent, node, topic) {}
  ~TestClient() {}

  example_interfaces::srv::AddTwoInts::Request::SharedPtr
  parse(sml::Identifier * id) override
  {
    example_interfaces::srv::AddTwoInts::Request::SharedPtr request =
      std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    auto a = std::stoi(id->GetParameterValue("a"));
    auto b = std::stoi(id->GetParameterValue("b"));
    request.get()->a = a;
    request.get()->b = b;
    RCLCPP_INFO(m_node->get_logger(), "Request computation: %d + %d", a, b);
    return request;
  }

  void
  parse(example_interfaces::srv::AddTwoInts::Response::SharedPtr msg) override
  {
    sml::Identifier * il = getAgent()->GetInputLink();
    sml::Identifier * pId = il->CreateIdWME("AddTwoIntsClient");
    pId->CreateIntWME("sum", msg.get()->sum);
    RCLCPP_INFO(m_node->get_logger(), "Result: %ld", msg.get()->sum);
  }
};

class Trigger : public soar_ros::Subscriber<std_msgs::msg::String>
{
public:
  Trigger(
    sml::Agent * agent, rclcpp::Node::SharedPtr node,
    const std::string & topic)
  : Subscriber<std_msgs::msg::String>(agent, node, topic) {}
  ~Trigger() {}

  void parse(std_msgs::msg::String msg) override
  {
    sml::Identifier * il = this->m_pAgent->GetInputLink();
    sml::Identifier * pId = il->CreateIdWME(this->m_topic.c_str());
    pId->CreateStringWME("data", msg.data.c_str());
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const std::string package_name = "soar_ros";
  const std::string share_directory =
    ament_index_cpp::get_package_share_directory(package_name);

  std::string soar_path = share_directory + "/Soar/main.soar";
  auto node = std::make_shared<soar_ros::SoarRunner>("Test Agent", soar_path);

  std::shared_ptr<soar_ros::Publisher<std_msgs::msg::String>> p =
    std::make_shared<TestOutput>(node.get()->getAgent(), node, "test");
  node->addPublisher(p);

  std::shared_ptr<soar_ros::Subscriber<std_msgs::msg::String>> s =
    std::make_shared<TestInput>(node.get()->getAgent(), node, "testinput");
  node->addSubscriber(s);

  std::shared_ptr<soar_ros::Subscriber<std_msgs::msg::String>> trigger =
    std::make_shared<Trigger>(node.get()->getAgent(), node, "Trigger");
  node->addSubscriber(trigger);

  std::shared_ptr<soar_ros::Service<example_interfaces::srv::AddTwoInts>>
  service = std::make_shared<TestService>(
    node.get()->getAgent(), node,
    "AddTwoInts");
  node->addService(service);

  std::shared_ptr<soar_ros::Client<example_interfaces::srv::AddTwoInts>>
  client = std::make_shared<TestClient>(
    node.get()->getAgent(), node,
    "AddTwoIntsClient");
  node->addClient(client, "AddTwoIntsClient");

  if (!node->get_parameter("debug").as_bool()) {
    node->startThread();
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
