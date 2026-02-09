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
#include "soar_ros/soar_ros.hpp"
#include <sml_Client.h>

class TestService : public soar_ros::Service<example_interfaces::srv::AddTwoInts>
{
public:
  using Service<example_interfaces::srv::AddTwoInts>::Service;

  example_interfaces::srv::AddTwoInts::Response::SharedPtr parse(sml::Identifier * id) override
  {
    example_interfaces::srv::AddTwoInts::Response::SharedPtr response =
      std::make_shared<example_interfaces::srv::AddTwoInts::Response>();
    auto sum = id->GetParameterValue("sum");
    int32_t num = std::stoi(sum);
    response.get()->sum = num;
    RCLCPP_INFO(m_node->get_logger(), "Computed: Sum=%ld", response.get()->sum);
    return response;
  }

  void parse(example_interfaces::srv::AddTwoInts::Request::SharedPtr msg) override
  {
    sml::Identifier * il = getAgent()->GetInputLink();
    sml::Identifier * pId = il->CreateIdWME("AddTwoInts");
    pId->CreateIntWME("a", msg.get()->a);
    pId->CreateIntWME("b", msg.get()->b);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const std::string package_name = "soar_ros";
  const std::string share_directory = ament_index_cpp::get_package_share_directory(package_name);
  std::string soar_path = share_directory + "/Soar/test_service.soar";

  auto node = std::make_shared<soar_ros::SoarRunner>();
  auto agent = node->addAgent("TestService", soar_path);
  std::shared_ptr<soar_ros::Service<example_interfaces::srv::AddTwoInts>> service =
    std::make_shared<TestService>(agent, node, "AddTwoInts");
  node->addService(service);

  if (!node->get_parameter("debug").as_bool()) {
    node->startThread();
  }

  rclcpp::spin(node);
  node->stopThread();
  rclcpp::shutdown();
  return 0;
}
