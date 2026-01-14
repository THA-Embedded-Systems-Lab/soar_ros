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
#include <rclcpp_action/rclcpp_action.hpp>

#include <example_interfaces/action/fibonacci.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "soar_ros/soar_ros.hpp"
#include "soar_ros/ActionClient.hpp"
#include <sml_Client.h>

class FibonacciActionClient : public soar_ros::ActionClient<example_interfaces::action::Fibonacci>
{
public:
  using ActionClient<example_interfaces::action::Fibonacci>::ActionClient;

protected:
  // Parse goal from Soar output-link
  std::shared_ptr<example_interfaces::action::Fibonacci::Goal> parse(sml::Identifier* id) override
  {
    auto goal = std::make_shared<example_interfaces::action::Fibonacci::Goal>();

    RCLCPP_INFO(rclcpp::get_logger("fibonacci_action_client"), "Parsing Fibonacci goal from Soar output-link...");

    // Navigate through the structured path: goal.request
    int order = id->FindByAttribute("goal", 0)
                    ->ConvertToIdentifier()
                    ->FindByAttribute("request", 0)
                    ->ConvertToIdentifier()
                    ->FindByAttribute("order", 0)
                    ->ConvertToIntElement()
                    ->GetValue();

    goal->order = order;
    RCLCPP_INFO(rclcpp::get_logger("fibonacci_action_client"), "Sending Fibonacci goal with order: %d", goal->order);

    return goal;
  }

  // Parse feedback and send to Soar input-link
  void parse(std::shared_ptr<example_interfaces::action::Fibonacci::Feedback> feedback) override
  {
    sml::Identifier* il = this->getAgent()->GetInputLink();
    sml::Identifier* ros_action_id = il->CreateIdWME("ros-action-client-fibonacci");
    sml::Identifier* feedback_id = ros_action_id->CreateIdWME("feedback");

    // Create sequence array in Soar
    for (size_t i = 0; i < feedback->sequence.size(); ++i)
    {
      std::string index = std::to_string(i);
      feedback_id->CreateIntWME(index.c_str(), feedback->sequence[i]);
    }

    RCLCPP_INFO(rclcpp::get_logger("fibonacci_action_client"), "Received feedback with %zu elements",
                feedback->sequence.size());
  }

  // Parse result and send to Soar input-link
  void parse(rclcpp_action::ClientGoalHandle<example_interfaces::action::Fibonacci>::WrappedResult wrapped) override
  {
    sml::Identifier* il = this->getAgent()->GetInputLink();
    sml::Identifier* ros_action_id = il->CreateIdWME("ros-action-client-fibonacci");
    sml::Identifier* result_id = ros_action_id->CreateIdWME("result");
    sml::Identifier* response_id = result_id->CreateIdWME("response");

    // Create sequence array in Soar
    for (size_t i = 0; i < wrapped.result->sequence.size(); ++i)
    {
      std::string index = std::to_string(i);
      response_id->CreateIntWME(index.c_str(), wrapped.result->sequence[i]);
    }

    RCLCPP_INFO(rclcpp::get_logger("fibonacci_action_client"), "Received final result with %zu elements",
                wrapped.result->sequence.size());

    // Print the final sequence for verification
    std::string sequence_str = "[";
    for (size_t i = 0; i < wrapped.result->sequence.size(); ++i)
    {
      sequence_str += std::to_string(wrapped.result->sequence[i]);
      if (i < wrapped.result->sequence.size() - 1)
      {
        sequence_str += ", ";
      }
    }
    sequence_str += "]";

    RCLCPP_INFO_STREAM(rclcpp::get_logger("fibonacci_action_client"), "Final Fibonacci sequence: " << sequence_str);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const std::string package_name = "soar_ros";
  const std::string share_directory = ament_index_cpp::get_package_share_directory(package_name);

  std::string soar_path = share_directory + "/Soar/actionClient_test.soar";
  auto node = std::make_shared<soar_ros::SoarRunner>("Fibonacci Action Test", soar_path);

  // Create the Fibonacci action client
  std::shared_ptr<soar_ros::ActionClient<example_interfaces::action::Fibonacci>> action_client =
      std::make_shared<FibonacciActionClient>(node.get()->getAgent(), node, "fibonacci");

  // Add the action client to the node
  node->addActionClient(action_client, "ros-action-client-fibonacci");

  if (!node->get_parameter("debug").as_bool())
  {
    node->startThread();
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok())
    executor.spin();

  rclcpp::shutdown();

  return 0;
}
