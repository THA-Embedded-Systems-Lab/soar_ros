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

// Test: two agents subscribed to the same ROS 2 topic both receive the
// message on their respective input links and echo it to their output links.
//
// Both agents load the same Soar productions (test_shared_input.soar).
// Each agent has its own Publisher that prints a tag prefixed with the agent
// name so the Python launch test can assert both agents produced the output.

#include <fstream>
#include <iostream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sml_Client.h>
#include <std_msgs/msg/string.hpp>

#include "soar_ros/soar_ros.hpp"

// ---------------------------------------------------------------------------
// Shared input subscriber — identical for both agents
// ---------------------------------------------------------------------------
class SharedInputSubscriber : public soar_ros::Subscriber<std_msgs::msg::String>
{
public:
    using Subscriber<std_msgs::msg::String>::Subscriber;

    void parse(std_msgs::msg::String msg) override
    {
        soar_ros::msg::toSoar(this->m_pAgent->GetInputLink(), this->m_topic.c_str(), msg);
    }
};

// ---------------------------------------------------------------------------
// Echo publisher — prints "<AGENT_TAG>:<data>" so the test can verify both
// agents independently processed the message.
// ---------------------------------------------------------------------------
class EchoPublisher : public soar_ros::Publisher<std_msgs::msg::String>
{
public:
    EchoPublisher(
        sml::Agent *agent,
        rclcpp::Node::SharedPtr node,
        const std::string &topic,
        const std::string &agent_tag)
        : soar_ros::Publisher<std_msgs::msg::String>(agent, node, topic),
          m_agent_tag(agent_tag)
    {
    }

    std_msgs::msg::String parse(sml::Identifier *id) override
    {
        std_msgs::msg::String msg;
        msg.data = id->GetParameterValue("data");
        // The Python test asserts for this exact format.
        std::cout << m_agent_tag << ":" << msg.data << std::endl;
        return msg;
    }

private:
    std::string m_agent_tag;
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const std::string share_dir =
        ament_index_cpp::get_package_share_directory("soar_ros");
    const std::string soar_path = share_dir + "/Soar/test_shared_input.soar";

    std::ifstream soar_file(soar_path);
    if (!soar_file.is_open())
    {
        std::cerr << "Error: Soar file not found at " << soar_path << std::endl;
        return 1;
    }

    auto node = std::make_shared<soar_ros::SoarRunner>("soar_ros");

    // Both agents load the same productions.
    auto agent_a = node->addAgent("AgentA", soar_path);
    auto agent_b = node->addAgent("AgentB", soar_path);

    // Each agent subscribes to the same topic independently.
    std::shared_ptr<soar_ros::Subscriber<std_msgs::msg::String>> input_a =
        std::make_shared<SharedInputSubscriber>(
            agent_a->getSmlAgent(), node, "shared_input");
    agent_a->addSubscriber(input_a);

    std::shared_ptr<soar_ros::Subscriber<std_msgs::msg::String>> input_b =
        std::make_shared<SharedInputSubscriber>(
            agent_b->getSmlAgent(), node, "shared_input");
    agent_b->addSubscriber(input_b);

    // Each agent publishes its echo output with a distinct agent tag so the
    // test can verify both agents independently received and processed the msg.
    std::shared_ptr<soar_ros::Publisher<std_msgs::msg::String>> output_a =
        std::make_shared<EchoPublisher>(
            agent_a->getSmlAgent(), node, "shared_echo_a", "SHARED_ECHO_A");
    agent_a->addPublisher(output_a, "shared_echo");

    std::shared_ptr<soar_ros::Publisher<std_msgs::msg::String>> output_b =
        std::make_shared<EchoPublisher>(
            agent_b->getSmlAgent(), node, "shared_echo_b", "SHARED_ECHO_B");
    agent_b->addPublisher(output_b, "shared_echo");

    if (!node->get_parameter("debug").as_bool())
    {
        node->startThread();
    }

    rclcpp::spin(node);
    node->stopThread();
    rclcpp::shutdown();
    return 0;
}
