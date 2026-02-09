#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "soar_ros/soar_ros.hpp"
#include <sml_Client.h>
#include <fstream>

class AgentAOutput : public soar_ros::Publisher<std_msgs::msg::String>
{
public:
  using Publisher<std_msgs::msg::String>::Publisher;

  std_msgs::msg::String parse(sml::Identifier * id) override
  {
    std_msgs::msg::String msg;
    msg.data = id->GetParameterValue("data");
    std::cout << "AGENT_A_OUTPUT:" << msg.data << std::endl;
    return msg;
  }
};

class AgentAInput : public soar_ros::Subscriber<std_msgs::msg::String>
{
public:
  using Subscriber<std_msgs::msg::String>::Subscriber;

  void parse(std_msgs::msg::String msg) override
  {
    sml::Identifier * il = this->m_pAgent->GetInputLink();
    sml::Identifier * pId = il->CreateIdWME(this->m_topic.c_str());
    pId->CreateStringWME("data", msg.data.c_str());
  }
};

class AgentBOutput : public soar_ros::Publisher<std_msgs::msg::String>
{
public:
  using Publisher<std_msgs::msg::String>::Publisher;

  std_msgs::msg::String parse(sml::Identifier * id) override
  {
    std_msgs::msg::String msg;
    msg.data = id->GetParameterValue("data");
    std::cout << "AGENT_B_OUTPUT:" << msg.data << std::endl;
    return msg;
  }
};

class AgentBInput : public soar_ros::Subscriber<std_msgs::msg::String>
{
public:
  using Subscriber<std_msgs::msg::String>::Subscriber;

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
  const std::string share_directory = ament_index_cpp::get_package_share_directory(package_name);
  std::string soar_path_a = share_directory + "/Soar/test_multi_agent_a.soar";
  std::string soar_path_b = share_directory + "/Soar/test_multi_agent_b.soar";

  std::ifstream soar_file_a(soar_path_a);
  if (!soar_file_a.is_open()) {
    std::cerr << "Error: Soar file not found at " << soar_path_a << std::endl;
    return 1;
  }

  std::ifstream soar_file_b(soar_path_b);
  if (!soar_file_b.is_open()) {
    std::cerr << "Error: Soar file not found at " << soar_path_b << std::endl;
    return 1;
  }

  auto node = std::make_shared<soar_ros::SoarRunner>();
  sml::Agent * agent_a = node->addAgent("AgentA", soar_path_a);
  sml::Agent * agent_b = node->addAgent("AgentB", soar_path_b);

  std::shared_ptr<soar_ros::Publisher<std_msgs::msg::String>> output_a =
    std::make_shared<AgentAOutput>(agent_a, node, "agent_a_output");
  node->addPublisher(output_a, "agent_a");

  std::shared_ptr<soar_ros::Subscriber<std_msgs::msg::String>> input_a =
    std::make_shared<AgentAInput>(agent_a, node, "agent_a_input");
  node->addSubscriber(input_a);

  std::shared_ptr<soar_ros::Publisher<std_msgs::msg::String>> output_b =
    std::make_shared<AgentBOutput>(agent_b, node, "agent_b_output");
  node->addPublisher(output_b, "agent_b");

  std::shared_ptr<soar_ros::Subscriber<std_msgs::msg::String>> input_b =
    std::make_shared<AgentBInput>(agent_b, node, "agent_b_input");
  node->addSubscriber(input_b);

  if (!node->get_parameter("debug").as_bool()) {
    node->startThread();
  }

  rclcpp::spin(node);
  node->stopThread();
  rclcpp::shutdown();
  return 0;
}
