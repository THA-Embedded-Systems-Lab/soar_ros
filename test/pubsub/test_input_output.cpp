#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "soar_ros/soar_ros.hpp"
#include <sml_Client.h>
#include <fstream>

class TestOutput : public soar_ros::Publisher<std_msgs::msg::String>
{
public:
  using Publisher<std_msgs::msg::String>::Publisher;

  std_msgs::msg::String parse(sml::Identifier* id) override
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
  using Subscriber<std_msgs::msg::String>::Subscriber;

  void parse(std_msgs::msg::String msg) override
  {
    sml::Identifier* il = this->m_pAgent->GetInputLink();
    sml::Identifier* pId = il->CreateIdWME(this->m_topic.c_str());
    pId->CreateStringWME("data", msg.data.c_str());
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const std::string package_name = "soar_ros";
  const std::string share_directory = ament_index_cpp::get_package_share_directory(package_name);
  std::string soar_path = share_directory + "/Soar/test_input_output.soar";
  std::cout << "Soar file path: " << soar_path << std::endl;
  std::ifstream soar_file(soar_path);
  if (!soar_file.is_open())
  {
    std::cerr << "Error: Soar file not found at " << soar_path << std::endl;
    return 1;
  }

  auto node = std::make_shared<soar_ros::SoarRunner>("TestRepublisher", soar_path);

  std::shared_ptr<soar_ros::Publisher<std_msgs::msg::String>> p =
      std::make_shared<TestOutput>(node.get()->getAgent(), node, "test");
  node->addPublisher(p);

  std::shared_ptr<soar_ros::Subscriber<std_msgs::msg::String>> s =
      std::make_shared<TestInput>(node.get()->getAgent(), node, "testinput");
  node->addSubscriber(s);

  if (!node->get_parameter("debug").as_bool())
  {
    node->startThread();
  }

  rclcpp::spin(node);
  node->~SoarRunner();
  rclcpp::shutdown();
  return 0;
}
