#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "soar_ros/soar_ros.hpp"
#include <sml_Client.h>

class Trigger : public soar_ros::Subscriber<std_msgs::msg::String>
{
public:
  using soar_ros::Subscriber<std_msgs::msg::String>::Subscriber;

  void parse(std_msgs::msg::String msg) override
  {
    sml::Identifier* il = this->m_pAgent->GetInputLink();
    sml::Identifier* pId = il->CreateIdWME(this->m_topic.c_str());
    pId->CreateStringWME("data", msg.data.c_str());
  }
};

class TestClient : public soar_ros::Client<example_interfaces::srv::AddTwoInts>
{
public:
  using soar_ros::Client<example_interfaces::srv::AddTwoInts>::Client;

  example_interfaces::srv::AddTwoInts::Request::SharedPtr parse(sml::Identifier* id) override
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

  void parse(example_interfaces::srv::AddTwoInts::Response::SharedPtr msg) override
  {
    sml::Identifier* il = getAgent()->GetInputLink();
    sml::Identifier* pId = il->CreateIdWME("AddTwoIntsClient");
    pId->CreateIntWME("sum", msg.get()->sum);
    RCLCPP_INFO(m_node->get_logger(), "Result: %ld", msg.get()->sum);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const std::string package_name = "soar_ros";
  const std::string share_directory = ament_index_cpp::get_package_share_directory(package_name);
  std::string soar_path = share_directory + "/Soar/test_client.soar";

  auto node = std::make_shared<soar_ros::SoarRunner>("TestClient", soar_path);

  std::shared_ptr<soar_ros::Client<example_interfaces::srv::AddTwoInts>> client =
      std::make_shared<TestClient>(node->getAgent(), node, "AddTwoIntsClient");
  node->addClient(client, "AddTwoIntsClient");

  std::shared_ptr<soar_ros::Subscriber<std_msgs::msg::String>> trigger =
      std::make_shared<Trigger>(node.get()->getAgent(), node, "Trigger");
  node->addSubscriber(trigger);

  if (!node->get_parameter("debug").as_bool())
  {
    node->startThread();
  }

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }
  node->stopThread();
  rclcpp::shutdown();
  return 0;
}
