#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
using StringMsg = std_msgs::msg::String;
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "soar_ros/soar_ros.hpp"
#include <sml_Client.h>
#include <fstream>

class Output : public soar_ros::Publisher<StringMsg>
{
public:
  Output(sml::Agent* agent, rclcpp::Node::SharedPtr node, const std::string& topic, int index)
    : Publisher<StringMsg>(agent, node, topic), index_(index)
  {
  }
  ~Output()
  {
  }

  StringMsg parse(sml::Identifier* id) override
  {
    StringMsg msg;
    std::string frame_id = id->GetParameterValue("frame_id");
    msg.data = frame_id;
    return msg;
  }

private:
  int index_;
};

class Input : public soar_ros::Subscriber<StringMsg>
{
public:
  Input(sml::Agent* agent, rclcpp::Node::SharedPtr node, const std::string& topic, int index)
    : Subscriber<StringMsg>(agent, node, topic), index_(index)
  {
  }
  ~Input()
  {
  }

  void parse(StringMsg msg) override
  {
    sml::Identifier* il = this->m_pAgent->GetInputLink();
    sml::Identifier* pId = il->CreateIdWME(this->m_topic.c_str());
    pId->CreateStringWME("frame_id", msg.data.c_str());
    pId->CreateIntWME("index", index_);
  }

private:
  int index_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const std::string package_name = "soar_ros";
  const std::string share_directory = ament_index_cpp::get_package_share_directory(package_name);
  std::string soar_path = share_directory + "/Soar/benchmark.soar";

  auto node = std::make_shared<soar_ros::SoarRunner>();
  auto agent = node->addAgent("benchmark", soar_path);

  node->declare_parameter<int>("num_inputs", 3);
  node->declare_parameter<int>("num_outputs", 3);
  int num_inputs = node->get_parameter("num_inputs").as_int();
  int num_outputs = node->get_parameter("num_outputs").as_int();

  // Add multiple publishers for multiple outputs
  for (int i = 0; i < num_outputs; i++)
  {
    std::string topic_name = "output" + std::to_string(i);
    std::shared_ptr<soar_ros::Publisher<StringMsg>> p =
        std::make_shared<Output>(agent, node, topic_name, i);
    node->addPublisher(p);
  }

  for (int i = 0; i < num_inputs; i++)
  {
    std::string topic_name = "input" + std::to_string(i);
    std::shared_ptr<soar_ros::Subscriber<StringMsg>> s =
        std::make_shared<Input>(agent, node, topic_name, i);
    node->addSubscriber(s);
  }

  if (!node->get_parameter("debug").as_bool())
  {
    node->startThread();
  }

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }
  node->~SoarRunner();
  rclcpp::shutdown();
  return 0;
}
