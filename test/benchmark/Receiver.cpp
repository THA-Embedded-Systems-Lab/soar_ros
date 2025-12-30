#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

class Receiver : public rclcpp::Node
{
public:
  Receiver() : Node("receiver")
  {
    this->declare_parameter<int>("num_outputs", 3);
    this->get_parameter("num_outputs", num_outputs_);
    for (int i = 0; i < num_outputs_; i++)
    {
      std::string topic_name = "output" + std::to_string(i);
      subscriptions_.push_back(
          this->create_subscription<std_msgs::msg::String>(topic_name, 10, [this, i](const std_msgs::msg::String& msg) {
            RCLCPP_INFO(this->get_logger(), "Received on output: frame_id=%s", msg.data.c_str());
          }));
    }
  }

private:
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
  int num_outputs_ = 3;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Receiver>();
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
