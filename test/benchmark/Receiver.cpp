#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

using Header = std_msgs::msg::Header;

class Receiver : public rclcpp::Node {
public:
  Receiver()
  : Node("receiver")
  {
    subscription_ = this->create_subscription<Header>(
      "output", 10,
      [this](const Header & msg) {
        double receive_time = this->now().seconds();
        RCLCPP_INFO(this->get_logger(),
          "Received header: frame_id=%s, sec=%d, nanosec=%u, receive_time=%.9f",
          msg.frame_id.c_str(), msg.stamp.sec, msg.stamp.nanosec, receive_time);
      });
  }

private:
  rclcpp::Subscription<Header>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Receiver>();
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
