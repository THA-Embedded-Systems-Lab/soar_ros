#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;
using Header = std_msgs::msg::Header;

#define MESSAGES_TO_SEND 1000

class Sender : public rclcpp::Node
{
public:
  Sender()
      : Node("sender")
  {
    this->declare_parameter<int>("frequency", 1.0);
    this->get_parameter("frequency", frequency_);
    publisher_ = this->create_publisher<Header>("input", 10);
    set_timer();
  }

private:
  void set_timer()
  {
    auto period = std::chrono::duration<double>(1.0 / frequency_);
    RCLCPP_INFO(this->get_logger(), "Publishing period: %.6f seconds", period.count());
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&Sender::publish_header, this));
  }

  void publish_header()
  {
    if (frame_counter_ < MESSAGES_TO_SEND)
    {
      Header msg;
      msg.frame_id = std::to_string(frame_counter_);
      msg.stamp = this->now();
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published header: frame_id=%s, sec=%d, nanosec=%u",
                  msg.frame_id.c_str(), msg.stamp.sec, msg.stamp.nanosec);
      frame_counter_++;
    }

    if (frame_counter_ == MESSAGES_TO_SEND)
    {
      // Wait 2 seconds, then shutdown
      shutdown_timer = this->create_wall_timer(
          std::chrono::seconds(2),
          [this]()
          {
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Sent " << MESSAGES_TO_SEND << " messages. Shutting down.");
            rclcpp::shutdown();
          });
      timer_->cancel();
    }
  }

  rclcpp::Publisher<Header>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer;
  int frequency_ = 1;
  uint64_t frame_counter_ = 0;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sender>();
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
