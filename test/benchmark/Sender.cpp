#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class Sender : public rclcpp::Node
{
public:
  Sender() : Node("sender")
  {
    this->declare_parameter<int>("frequency", 1.0);
    this->declare_parameter<int>("num_inputs", 3);
    this->declare_parameter<int>("messages_to_send", 1000);
    this->get_parameter("frequency", frequency_);
    this->get_parameter("num_inputs", num_inputs_);
    this->get_parameter("messages_to_send", messages_to_send_);

    for (int i = 0; i < num_inputs_; i++)
    {
      std::string topic_name = "input" + std::to_string(i);
      rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
      publishers_.push_back(this->create_publisher<std_msgs::msg::String>(topic_name, qos));
      while (!publishers_.back()->get_subscription_count())
      {
        RCLCPP_INFO(this->get_logger(), "Waiting for subscribers on topic '%s'...", topic_name.c_str());
        rclcpp::sleep_for(100ms);
      }
    }
    set_timer();
  }

private:
  void set_timer()
  {
    auto period = std::chrono::duration<double>(1.0 / frequency_);
    RCLCPP_INFO(this->get_logger(), "Publishing period: %.6f seconds", period.count());
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                                     std::bind(&Sender::publish_string, this));
  }

  void publish_string()
  {
    if (frame_id_ < messages_to_send_)
    {
      for (int i = 0; i < num_inputs_; i++)
      {
        std_msgs::msg::String msg;
        msg.data = std::to_string(frame_id_) + "_" + std::to_string(i);
        publishers_[i]->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published to input: frame_id=%s", msg.data.c_str());
      }
      frame_id_++;
    }
    if (frame_id_ == messages_to_send_)
    {
      shutdown_timer = this->create_wall_timer(std::chrono::seconds(1), [this]() {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Sent " << messages_to_send_ << " messages to " << num_inputs_ << " topics. Shutting down.");
        rclcpp::shutdown();
      });
      timer_->cancel();
    }
  }

  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer;
  int frequency_ = 1;
  int num_inputs_ = 3;
  int frame_id_ = 0;
  int messages_to_send_ = 1000;
};

int main(int argc, char* argv[])
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
