#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>

using Header = std_msgs::msg::Header;

class ReceiverMIMO : public rclcpp::Node
{
public:
    ReceiverMIMO()
        : Node("receiver_mimo")
    {
        this->declare_parameter<int>("num_outputs", 3);
        this->get_parameter("num_outputs", num_outputs_);
        // Create multiple subscriptions for multiple outputs
        for (int i = 0; i < num_outputs_; i++)
        {
            std::string topic_name = "output" + std::to_string(i);
            subscriptions_.push_back(
                this->create_subscription<Header>(
                    topic_name, 10,
                    [this, i](const Header &msg)
                    {
                        double receive_time = this->now().seconds();
                        RCLCPP_INFO(this->get_logger(),
                                    "Received header on output%d: frame_id=%s, sec=%d, nanosec=%u, receive_time=%.9f",
                                    i, msg.frame_id.c_str(), msg.stamp.sec, msg.stamp.nanosec, receive_time);
                    }));
        }
    }

private:
    std::vector<rclcpp::Subscription<Header>::SharedPtr> subscriptions_;
    int num_outputs_ = 3;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReceiverMIMO>();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
