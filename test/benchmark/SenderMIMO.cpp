#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;
using Header = std_msgs::msg::Header;

#define MESSAGES_TO_SEND 1000

class SenderMIMO : public rclcpp::Node
{
public:
    SenderMIMO()
        : Node("sender_mimo")
    {
        this->declare_parameter<int>("frequency", 1.0);
        this->declare_parameter<int>("num_inputs", 3);
        this->get_parameter("frequency", frequency_);
        this->get_parameter("num_inputs", num_inputs_);

        // Create multiple publishers for multiple inputs
        for (int i = 0; i < num_inputs_; i++)
        {
            std::string topic_name = "input" + std::to_string(i);
            publishers_.push_back(this->create_publisher<Header>(topic_name, 10));
        }

        set_timer();
    }

private:
    void set_timer()
    {
        auto period = std::chrono::duration<double>(1.0 / frequency_);
        RCLCPP_INFO(this->get_logger(), "Publishing period: %.6f seconds", period.count());
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&SenderMIMO::publish_header, this));
    }

    void publish_header()
    {
        if (frame_counter_ < MESSAGES_TO_SEND)
        {
            auto timestamp = this->now();

            // Publish to all input topics
            for (int i = 0; i < num_inputs_; i++)
            {
                Header msg;
                msg.frame_id = std::to_string(frame_counter_) + "_" + std::to_string(i);
                msg.stamp = timestamp;
                publishers_[i]->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published header to input%d: frame_id=%s, sec=%d, nanosec=%u",
                            i, msg.frame_id.c_str(), msg.stamp.sec, msg.stamp.nanosec);
            }

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
                                       "Sent " << MESSAGES_TO_SEND << " messages to " << num_inputs_ << " topics. Shutting down.");
                    rclcpp::shutdown();
                });
            timer_->cancel();
        }
    }

    std::vector<rclcpp::Publisher<Header>::SharedPtr> publishers_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer;
    int frequency_ = 1;
    int num_inputs_ = 3;
    uint64_t frame_counter_ = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SenderMIMO>();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return 0;
}
