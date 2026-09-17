#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

class Sender : public rclcpp::Node
{
public:
  Sender()
  : Node("sender")
  {
    this->declare_parameter<int>("frequency", 1);
    this->declare_parameter<int>("num_inputs", 2);
    this->declare_parameter<int>("messages_to_send", 10);
    this->declare_parameter<bool>("staircase_mode", false);
    this->declare_parameter<int>("staircase_start_freq", 1000);
    this->declare_parameter<int>("staircase_end_freq", 3000);
    this->declare_parameter<int>("staircase_step_freq", 200);
    this->declare_parameter<double>("staircase_hold_duration", 4.0);
    this->declare_parameter<double>("staircase_recovery_duration", 1.0);
    this->get_parameter("frequency", frequency_);
    this->get_parameter("num_inputs", num_inputs_);
    this->get_parameter("messages_to_send", messages_to_send_);
    this->get_parameter("staircase_mode", staircase_mode_);
    this->get_parameter("staircase_start_freq", staircase_start_freq_);
    this->get_parameter("staircase_end_freq", staircase_end_freq_);
    this->get_parameter("staircase_step_freq", staircase_step_freq_);
    this->get_parameter("staircase_hold_duration", staircase_hold_duration_);
    this->get_parameter("staircase_recovery_duration", staircase_recovery_duration_);

    for (int i = 0; i < num_inputs_; i++) {
      std::string topic_name = "input" + std::to_string(i);
      rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
      publishers_.push_back(this->create_publisher<std_msgs::msg::String>(topic_name, qos));
      while (!publishers_.back()->get_subscription_count()) {
        RCLCPP_INFO(this->get_logger(), "Waiting for subscribers on topic '%s'...",
          topic_name.c_str());
        rclcpp::sleep_for(100ms);
      }
    }

    if (staircase_mode_) {
      RCLCPP_INFO(this->get_logger(),
        "Staircase mode: %d -> %d Hz, step %d Hz, hold %.1fs, recovery %.1fs",
        staircase_start_freq_, staircase_end_freq_, staircase_step_freq_,
        staircase_hold_duration_, staircase_recovery_duration_);
      current_frequency_ = staircase_start_freq_;
      start_hold_phase();
    } else {
      set_timer();
    }
  }

private:
  void set_timer()
  {
    if (timer_) {
      timer_->cancel();
    }
    auto period = std::chrono::duration<double>(1.0 / frequency_);
    RCLCPP_INFO(this->get_logger(), "Publishing period: %.6f seconds (%d Hz)", period.count(),
      frequency_);
    timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                     std::bind(&Sender::publish_string, this));
  }

  void start_hold_phase()
  {
    RCLCPP_INFO(this->get_logger(), "Staircase: holding at %d Hz for %.1f s",
      current_frequency_, staircase_hold_duration_);
    frequency_ = current_frequency_;
    set_timer();
    phase_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(staircase_hold_duration_)),
      std::bind(&Sender::end_hold_phase, this));
  }

  void end_hold_phase()
  {
    phase_timer_->cancel();
    timer_->cancel();

    current_frequency_ += staircase_step_freq_;
    if (current_frequency_ > staircase_end_freq_) {
      finish_staircase();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Staircase: recovery for %.1f s before %d Hz",
      staircase_recovery_duration_, current_frequency_);
    phase_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(staircase_recovery_duration_)),
      std::bind(&Sender::start_hold_phase, this));
  }

  void finish_staircase()
  {
    shutdown_timer = this->create_wall_timer(std::chrono::seconds(1), [this]() {
          RCLCPP_INFO_STREAM(this->get_logger(),
                         "Staircase complete. Sent " << frame_id_ << " messages to " <<
            num_inputs_ << " topics. Shutting down.");
          rclcpp::shutdown();
    });
  }

  void publish_string()
  {
    if (staircase_mode_ || frame_id_ < messages_to_send_) {
      for (int i = 0; i < num_inputs_; i++) {
        std_msgs::msg::String msg;
        msg.data = std::to_string(frame_id_) + "_" + std::to_string(i);
        publishers_[i]->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published to input: frame_id=%s", msg.data.c_str());
      }
      frame_id_++;
    }
    if (!staircase_mode_ && frame_id_ == messages_to_send_) {
      shutdown_timer = this->create_wall_timer(std::chrono::seconds(1), [this]() {
            RCLCPP_INFO_STREAM(this->get_logger(),
                           "Sent " << messages_to_send_ << " messages to " << num_inputs_ <<
              " topics. Shutting down.");
            rclcpp::shutdown();
      });
      timer_->cancel();
    }
  }

  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr phase_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer;
  int frequency_ = 1;
  int num_inputs_ = 3;
  int frame_id_ = 0;
  int messages_to_send_ = 1000;
  bool staircase_mode_ = false;
  int staircase_start_freq_ = 1000;
  int staircase_end_freq_ = 3000;
  int staircase_step_freq_ = 200;
  double staircase_hold_duration_ = 4.0;
  double staircase_recovery_duration_ = 1.0;
  int current_frequency_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sender>();
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
