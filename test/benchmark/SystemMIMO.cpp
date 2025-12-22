#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
using Header = std_msgs::msg::Header;
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "soar_ros/soar_ros.hpp"
#include <sml_Client.h>
#include <fstream>

#define NUM_INPUTS 3
#define NUM_OUTPUTS 3

class Output : public soar_ros::Publisher<Header>
{
public:
    Output(
        sml::Agent *agent, rclcpp::Node::SharedPtr node,
        const std::string &topic, int index)
        : Publisher<Header>(agent, node, topic), index_(index) {}
    ~Output() {}

    Header parse(sml::Identifier *id) override
    {
        Header msg;
        msg.frame_id = id->GetParameterValue("frame_id");
        std::string sec_str = id->GetParameterValue("stamp_sec");
        std::string nanosec_str = id->GetParameterValue("stamp_nanosec");
        msg.stamp.sec = std::stoi(sec_str);
        msg.stamp.nanosec = static_cast<uint32_t>(std::stoul(nanosec_str));
        return msg;
    }

private:
    int index_;
};

class Input : public soar_ros::Subscriber<Header>
{
public:
    Input(
        sml::Agent *agent, rclcpp::Node::SharedPtr node,
        const std::string &topic, int index)
        : Subscriber<Header>(agent, node, topic), index_(index) {}
    ~Input() {}

    void parse(Header msg) override
    {
        sml::Identifier *il = this->m_pAgent->GetInputLink();
        sml::Identifier *pId = il->CreateIdWME(this->m_topic.c_str());
        pId->CreateStringWME("frame_id", msg.frame_id.c_str());
        pId->CreateStringWME("stamp_sec", std::to_string(msg.stamp.sec).c_str());
        pId->CreateStringWME("stamp_nanosec", std::to_string(msg.stamp.nanosec).c_str());
        pId->CreateIntWME("index", index_);
    }

private:
    int index_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const std::string package_name = "soar_ros";
    const std::string share_directory =
        ament_index_cpp::get_package_share_directory(package_name);
    std::string soar_path = share_directory + "/Soar/benchmark_mimo.soar";

    auto node = std::make_shared<soar_ros::SoarRunner>("benchmark_mimo",
                                                       soar_path);

    // Add multiple publishers for multiple outputs
    for (int i = 0; i < NUM_OUTPUTS; i++)
    {
        std::string topic_name = "output" + std::to_string(i);
        std::shared_ptr<soar_ros::Publisher<Header>> p =
            std::make_shared<Output>(node.get()->getAgent(), node, topic_name, i);
        node->addPublisher(p);
    }

    // Add multiple subscribers for multiple inputs
    for (int i = 0; i < NUM_INPUTS; i++)
    {
        std::string topic_name = "input" + std::to_string(i);
        std::shared_ptr<soar_ros::Subscriber<Header>> s =
            std::make_shared<Input>(node.get()->getAgent(), node, topic_name, i);
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
