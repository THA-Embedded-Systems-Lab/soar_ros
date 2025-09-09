#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "soar_ros/soar_ros.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const std::string package_name = "soar_ros";
  const std::string share_directory =
    ament_index_cpp::get_package_share_directory(package_name);

  std::string soar_path = share_directory + "/Soar/main.soar";
  auto node = std::make_shared<soar_ros::SoarRunner>("Main Soar Agent", soar_path);

  if (!node->get_parameter("debug").as_bool()) {
    node->startThread();
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
