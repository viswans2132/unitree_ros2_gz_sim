#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robot_description_publisher");

  node->declare_parameter<std::string>("robot_description", "");
  const auto urdf = node->get_parameter("robot_description").as_string();

  if (urdf.empty()) {
    RCLCPP_FATAL(node->get_logger(), "robot_description parameter is empty. Nothing to publish.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::QoS qos(1);
  qos.transient_local();
  qos.reliable();

  auto pub = node->create_publisher<std_msgs::msg::String>("/robot_description", qos);

  std_msgs::msg::String msg;
  msg.data = urdf;
  pub->publish(msg);

  RCLCPP_INFO(node->get_logger(), "Published robot_description (%zu chars) to /robot_description (transient_local).", urdf.size());

  // Keep node alive so late subscribers can connect (QoS is transient_local, but staying alive is still fine)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
