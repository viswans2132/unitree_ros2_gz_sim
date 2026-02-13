#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class RobotDescriptionPublisher : public rclcpp::Node
{
public:
  RobotDescriptionPublisher()
  : Node("robot_description_publisher")
  {
    this->declare_parameter<std::string>("robot_description", "");
    const auto urdf = this->get_parameter("robot_description").as_string();

    if (urdf.empty()) {
      RCLCPP_FATAL(this->get_logger(), "robot_description parameter is empty. Nothing to publish.");
      throw std::runtime_error("robot_description is empty");
    }

    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();

    pub_ = this->create_publisher<std_msgs::msg::String>("/robot_description", qos);

    msg_ = std::make_shared<std_msgs::msg::String>();
    msg_->data = urdf;

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RobotDescriptionPublisher::on_timer, this)
    );

    RCLCPP_INFO(this->get_logger(),
      "Publishing robot_description (%zu chars) on /robot_description at 2 Hz.",
      msg_->data.size());
  }

private:
  void on_timer()
  {
    pub_->publish(*msg_);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::shared_ptr<std_msgs::msg::String> msg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RobotDescriptionPublisher>());
  } catch (const std::exception & e) {
    // already logged
  }
  rclcpp::shutdown();
  return 0;
}
