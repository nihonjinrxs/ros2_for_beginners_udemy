#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

using namespace std::chrono_literals;

class HardwareStatusPublisher : public rclcpp::Node
{
public:
  HardwareStatusPublisher() : Node("hardware_status_publisher")
  {
    hardware_status_publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>(
        "hardware_status", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&HardwareStatusPublisher::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "HardwareStatusPublisher node has been started");
  }

private:
  void timerCallback()
  {
    this->publishHardwareStatus();
  }

  void publishHardwareStatus()
  {
    auto msg = my_robot_interfaces::msg::HardwareStatus();
    msg.temperature = 68.4;
    msg.are_motors_ready = false;
    msg.debug_message = "Motors are cooling off";
    hardware_status_publisher_->publish(msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr hardware_status_publisher_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareStatusPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}