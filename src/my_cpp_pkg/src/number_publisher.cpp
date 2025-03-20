#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

class NumberPublisher : public rclcpp::Node
{
public:
  NumberPublisher() : Node("number_publisher")
  {
    number_ = 2;
    timer_ = this->create_wall_timer(1.0s, std::bind(&NumberPublisher ::timerCallback, this));
    number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    RCLCPP_INFO(this->get_logger(), "NumberPublisher node has been started.");
  }

private:
  void timerCallback()
  {
    publishNumber();
  }

  void publishNumber()
  {
    auto msg = example_interfaces::msg::Int64();
    msg.data = number_;
    number_publisher_->publish(msg);
  }

  int64_t number_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}