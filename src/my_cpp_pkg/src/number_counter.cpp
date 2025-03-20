#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::placeholders;

class NumberCounter : public rclcpp::Node
{
public:
  NumberCounter() : Node("number_counter")
  {
    number_subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
        "number", 10,
        std::bind(&NumberCounter::timerCallbackCountNumber, this, _1)
    );
    number_count_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
    RCLCPP_INFO(this->get_logger(), "NumberCounter node has been started.");
  }

private:
  void timerCallbackCountNumber(const example_interfaces::msg::Int64::SharedPtr msg)
  {
    count_ += msg->data;
    auto count_msg = example_interfaces::msg::Int64();
    count_msg.data = count_;
    number_count_publisher_->publish(count_msg);
  }

  int64_t count_;
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_count_publisher_;
  rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberCounter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}