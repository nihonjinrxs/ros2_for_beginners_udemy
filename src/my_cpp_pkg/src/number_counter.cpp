#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

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
    reset_counter_server_ = this->create_service<example_interfaces::srv::SetBool>(
        "reset_counter",
        std::bind(&NumberCounter::resetCounterCallback, this, _1, _2)
    );
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

  void resetCounterCallback(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                            const example_interfaces::srv::SetBool::Response::SharedPtr response)
  {
    if (request->data) {
      count_ = 0;
      response->success = true;
      response->message = "Counter reset";
    } else {
      response->success = false;
      response->message =  "Counter not reset";
    }
  }

  int64_t count_;
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_count_publisher_;
  rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr number_subscriber_;
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_counter_server_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberCounter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}