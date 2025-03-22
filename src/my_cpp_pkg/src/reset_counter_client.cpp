#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class ResetCounterClient : public rclcpp::Node
{
public:
  ResetCounterClient() : Node("reset_counter_client")
  {
    client_ = this-> create_client<example_interfaces::srv::SetBool>("reset_counter");
    request_ = std::make_shared<example_interfaces::srv::SetBool::Request>();
    RCLCPP_INFO(this->get_logger(), "ResetCounterClient node started");
  }

  void callResetCounter(bool data)
  {
    while(!client_->wait_for_service(1.0s))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
    }

    request_->data = data;
    client_->async_send_request(request_, std::bind(&ResetCounterClient::callResetCounterCallback, this, _1));
  }

private:
  void callResetCounterCallback(rclcpp::Client<example_interfaces::srv::SetBool>::SharedFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "%s (%s -> %s)", response->message.data(), (request_->data) ? "true" : "false", (response->success) ? "true" : "false");
  }

  rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_;
  example_interfaces::srv::SetBool::Request::SharedPtr request_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ResetCounterClient>();
  node->callResetCounter(false);
  node->callResetCounter(true);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}