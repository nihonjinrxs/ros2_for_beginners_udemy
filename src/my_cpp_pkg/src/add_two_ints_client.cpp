#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class AddTwoIntsClient : public rclcpp::Node
{
public:
  AddTwoIntsClient() : Node("add_two_ints_client")
  {
    client_ = this-> create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  }

  void callAddTwoInts(int a, int b)
  {
    while(!client_->wait_for_service(1.0s))
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
    }

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    client_->async_send_request(request, std::bind(&AddTwoIntsClient::callAddTwoIntsCallback, this, _1));
  }

private:
  void callAddTwoIntsCallback(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Sum: %d", (int)response->sum);
  }

  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClient>();
  node->callAddTwoInts(10, 5);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}