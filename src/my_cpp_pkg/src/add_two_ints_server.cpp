#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;

class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer() : Node("add_two_ints_server")
  {
    server_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints",
        std::bind(&AddTwoIntsServer::addTwoIntsCallback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "AddTwoInts server has been started.");
  }

private:
  void addTwoIntsCallback(const example_interfaces::srv::AddTwoInts::Request::SharedPtr request,
                          const example_interfaces::srv::AddTwoInts::Response::SharedPtr response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)response->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}