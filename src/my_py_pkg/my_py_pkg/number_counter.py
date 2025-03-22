#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounter(Node):
  def __init__(self):
    super().__init__("number_counter")
    self.count_ = 0
    self.number_subscriber_ = self.create_subscription(Int64, "number", self.count_number_callback, 10)
    self.number_count_publisher_ = self.create_publisher(Int64, "number_count", 10)
    self.server_ = self.create_service(SetBool, "reset_counter", self.reset_counter_callback)
    self.get_logger().info("NumberCounter node has been started.")

  def count_number_callback(self, msg: Int64):
    self.count_ += msg.data
    count_msg = Int64()
    count_msg.data = self.count_
    self.number_count_publisher_.publish(count_msg)
  
  def reset_counter_callback(self, request: SetBool.Request, response: SetBool.Response):
    if (request.data):
      self.count_ = 0
      response.success = True
      response.message = "Counter reset"
    else:
      response.message = False
      response.message = "Counter not reset"
    return response

def main(args=None):
  rclpy.init(args=args)
  node = NumberCounter()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()