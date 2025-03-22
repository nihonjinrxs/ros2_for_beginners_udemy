#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from functools import partial

class ResetCounterClient(Node):
  def __init__(self):
    super().__init__("reset_counter_client")
    self.client_ = self.create_client(SetBool, "reset_counter")
    self.get_logger().info("ResetCounterClient has been started")

  def call_reset_counter(self, do_reset:bool):
    while not self.client_.wait_for_service(1.0):
      self.get_logger().warn("Waiting for ResetCounter server...")

    request = SetBool.Request()
    request.data = do_reset

    future = self.client_.call_async(request)

    future.add_done_callback(partial(self.call_reset_counter_callback, request=request))

  def call_reset_counter_callback(self, future, request:SetBool.Request):
    response = future.result()
    self.get_logger().info(
      response.message + " (" + str(request.data) + " -> " + str(response.success) + ")"
    )

def main(args=None):
  rclpy.init(args=args)
  node = ResetCounterClient()
  node.call_reset_counter(False)
  node.call_reset_counter(True)
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()