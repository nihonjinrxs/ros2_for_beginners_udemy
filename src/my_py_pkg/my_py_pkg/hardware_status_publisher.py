#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisher(Node):
  def __init__(self):
    super().__init__("hardware_status_publisher")
    self.create_timer(1.0, self.timer_callback)
    self.hardware_status_publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
    self.get_logger().info("HardwareStatusPublisher node has been started")

  def timer_callback(self):
    self.publish_hardware_status()
  
  def publish_hardware_status(self):
    msg = HardwareStatus()
    msg.temperature = 71.2
    msg.are_motors_ready = True
    msg.debug_message = "Nothing special"
    self.hardware_status_publisher_.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = HardwareStatusPublisher()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()