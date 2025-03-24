#!/usr/bin/env python3
from random import randrange
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LEDPanelState
from my_robot_interfaces.srv import SetLED

class LEDPanel(Node):
  def __init__(self):
    super().__init__("led_panel")
    self.NUM_LEDS = 3
    self.panel_state_ = [False, False, False]
    self.led_panel_state_publisher_ = self.create_publisher(
      LEDPanelState, "led_panel_state", 10
    )
    self.set_led_server_ = self.create_service(SetLED, "set_led", self.set_led_callback)
    self.create_timer(5.0, self.timer_callback)

  def timer_callback(self):
    self.publish_led_panel_state()

  def set_led_callback(self, request: SetLED.Request, response: SetLED.Response):
    response.success = self.set_led(request.led_number, request.state)
    return response
  
  def set_led(self, led_number: int, state: bool):
    self.get_logger().warn(
      "Request: led_number = " + str(led_number) + ", state = " + str(state))
    if (led_number < 0):
      return False
    elif (led_number >= self.NUM_LEDS):
      return False
    self.panel_state_[led_number] = state
    self.publish_led_panel_state()
    return True

  def publish_led_panel_state(self):
    msg = LEDPanelState()
    msg.led_states = self.panel_state_
    self.led_panel_state_publisher_.publish(msg)


def main(args=None):
  rclpy.init(args=args)
  node = LEDPanel()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()