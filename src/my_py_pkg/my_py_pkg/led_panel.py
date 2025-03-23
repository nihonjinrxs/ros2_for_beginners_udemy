#!/usr/bin/env python3
from random import randrange
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LEDPanelState

class LEDPanel(Node):
  def __init__(self):
    super().__init__("led_panel")
    self.NUM_LEDS = 3
    self.panel_state_ = [False, False, False]
    self.led_panel_state_publisher_ = self.create_publisher(
      LEDPanelState, "led_panel_state", 10
    )
    self.create_timer(1.0, self.timer_callback)

  def timer_callback(self):
    self.publish_led_panel_state()
    # temporarily update state to see changes
    led_number = randrange(self.NUM_LEDS)
    self.set_led(led_number, not self.panel_state_[led_number])
  
  def set_led(self, led_number: int, state: bool):
    if (led_number < 0):
      led_number = 0
    elif (led_number > self.NUM_LEDS):
      led_number = led_number % self.NUM_LEDS
    self.panel_state_[led_number] = state

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