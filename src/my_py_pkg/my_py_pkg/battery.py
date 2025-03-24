#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
from my_robot_interfaces.srv import SetLED
from my_robot_interfaces.msg import BatteryChargeState
from random import randrange, choice
from functools import partial

class Battery(Node):
  class BatteryState(Enum):
    STEADY = BatteryChargeState.STEADY
    DISCHARGING = BatteryChargeState.DISCHARGING
    CHARGING = BatteryChargeState.CHARGING

  def __init__(self):
    super().__init__("battery")
    self.LED_NUMBER = 2
    self.LED_CHARGE_THRESHOLD = 30
    self.battery_charge = 100
    self.battery_state = self.BatteryState.STEADY
    self.battery_state_simulation_counter = 0
    self.battery_charge_publisher_ = self.create_publisher(
      BatteryChargeState, "battery_charge_state", 10
    )
    self.led_panel_client_ = self.create_client(SetLED, "set_led")
    self.create_timer(1.0, self.timer_callback)
    self.get_logger().info("Battery node has been started")

  def timer_callback(self):
    self.update_battery_charge()
    self.publish_battery_charge_state()
    self.maybe_update_led_state()
  
  def update_battery_charge(self):
    if self.battery_state_simulation_counter == 0:
      self.battery_state_simulation_counter = randrange(30)
      self.battery_state = choice(list(self.BatteryState))
    else:
      self.battery_state_simulation_counter -= 1

    match self.battery_state:
      case self.BatteryState.DISCHARGING:
        self.battery_charge -= 5
        self.battery_charge = max(self.battery_charge, 0)
      case self.BatteryState.CHARGING:
        self.battery_charge += 12
        if self.battery_charge >= 100:
          self.battery_state = self.BatteryState.STEADY
        self.battery_charge = min(self.battery_charge, 100)
      case _:
        pass
  
  def maybe_update_led_state(self):
    while not self.led_panel_client_.wait_for_service(1.0):
      self.get_logger().warn("Waiting for service...")
    
    request = SetLED.Request()
    request.led_number = self.LED_NUMBER
    request.state = self.battery_charge < self.LED_CHARGE_THRESHOLD
    future = self.led_panel_client_.call_async(request)
    future.add_done_callback(partial(self.maybe_update_led_state_callback, request=request))
  
  def maybe_update_led_state_callback(self, future, request):
    response = future.result()
    led_requested = "LED ID " + str(request.led_number)
    led_state = "on" if request.state else "off"
    success = "success" if response.success else "failure"
    self.get_logger().info("Sent update to LED Panel to turn " +
                           led_requested + " " + led_state + ": " + success)

  def publish_battery_charge_state(self):
    msg = BatteryChargeState()
    msg.percent_charge = self.battery_charge
    msg.state = self.battery_state.value
    self.battery_charge_publisher_.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = Battery()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()