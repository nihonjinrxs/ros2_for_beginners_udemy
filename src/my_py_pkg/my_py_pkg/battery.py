#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future
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
    self.battery_state_simulation_period_end_ = self.get_current_time_seconds() + float(randrange(20, 50))
    self.battery_charge_change_rate_ = 0.00
    self.last_led_update_ = self.get_current_time_seconds() + 1.0
    self.battery_charge_publisher_ = self.create_publisher(
      BatteryChargeState, "battery_charge_state", 10
    )
    self.led_panel_client_ = self.create_client(SetLED, "set_led")
    self.battery_simulation_timer_ = self.create_timer(0.01, self.simulate_battery_use)
    self.battery_and_led_update_timer_ = self.create_timer(1.0, self.maybe_update_led_state)
    self.get_logger().info("Battery node has been started")

  def get_current_time_seconds(self):
    seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
    return seconds + nanoseconds / 1_000_000_000
  
  def simulate_battery_use(self):
    time_now = self.get_current_time_seconds()
    if self.battery_state_simulation_period_end_ <= time_now:
      self.battery_state_simulation_period_end_ = time_now + float(randrange(20, 50))
      self.battery_charge_change_rate_ = float(randrange(10, 1000)) * 0.0001
      self.battery_state = choice(list(self.BatteryState))
      match self.battery_state:
        case self.BatteryState.CHARGING:
          self.get_logger().info("Battery is now charging at an average rate of " +
                                 str(self.battery_charge_change_rate_ * 100.0) +
                                 " percent per second, currently " + str(self.battery_charge) + "%")
        case self.BatteryState.DISCHARGING:
          self.get_logger().info("Battery is now discharging at an average rate of " +
                                 str(self.battery_charge_change_rate_ * 100.0) +
                                 " percent per second, currently " + str(self.battery_charge) + "%")
        case self.BatteryState.STEADY:
          self.get_logger().info("Battery is in steady state, currently " + str(self.battery_charge) + "%")

    match self.battery_state:
      case self.BatteryState.DISCHARGING:
        self.battery_charge -= self.battery_charge_change_rate_
        self.battery_charge = max(self.battery_charge, 0.0)
      case self.BatteryState.CHARGING:
        self.battery_charge += self.battery_charge_change_rate_
        if self.battery_charge >= 100:
          self.battery_state = self.BatteryState.STEADY
          self.get_logger().info("Battery is in steady state")
        self.battery_charge = min(self.battery_charge, 100.0)
      case _:
        pass
  
  def maybe_update_led_state(self):
    time_now = self.get_current_time_seconds()
    self.publish_battery_charge_state()
    if (time_now <= self.last_led_update_ + 5.0):
      while not self.led_panel_client_.wait_for_service(1.0):
        self.get_logger().warn("Waiting for service...")
    
      request = SetLED.Request()
      request.led_number = self.LED_NUMBER
      request.state = self.battery_charge < self.LED_CHARGE_THRESHOLD
      future = self.led_panel_client_.call_async(request)
      future.add_done_callback(partial(self.maybe_update_led_state_callback, request=request))
  
  def maybe_update_led_state_callback(self, future: Future, request: SetLED.Request):
    response = future.result()
    led_requested = "LED ID " + str(request.led_number)
    led_state = "on" if request.state is not None and request.state else "off"
    success = "success" if response.success is not None and response.success else "failure"
    self.get_logger().info("Sent update to LED Panel to turn " +
                           led_requested + " " + led_state + ": " + success)
    self.last_led_update_ = self.get_current_time_seconds()

  def publish_battery_charge_state(self):
    msg = BatteryChargeState()
    msg.percent_charge = self.battery_charge
    msg.state = self.battery_state.value
    msg.state_name = self.battery_state.name
    self.battery_charge_publisher_.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = Battery()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()