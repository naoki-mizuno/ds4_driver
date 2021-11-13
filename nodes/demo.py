#!/usr/bin/env python3

import rclpy
from ds4_driver.msg import Feedback, Status


class Handler(object):
    def __init__(self, node, status_topic="status", feedback_topic="set_feedback"):
        self._node = node
        self._min_interval = 0.1
        self._last_pub_time = self._node.get_clock().now()
        self._prev = Status()
        self._led = {
            "r": 0,
            "g": 0,
            "b": 0,
            "flashing": False,
        }

        self._pub_feedback = self._node.create_publisher(Feedback, feedback_topic, 0)
        self._sub_status = self._node.create_subscription(
            Status, status_topic, self.cb_status, 0
        )

    def cb_status(self, msg):
        """
        :type msg: Status
        """
        now = self._node.get_clock().now()

        # Commenting out this check for now because to_sec() does not seem to work for rclpy Duration objects
        # This checks to see if the min interval for a callback has been violated.
        # if (now - self._last_pub_time).to_sec() < self._min_interval:
        #     return

        feedback = Feedback()

        feedback.set_rumble = True
        feedback.rumble_small = abs(msg.axis_left_y)
        feedback.rumble_big = abs(msg.axis_right_y)

        # Set LED using the touchpad
        touch = msg.touch0
        if touch.active and msg.button_circle:
            feedback.set_led = True
            self._led["r"] = touch.x
        if touch.active and msg.button_triangle:
            feedback.set_led = True
            self._led["g"] = touch.x
        if touch.active and msg.button_cross:
            feedback.set_led = True
            self._led["b"] = touch.x
        feedback.led_r = float(self._led["r"])
        feedback.led_g = float(self._led["g"])
        feedback.led_b = float(self._led["b"])

        # Turn on/off flash with PS button
        if not self._prev.button_ps and msg.button_ps:
            feedback.set_led_flash = True
            if self._led["flashing"]:
                feedback.led_flash_off = 0.0
            else:
                feedback.led_flash_on = 0.2
                feedback.led_flash_off = 0.2
            self._led["flashing"] = not self._led["flashing"]

        self._pub_feedback.publish(feedback)
        self._prev = msg
        self._last_pub_time = now


def main():
    rclpy.init()
    sample_node = rclpy.create_node("sample")

    Handler(sample_node)

    rclpy.spin(sample_node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
