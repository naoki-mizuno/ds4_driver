#!/usr/bin/env python

import rospy
from ds4_driver.msg import Feedback, Status


class Handler(object):
    def __init__(self, status_topic='status', feedback_topic='set_feedback'):
        self._min_interval = 0.1
        self._last_pub_time = rospy.Time()
        self._prev = Status()
        self._led = {
            'r': 0,
            'g': 0,
            'b': 0,
            'flashing': False,
        }

        self._pub_feedback = rospy.Publisher(feedback_topic, Feedback, queue_size=1)
        rospy.Subscriber(status_topic, Status, self.cb_status, queue_size=1)

    def cb_status(self, msg):
        """
        :type msg: Status
        """
        now = rospy.Time.now()
        if (now - self._last_pub_time).to_sec() < self._min_interval:
            return

        feedback = Feedback()

        feedback.set_rumble = True
        feedback.rumble_small = abs(msg.axis_left_y)
        feedback.rumble_big = abs(msg.axis_right_y)

        # Set LED using the touchpad
        touch = msg.touch0
        if touch.active and msg.button_circle:
            feedback.set_led = True
            self._led['r'] = touch.x
        if touch.active and msg.button_triangle:
            feedback.set_led = True
            self._led['g'] = touch.x
        if touch.active and msg.button_cross:
            feedback.set_led = True
            self._led['b'] = touch.x
        feedback.led_r = self._led['r']
        feedback.led_g = self._led['g']
        feedback.led_b = self._led['b']

        # Turn on/off flash with PS button
        if not self._prev.button_ps and msg.button_ps:
            feedback.set_led_flash = True
            if self._led['flashing']:
                feedback.led_flash_off = 0
            else:
                feedback.led_flash_on = 0.2
                feedback.led_flash_off = 0.2
            self._led['flashing'] = not self._led['flashing']

        self._pub_feedback.publish(feedback)
        self._prev = msg
        self._last_pub_time = now


def main():
    rospy.init_node('sample')

    Handler()

    rospy.spin()


if __name__ == '__main__':
    main()
