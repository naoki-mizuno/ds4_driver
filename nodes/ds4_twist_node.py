#!/usr/bin/env python

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from ds4_driver.msg import Status


class StatusToTwist(object):
    def __init__(self,node):
        self._node = node
        self._node.declare_parameter('~stamped', False)
        self._node.declare_parameter('~frame_id', 'base_link')
        self._stamped = self._node.get_parameter('~stamped')
        if self._stamped:
            self._cls = TwistStamped
            self._frame_id = self._node.get_parameter('~frame_id').value
        else:
            self._cls = Twist
        self._node.declare_parameter('~inputs', [])
        self._node.declare_parameter('~scales', [])
        self._inputs = self._node.get_parameter('~inputs')
        self._scales = self._node.get_parameter('~scales')

        self._attrs = []
        for attr in Status.__slots__:
            if attr.startswith('axis_') or attr.startswith('button_'):
                self._attrs.append(attr)

        self._pub = self._node.create_publisher(self._cls, 'cmd_vel', 0)
        self._sub = self._node.create_subscription(Status, 'status', self.cb_status, 0)

    def cb_status(self, msg):
        """
        :param msg:
        :type msg: Status
        :return:
        """
        input_vals = {}
        for attr in self._attrs:
            input_vals[attr] = getattr(msg, attr)

        to_pub = self._cls()
        if self._stamped:
            to_pub.header.stamp = self._node.get_clock().now().to_msg()
            to_pub.header.frame_id = self._frame_id
            twist = to_pub.twist
        else:
            twist = to_pub

        for vel_type in self._inputs.value:
            vel_vec = getattr(twist, vel_type)
            for k, expr in self._inputs[vel_type].items():
                scale = self._scales[vel_type].get(k, 1.0)
                val = eval(expr, {}, input_vals)
                setattr(vel_vec, k, scale * val)

        self._pub.publish(to_pub)


def main():
    rclpy.init()
    node = rclpy.create_node('ds4_twist')

    StatusToTwist(node)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
