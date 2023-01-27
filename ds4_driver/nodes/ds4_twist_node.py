#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from ds4_driver_msgs.msg import Status

from collections import defaultdict
import itertools


class StatusToTwist(object):
    def __init__(self, node):
        self._node = node
        self._node.declare_parameter("stamped", False)
        self._node.declare_parameter("frame_id", "base_link")

        self._logger = self._node.get_logger()

        self._stamped = self._node.get_parameter("stamped").value
        self._frame_id = self._node.get_parameter("frame_id").value
        if self._stamped:
            self._cls = TwistStamped
        else:
            self._cls = Twist

        # Automatically create missing keys in a dict
        def make_defaultdict():
            return defaultdict(make_defaultdict)

        param_dict = make_defaultdict()
        param_types = ["inputs", "scales"]
        param_categories = ["angular", "linear"]
        param_axis = ["x", "y", "z"]
        for t, c, a in itertools.product(param_types, param_categories, param_axis):
            param_name = "{}.{}.{}".format(t, c, a)
            if t == "inputs":
                self._node.declare_parameter(param_name, "")
            elif t == "scales":
                self._node.declare_parameter(param_name, 0.0)

            param_value = self._node.get_parameter(param_name).value
            if param_value not in (None, ""):
                param_dict[t][c][a] = param_value

        # Convert back to dict (in case a non-existent key is accessed later)
        self._inputs = {k: dict(v) for k, v in param_dict["inputs"].items()}
        self._scales = {k: dict(v) for k, v in param_dict["scales"].items()}

        if self._inputs == {}:
            msg = "inputs parameter is not specified: not doing anything"
            self._logger.warning(msg)

        self._attrs = []
        for attr in Status.__slots__:
            # ROS2 message slots have a prepended underscore
            if attr.startswith("_axis_") or attr.startswith("_button_"):
                self._attrs.append(attr[1:])  # get rid of the prepended underscore
        self._pub = self._node.create_publisher(self._cls, "cmd_vel", 0)
        self._sub = self._node.create_subscription(Status, "status", self.cb_status, 0)

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

        for vel_type in self._inputs.keys():
            vel_vec = getattr(twist, vel_type)
            for k, expr in self._inputs[vel_type].items():
                scale = self._scales[vel_type].get(k, 1.0)
                if scale is None:
                    scale = 1.0
                try:
                    val = eval(expr, {}, input_vals)
                    setattr(vel_vec, k, scale * val)
                except NameError:
                    # some names are not defined
                    pass

        self._pub.publish(to_pub)


def main():
    rclpy.init()
    node = rclpy.create_node("ds4_twist")

    StatusToTwist(node)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
