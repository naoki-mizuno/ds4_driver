#!/usr/bin/env python

from ds4_driver.logger import Logger
from ds4_driver.controller_ros import ControllerRos

from ds4drv.backends import BluetoothBackend, HidrawBackend
from ds4drv.exceptions import BackendError

import rospy

import signal
import sys


class SignalHandler(object):
    def __init__(self, controller):
        self.controller = controller

    def __call__(self, signum, frame):
        rospy.loginfo('Shutting down...')
        self.controller.exit()
        sys.exit(0)


def main():
    rospy.init_node('ds4_driver_node')

    device_addr = rospy.get_param('~device_addr', None)
    backend_type = rospy.get_param('~backend', 'hidraw')

    controller = ControllerRos()

    sigint_handler = SignalHandler(controller)
    # Since backend.devices is a non-ROS iterator that doesn't consider
    # rospy.is_shutdown(), the program freezes upon receiving SIGINT when
    # using rospy.on_shutdown. Thus, we need to define our shutdown sequence
    # using signal.signal as is done in the original ds4drv script.
    signal.signal(signal.SIGINT, sigint_handler)

    if backend_type == 'bluetooth':
        backend = BluetoothBackend(Logger('backend'))
    else:
        backend = HidrawBackend(Logger('backend'))

    try:
        backend.setup()
    except BackendError as err:
        rospy.logerr(err)
        rospy.signal_shutdown(str(err))
        sys.exit(1)

    for device in backend.devices:
        rospy.loginfo('Connected to {0}'.format(device.name))
        if device_addr in (None, '', device.device_addr):
            controller.setup_device(device)
            if not controller.is_alive():
                controller.start()
            controller.loop.register_event('device-report', controller.cb_report)
        else:
            rospy.loginfo("...but it's not the one we're looking for :(")


if __name__ == '__main__':
    main()
