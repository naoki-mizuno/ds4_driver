from ds4_driver.controller import Controller

from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedback
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import Imu
from ds4_driver_msgs.msg import Feedback
from ds4_driver_msgs.msg import Report
from ds4_driver_msgs.msg import Status

import copy
import math


class ControllerRos(Controller):
    def __init__(self, node):
        super(ControllerRos, self).__init__()

        self.node = node
        self._logger = self.node.get_logger()

        self.node.declare_parameter("use_standard_msgs", False)
        self.node.declare_parameter("deadzone", 0.1)
        self.node.declare_parameter("frame_id", "ds4")
        self.node.declare_parameter("imu_frame_id", "ds4_imu")
        self.node.declare_parameter("autorepeat_rate", 0.0)
        self.node.declare_parameter("max_status_rate", 100.0)

        self.use_standard_msgs = self.node.get_parameter("use_standard_msgs").value
        self.deadzone = self.node.get_parameter("deadzone").value
        self.frame_id = self.node.get_parameter("frame_id").value
        self.imu_frame_id = self.node.get_parameter("imu_frame_id").value
        # Only publish Joy messages on change
        self._autorepeat_rate = self.node.get_parameter("autorepeat_rate").value
        self._max_status_rate = self.node.get_parameter("max_status_rate").value
        self._prev_joy = None

        self.stop_rumble_timer = None

        self._last_status_publish_time = None

        # Use ROS-standard messages (like sensor_msgs/Joy)
        if self.use_standard_msgs:
            self.pub_report = self.node.create_publisher(Report, "raw_report", 0)
            self.pub_battery = self.node.create_publisher(BatteryState, "battery", 0)
            self.pub_joy = self.node.create_publisher(Joy, "joy", 0)
            self.pub_imu = self.node.create_publisher(Imu, "imu", 0)
            self.sub_feedback = self.node.create_subscription(
                JoyFeedbackArray, "set_feedback", self.cb_joy_feedback, 0
            )

            if self._autorepeat_rate != 0:
                period = 1.0 / self._autorepeat_rate
                self.node.create_timer(period, self.cb_joy_pub_timer)
        else:
            self.pub_status = self.node.create_publisher(Status, "status", 1)
            self.sub_feedback = self.node.create_subscription(
                Feedback, "set_feedback", self.cb_feedback, 0
            )

    def cb_report(self, report):
        """
        Callback method for ds4drv event loop
        :param report:
        :return:
        """
        now = self.node.get_clock().now()
        if self._max_status_rate > 0 and self._last_status_publish_time is not None:
            dt = (now - self._last_status_publish_time).nanoseconds / 1e9
            if dt < (1 / self._max_status_rate):
                return

        report_msg = Report()
        report_msg.header.frame_id = self.frame_id
        report_msg.header.stamp = now.to_msg()
        for attr in dir(report):
            if attr.startswith("_"):
                continue
            if hasattr(report_msg, attr):
                val = getattr(report, attr)
                setattr(report_msg, attr, val)
        # Fix (potentially) incorrect data reported from device
        imu_data = Controller.get_imu_data(report)
        report_msg.lin_acc_x = imu_data["lin_acc"]["x"]
        report_msg.lin_acc_y = imu_data["lin_acc"]["y"]
        report_msg.lin_acc_z = imu_data["lin_acc"]["z"]
        report_msg.ang_vel_x = imu_data["ang_vel"]["x"]
        report_msg.ang_vel_y = imu_data["ang_vel"]["y"]
        report_msg.ang_vel_z = imu_data["ang_vel"]["z"]

        status_msg = self._report_to_status_(report_msg, self.deadzone)
        status_msg.imu.header.frame_id = self.imu_frame_id

        if self.use_standard_msgs:
            battery_msg = self._status_to_battery_(status_msg)
            joy_msg = self._status_to_joy_(status_msg)
            imu_msg = self._status_to_imu_(status_msg)
            self.pub_report.publish(report_msg)
            self.pub_battery.publish(battery_msg)
            if (
                self._prev_joy is None
                or joy_msg.axes != self._prev_joy.axes
                or joy_msg.buttons != self._prev_joy.buttons
            ):
                self.pub_joy.publish(joy_msg)
            self.pub_imu.publish(imu_msg)

            self._prev_joy = joy_msg
        else:
            self.pub_status.publish(status_msg)

        self._last_status_publish_time = now

    def cb_feedback(self, msg):
        """
        Callback method for ds4_driver/Feedback
        :param msg:
        :type msg: Feedback
        :return:
        """
        if self.device is None:
            self._logger.warning("No Device")
            return

        def to_int(v):
            return int(v * 255)

        try:
            self.control(
                # LED color
                led_red=to_int(msg.led_r) if msg.set_led else None,
                led_green=to_int(msg.led_g) if msg.set_led else None,
                led_blue=to_int(msg.led_b) if msg.set_led else None,
                # Rumble
                rumble_small=to_int(msg.rumble_small) if msg.set_rumble else None,
                rumble_big=to_int(msg.rumble_big) if msg.set_rumble else None,
                # Flash LED
                flash_on=to_int(msg.led_flash_on / 2.5) if msg.set_led_flash else None,
                flash_off=to_int(msg.led_flash_off / 2.5) if msg.set_led_flash else None,
            )
        except OSError as e:
            self._logger.error(str(e) + " The controller might be disconnected!")

        # Timer to stop rumble
        if msg.set_rumble and msg.rumble_duration != 0:
            self._logger.info(f"Rumbling for {msg.rumble_duration} seconds")
            self.stop_rumble_timer = self.node.create_timer(
                msg.rumble_duration, self.cb_stop_rumble
            )

    def cb_stop_rumble(self):
        try:
            self.control(rumble_small=0, rumble_big=0)
        except OSError as e:
            self._logger.error(str(e) + " The controller might be disconnected!")

        try:
            if self.stop_rumble_timer is not None:
                self.node.destroy_timer(self.stop_rumble_timer)
                self.stop_rumble_timer = None
        except AttributeError:
            # The program exited and self.device was set to None
            pass

    def cb_joy_feedback(self, msg):
        """
        Callback method for sensor_msgs/JoyFeedbackArray
        The message contains the following feedback:
        LED0: red
        LED1: green
        LED2: blue
        RUMBLE0: rumble small
        RUMBLE1: rumble big
        :param msg:
        :type msg: JoyFeedbackArray
        :return:
        """
        feedback = Feedback()
        for jf in msg.array:
            if jf.type == JoyFeedback.TYPE_LED:
                feedback.set_led = True
                if jf.id == 0:
                    feedback.led_r = jf.intensity
                elif jf.id == 1:
                    feedback.led_g = jf.intensity
                elif jf.id == 2:
                    feedback.led_b = jf.intensity
            elif jf.type == JoyFeedback.TYPE_RUMBLE:
                feedback.set_rumble = True
                if jf.id == 0:
                    feedback.rumble_small = jf.intensity
                elif jf.id == 1:
                    feedback.rumble_big = jf.intensity

        feedback.rumble_duration = 1.0

        self.cb_feedback(feedback)

    def cb_joy_pub_timer(self):
        if self._prev_joy is not None:
            self.pub_joy.publish(self._prev_joy)

    @staticmethod
    def _report_to_status_(report_msg, deadzone=0.05):
        status_msg = Status()
        status_msg.header = copy.deepcopy(report_msg.header)

        # Sticks (signs are flipped for consistency with other joypads)
        status_msg.axis_left_x = -ControllerRos._normalize_axis_(
            report_msg.left_analog_x, deadzone
        )
        status_msg.axis_left_y = -ControllerRos._normalize_axis_(
            report_msg.left_analog_y, deadzone
        )
        status_msg.axis_right_x = -ControllerRos._normalize_axis_(
            report_msg.right_analog_x, deadzone
        )
        status_msg.axis_right_y = -ControllerRos._normalize_axis_(
            report_msg.right_analog_y, deadzone
        )

        # Shoulder buttons
        status_msg.axis_l2 = report_msg.l2_analog / 255.0
        status_msg.axis_r2 = report_msg.r2_analog / 255.0

        # Buttons
        status_msg.button_dpad_up = report_msg.dpad_up
        status_msg.button_dpad_down = report_msg.dpad_down
        status_msg.button_dpad_left = report_msg.dpad_left
        status_msg.button_dpad_right = report_msg.dpad_right
        plug_attrs = [attr for attr in dir(report_msg) if attr.startswith("button_")]
        for attr in plug_attrs:
            val = getattr(report_msg, attr)
            setattr(status_msg, attr, val)

        # IMU (X: right, Y: up, Z: towards user)
        status_msg.imu.header = copy.deepcopy(status_msg.header)

        # To m/s^2: 0.98 mg/LSB (BMI055 data sheet Chapter 5.2.1)
        def to_mpss(v):
            return float(v) / (2 ** 13 - 1) * 9.80665 * 0.98

        # To rad/s: 32767: 2000 deg/s (BMI055 data sheet Chapter 7.2.1)
        def to_radps(v):
            return float(v) / (2 ** 15 - 1) * math.pi / 180 * 2000

        # Convert
        status_msg.imu.linear_acceleration.x = to_mpss(report_msg.lin_acc_x)
        status_msg.imu.linear_acceleration.y = to_mpss(report_msg.lin_acc_y)
        status_msg.imu.linear_acceleration.z = to_mpss(report_msg.lin_acc_z)
        status_msg.imu.angular_velocity.x = to_radps(report_msg.ang_vel_x)
        status_msg.imu.angular_velocity.y = to_radps(report_msg.ang_vel_y)
        status_msg.imu.angular_velocity.z = to_radps(report_msg.ang_vel_z)
        # No orientation reported
        status_msg.imu.orientation_covariance[0] = -1

        # Trackpads
        status_msg.touch0.id = report_msg.trackpad_touch0_id
        status_msg.touch0.active = report_msg.trackpad_touch0_active
        status_msg.touch0.x = report_msg.trackpad_touch0_x / float(
            Controller.TOUCHPAD_MAX_X
        )
        status_msg.touch0.y = report_msg.trackpad_touch0_y / float(
            Controller.TOUCHPAD_MAX_Y
        )
        status_msg.touch1.id = report_msg.trackpad_touch1_id
        status_msg.touch1.active = report_msg.trackpad_touch1_active
        status_msg.touch1.x = report_msg.trackpad_touch1_x / float(
            Controller.TOUCHPAD_MAX_X
        )
        status_msg.touch1.y = report_msg.trackpad_touch1_y / float(
            Controller.TOUCHPAD_MAX_Y
        )

        # Battery
        if report_msg.battery == Controller.BATTERY_FULL_CHARGING:
            status_msg.battery_full_charging = True
            status_msg.battery_percentage = 1.0
        else:
            status_msg.battery_full_charging = False
            status_msg.battery_percentage = (
                float(report_msg.battery) / Controller.BATTERY_MAX
            )

        # Plugs
        plug_attrs = [attr for attr in dir(report_msg) if attr.startswith("plug_")]
        for attr in plug_attrs:
            val = getattr(report_msg, attr)
            setattr(status_msg, attr, val)

        return status_msg

    @staticmethod
    def _normalize_axis_(val, deadzone=0.0):
        """
        Convert a value of [0, 255] to [-1.0, 1.0]
        :param val:
        :param deadzone:
        :return:
        """
        norm_val = 2 * (val - 127.5) / 255
        if abs(norm_val) < deadzone:
            return 0.0
        else:
            return norm_val

    @staticmethod
    def _status_to_joy_(status):
        """
        Converts a ds4_driver/Status message to sensor_msgs/Joy
        :param status:
        :type status: Status
        :return:
        """
        msg = Joy()
        msg.header = copy.deepcopy(status.header)
        msg.axes = [
            status.axis_left_x,
            status.axis_left_y,
            status.axis_right_x,
            status.axis_right_y,
            status.axis_l2,
            status.axis_r2,
        ]
        msg.buttons = [
            status.button_square,
            status.button_triangle,
            status.button_circle,
            status.button_cross,
            status.button_l1,
            status.button_l2,
            status.button_r1,
            status.button_r2,
            status.button_share,
            status.button_options,
            status.button_ps,
            status.button_trackpad,
            status.button_l3,
            status.button_r3,
            status.button_dpad_left,
            status.button_dpad_up,
            status.button_dpad_right,
            status.button_dpad_down,
        ]
        return msg

    @staticmethod
    def _status_to_battery_(status):
        """
        Converts a ds4_driver/Status to sensor_msgs/BatteryState
        Reference: https://www.psdevwiki.com/ps4/DualShock_4#Specifications
        :param status:
        :type status: Status
        :return:
        """
        msg = BatteryState()
        msg.header = status.header
        msg.percentage = status.battery_percentage
        msg.voltage = Controller.MAX_VOLTAGE * msg.percentage
        msg.current = float("NaN")
        msg.charge = float("NaN")
        msg.capacity = float("NaN")
        msg.design_capacity = 1.0
        if not status.plug_usb:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        elif not status.battery_full_charging:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif status.battery_full_charging:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        return msg

    @staticmethod
    def _status_to_imu_(status):
        """
        Converts a ds4_driver/Status to sensor_msgs/Imu
        :param status:
        :type status: Status
        :return:
        """
        return status.imu
