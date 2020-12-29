import ds4drv
from ds4drv.eventloop import EventLoop

from threading import Thread
from distutils.version import StrictVersion


# Based on DS4Controller class in __main__.py of ds4drv
class Controller(Thread):
    # Reference: https://www.psdevwiki.com/ps4/DualShock_4#Specifications
    MAX_VOLTAGE = 3.65
    # Reference: https://www.psdevwiki.com/ps4/DS4-USB#cite_note-2
    TOUCHPAD_MAX_X = 1919
    TOUCHPAD_MAX_Y = 942
    BATTERY_FULL_CHARGING = 11
    BATTERY_MAX = 8

    def __init__(self):
        super(Controller, self).__init__(target=self.run)
        self.device = None
        self.loop = EventLoop()

        self._led = (0, 0, 1)
        self._led_flash = (0, 0)

    def fire_event(self, event, *args):
        self.loop.fire_event(event, *args)

    def setup_device(self, device):
        self.device = device
        self.fire_event('device-setup', device)
        self.loop.add_watcher(device.report_fd, self.read_report)

    def cleanup_device(self):
        if self.device is None:
            return

        self.fire_event('device-cleanup')
        self.loop.remove_watcher(self.device.report_fd)
        self.device.close()
        self.device = None

    def read_report(self):
        report = self.device.read_report()

        if not report:
            if report is False:
                return

            self.cleanup_device()
            return

        self.fire_event('device-report', report)

    def run(self):
        self.loop.run()

    def exit(self):
        if self.device is not None:
            self.cleanup_device()
        self.loop.stop()
        if self.is_alive():
            self.join()

    def control(self, led_red=None, led_green=None, led_blue=None,
                rumble_small=None, rumble_big=None,
                flash_on=None, flash_off=None):
        """
        Similar to DS4Device.control but with None as default values
        :param led_red:
        :param led_green:
        :param led_blue:
        :param rumble_small:
        :param rumble_big:
        :param flash_on:
        :param flash_off:
        :return:
        """
        self._led = (
            self._led[0] if led_red is None else led_red,
            self._led[1] if led_green is None else led_green,
            self._led[2] if led_blue is None else led_blue,
        )
        self._led_flash = (
            self._led_flash[0] if flash_on is None else flash_on,
            self._led_flash[1] if flash_off is None else flash_off,
        )
        # Once to change LED flashing state
        self._control()
        # Then to actually execute the control
        self._control(small_rumble=rumble_small if rumble_small is not None else 0,
                      big_rumble=rumble_big if rumble_big is not None else 0)

    def _control(self, **kwargs):
        self.device.control(led_red=self._led[0],
                            led_green=self._led[1],
                            led_blue=self._led[2],
                            flash_led1=self._led_flash[0],
                            flash_led2=self._led_flash[1],
                            **kwargs)

    @staticmethod
    def get_imu_data(report):
        """
        Sets the correct IMU data in the report
        See: https://github.com/chrippa/ds4drv/pull/168
        :param report:
        :return:
        """
        ver_with_bug = StrictVersion('0.5.1')
        current = StrictVersion(ds4drv.__version__)

        # Bug is fixed
        if current > ver_with_bug or not hasattr(report, 'orientation_roll'):
            return {
                'lin_acc': {
                    'x': report.lin_acc_x,
                    'y': report.lin_acc_y,
                    'z': report.lin_acc_z,
                },
                'ang_vel': {
                    'x': report.ang_vel_x,
                    'y': report.ang_vel_y,
                    'z': report.ang_vel_z,
                }
            }
        else:
            return {
                'lin_acc': {
                    # Note: Bit is flipped in ds4drv (not sure why)
                    'x': ~report.orientation_roll,
                    'y': report.orientation_yaw,
                    'z': report.orientation_pitch,
                },
                'ang_vel': {
                    'x': report.motion_y,
                    'y': report.motion_x,
                    'z': report.motion_z,
                }
            }
