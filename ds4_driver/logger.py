import rclpy


class Logger(object):
    def __init__(self, module):
        self.module = module

    @staticmethod
    def new_module(module):
        return Logger(module)

    def error(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rclpy.logging.get_logger("ds4drv").error(msg)

    def warning(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rclpy.logging.get_logger("ds4drv").warning(msg)

    def info(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rclpy.logging.get_logger("ds4drv").info(msg)

    def debug(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rclpy.logging.get_logger("ds4drv").debug(msg)

    def _format_msg_(self, msg, *args, **kwargs):
        msg = msg.format(*args, **kwargs)
        return "[{0}]: {1}".format(self.module, msg)
