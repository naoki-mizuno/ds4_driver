import rospy


class Logger(object):
    def __init__(self, module):
        self.module = module

    @staticmethod
    def new_module(module):
        return Logger(module)

    def error(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rospy.logerr(msg)

    def warning(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rospy.logwarn(msg)

    def info(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rospy.loginfo(msg)

    def debug(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rospy.logdebug(msg)

    def _format_msg_(self, msg, *args, **kwargs):
        msg = msg.format(*args, **kwargs)
        return '[{0}]: {1}'.format(self.module, msg)
