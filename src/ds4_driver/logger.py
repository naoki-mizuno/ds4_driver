import rospy


class Logger(object):
    def __init__(self, module):
        self.module = module

    @staticmethod
    def new_module(module):
        return Logger(module)

    def error(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg)
        rospy.logerr(msg, *args, **kwargs)

    def warning(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg)
        rospy.logwarn(msg, *args, **kwargs)

    def info(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg)
        rospy.loginfo(msg, *args, **kwargs)

    def debug(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg)
        rospy.logdebug(msg, *args, **kwargs)

    def _format_msg_(self, msg):
        return '[{0}]: {1}'.format(self.module, msg)
