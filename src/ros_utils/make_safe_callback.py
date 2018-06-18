import rospy

def make_safe_callback(f):
    def f_safe(*args, **kwargs):
        try:
            result = f(*args, **kwargs)
        except Exception:
            import traceback
            rospy.logerr(traceback.format_exc())
            rospy.signal_shutdown('Exception in callback')
        return result
    return f_safe
