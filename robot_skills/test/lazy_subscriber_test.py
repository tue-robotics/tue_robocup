# ROS
import rospy
import std_msgs.msg


class TestObject(object):
    def __init__(self):
        self._active = False
        rospy.Subscriber("lazy_test_topic", std_msgs.msg.Bool, self._callback, queue_size=1)

    def _callback(self, msg):
        print("Yeah, I received {}".format(msg))
        if not self._active:
            print("... but I'm not processing it...")


rospy.init_node("lazy_subscriber_test")
o = TestObject()
rospy.spin()
