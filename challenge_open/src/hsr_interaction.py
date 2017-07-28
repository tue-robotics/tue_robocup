# ROS
import rospy
import smach
import std_msgs.msg


class HsrInteraction(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])

        self.robot = robot

        self._hsr_pub = rospy.Publisher('/hsrb/trigger', std_msgs.msg.String, queue_size=1)
        rospy.Subscriber("/amigo/trigger", std_msgs.msg.String, self._trigger_callback)

        self._active = False

        self._trigger_string = "hsrdone"

    def execute(self, userdata):

        rospy.logwarn("To stop this state manually, enter 'amigo-trigger-command {}'".format(self._trigger_string))

        self._active = True

        # cycle(3)
        msg = std_msgs.msg.String()
        msg.data = "cycle(3)"

        self.robot.speech.speak("Hey buddy, can you give me some drinks")

        # Wait until trigger has been received
        rate = rospy.Rate(2.0)
        while self._active and not rospy.is_shutdown():
            rate.sleep()

        # See something nice
        self.robot.speech.speak("Thanks buddy. Guys, I'm on my way")

        return "done"

    def _trigger_callback(self, msg):
        """ Callback function for trigger topic. If the specified message is returned, this state will exit
        :param msg: string message
        """
        if msg.data == self._trigger_string:
            rospy.loginfo("Stopping order counter by external trigger")
            self._active = False
