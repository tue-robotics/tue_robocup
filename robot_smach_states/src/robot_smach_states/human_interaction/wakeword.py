import actionlib
import rospy
import smach

from picovoice_msgs.msg import GetKeywordAction, GetKeywordGoal, Keyword


class WaitForWakeWord(smach.State):
    """
    This state will block execution until a wakeword is heard by the picovoice keyword action
    """
    def __init__(self, robot, wakeword, wakeword_url, action_name):
        smach.State.__init__(self, outcomes=['heard', 'failed', 'preempted'])
        self.wakeword=wakeword
        self.wakeword_url = wakeword_url

        self.client = actionlib.SimpleActionClient(action_name, GetKeywordAction)

    def execute(self, userdata=None):
        # send goal
        keyword = Keyword()
        keyword.name = self.wakeword
        keyword.url = self.wakeword_url

        goal = GetKeywordGoal()
        goal.keywords = [keyword]

        self.client.send_goal(goal)

        # await response
        while not rospy.is_shutdown():
            if self.client.wait_for_result(rospy.Duration(0.2)):
                res = self.client.get_result()
                if res.is_understood:
                    return "heard"
                else:
                    return "failed"
        return 'preempted'
