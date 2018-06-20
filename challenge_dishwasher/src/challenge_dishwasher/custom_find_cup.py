import rospy
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from robot_skills.amigo import Amigo
from smach import StateMachine, State

_ = tf2_geometry_msgs


class CustomFindCup(State):
    def __init__(self, robot):
        State.__init__(self, outcomes=['succeeded', 'failed'], output_keys=['position'])

    def execute(self, ud):
        ud.position = PointStamped
        return 'succeeded'


class TestCustomFindCup(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            StateMachine.add("FIND_CUP",
                             CustomFindCup(robot),
                             transitions={'succeeded': 'succeeded',
                                          'failed': 'failed'})


if __name__ == '__main__':
    rospy.init_node('test_open_dishwasher')

    robot = Amigo()
    robot.ed.reset()
    robot.leftArm.reset()
    robot.torso.reset()

    sm = TestCustomFindCup(robot)
    sm.execute()
