from robot_skills.util import kdl_conversions
import robot_smach_states
import smach
from robocup_knowledge import load_knowledge
import math
import rospy

challenge_knowledge = load_knowledge('challenge_open')


# TODO: In order to let this state work smoothly, we should take timestamps into account
# TODO: We could access the pose stamped by self._robot.ssl._msg
class SSLLookatAndRotate(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata):
        yaw = self._robot.ssl.get_last_yaw(1.0)
        look_distance = 2.0
        height = 1.7

        if yaw:
            rospy.loginfo("SSL Yaw: %.2f", yaw)

            lookat_vector = kdl_conversions.VectorStamped(x=look_distance * math.cos(yaw),
                                                          y=look_distance * math.sin(yaw),
                                                          z=height,
                                                          frame_id="/%s/base_link" % self._robot.robot_name).\
                projectToFrame("map", self._robot.tf_listener)

            self._robot.head.look_at_point(lookat_vector, pan_vel=2.0)

            if yaw > math.pi:
                yaw -= 2 * math.pi
            if yaw < -math.pi:
                yaw += 2 * math.pi

            vyaw = 1.0
            self._robot.base.force_drive(0, 0, (yaw / abs(yaw)) * vyaw, abs(yaw) / vyaw)
        else:
            rospy.loginfo("No SSL Yaw found ...")

        return "done"


class SSLDemo(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['done', 'preempted'])

        with self:

            smach.StateMachine.add("LOOKAT_ROTATE", SSLLookatAndRotate(robot),
                                   transitions={'done': 'WAIT_FOR_TRIGGER_TIMEOUT'})

            smach.StateMachine.add("WAIT_FOR_TRIGGER_TIMEOUT",
                                   robot_smach_states.WaitForTriggerTimeout(robot, 0.05, ["continue"],
                                                                            "/%s/trigger" % robot.robot_name),
                                   transitions={"continue": "done",
                                                "timeout": "LOOKAT_ROTATE"})


# ------------------------------------------------------------------------------------------------

def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done'])

    with sm:

        # Start challenge via StartChallengeRobust, skipped atm
        smach.StateMachine.add("SSLDEMO",
                               SSLDemo(robot),
                               transitions={"done": "Done",
                                            "preempted": "Done"})

    return sm


if __name__ == '__main__':
    import robot_smach_states
    rospy.init_node('test_ssl_demo')
    robot_smach_states.util.startup(setup_statemachine, challenge_name="challenge_open")
