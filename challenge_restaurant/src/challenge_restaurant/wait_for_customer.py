#!/usr/bin/python

# System
import math

# ROS
import PyKDL as kdl
import rospy
import smach

# TU/e Robotics
from robot_skills.util.kdl_conversions import frame_stamped, VectorStamped
from hmi import TimeoutException
from geometry_msgs.msg import PointStamped
from tue_msgs.msg import People


class WaitForCustomer(smach.State):
    """ Wait for the waiving person """

    def __init__(self, robot, caller_id, kitchen_designator):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'rejected'])
        self._robot = robot
        self._caller_id = caller_id
        self._kitchen_designator = kitchen_designator
        self._people_sub = rospy.Subscriber(robot.robot_name + '/persons', People, self.people_cb)
        self.rate = 10
        self.head_samples = 20
        self.people_received = People()

    def execute(self, userdata=None):
        """ Does the actual work

        :param userdata:
        :return:
        """

        self._robot.head.reset()
        rospy.sleep(1)

        self._robot.speech.speak("I'm waiting for a waving person")
        waving_persons = []

        look_distance = 3.0  # ToDo: magic number
        look_angles = [0.0,  # ToDo: magic number
                       math.pi / 6,
                       math.pi / 4,
                       math.pi / 2.3,
                       0.0,
                       -math.pi / 6,
                       -math.pi / 4,
                       -math.pi / 2.3]
        head_goals = [VectorStamped(x=look_distance * math.cos(angle),
                                    y=look_distance * math.sin(angle), z=1.3,
                                    frame_id="/%s/base_link" % self._robot.robot_name) for angle in look_angles]

        i = 0
        while not rospy.is_shutdown() and not waving_persons:
            self._robot.head.look_at_point(head_goals[int(math.floor((i/self.head_samples))) % len(head_goals)])
            self._robot.head.wait_for_motion_done()
            i = i + 1

            rospy.sleep(1/self.rate)
            for person in self.people_received.people:
                if {'RWave', 'LWave'}.intersection(set(person.tags)):
                    waving_persons.append(person)

        if not waving_persons:
            return 'aborted'

        rospy.loginfo('waving persons: %s', waving_persons)
        if len(waving_persons) > 1:
            rospy.logwarn('using the first person')

        header = self.people_received.header
        point = waving_persons[0].position
        pose = frame_stamped(header.frame_id, point.x, point.y, point.z)
        rospy.loginfo('update customer position to %s', pose)
        self._robot.ed.update_entity(id=self._caller_id, frame_stamped=pose, type="waypoint")

        # look at the barman
        kitchen_entity = self._kitchen_designator.resolve()
        target_pose = kitchen_entity._pose
        head_target_kdl = target_pose * kdl.Vector(20.0, 0.0, 0.0)
        head_target = VectorStamped(x=head_target_kdl.x(), y=head_target_kdl.y(), z=head_target_kdl.z(),
                                    frame_id="/map")
        # pose = kitchen_entity.pose.extractVectorStamped()
        # pose.vector[2] = 1.5

        self._robot.head.look_at_point(head_target)

        self._robot.speech.speak("I have seen a waving person, should I continue?")

        if self._confirm():
            self._robot.head.cancel_goal()
            return 'succeeded'
        else:
            self._robot.head.cancel_goal()
            return 'rejected'

    def people_cb(self, persons):
        self.people_received = persons

    def _confirm(self):
        cgrammar = """
        C[True] -> amigo take the order
        C[False] -> amigo wait
        """
        for i in range(3):
            try:
                speech_result = self._robot.hmi.query(description="Should I get the order?",
                                                      grammar=cgrammar, target="C")
                return speech_result.semantics
            except TimeoutException:
                pass
        return False


class WaitForClickedCustomer(smach.State):
    """ Wait for the waiving person """

    def __init__(self, robot, caller_id):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'rejected'])
        self._robot = robot
        self._caller_id = caller_id
        self._sub = rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        self.rate = 10
        self._point = None

    def callback(self, point_stamped):
        rospy.loginfo("Recieved a point:\n{}".format(point_stamped))
        self._point = point_stamped

    def execute(self, userdata):
        self._robot.speech.speak("I'm waiting for a customer")
        rospy.loginfo("You can click in rviz")

        rate = rospy.Rate(self.rate)
        self._point = False
        while not rospy.is_shutdown() and not self._point:
            rate.sleep()

        if not self._point:
            return 'aborted'

        # TODO, get data from point into ED
        pose = frame_stamped("map", self._point.point.x, self._point.point.y, 0.0)
        self._robot.ed.update_entity(id=self._caller_id, frame_stamped=pose, type="waypoint")
        return 'succeeded'


if __name__ == '__main__':
    rospy.init_node('wait_for_customer')

    from robot_skills.amigo import Amigo
    robot = Amigo()
    robot.ed.reset()

    sm = smach.StateMachine(outcomes=['done', 'aborted'])
    with sm:
        smach.StateMachine.add('STORE_WAYPOINT',
                               WaitForCustomer(robot),
                               transitions={
                                    'succeeded': 'done',
                                    'aborted': 'done',
                                    'rejected': 'done'})

    # states.startup(setup_statemachine, challenge_name="automatic_side_detection")
    sm.execute()

