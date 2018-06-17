#!/usr/bin/python

import math
from Queue import Queue, Empty

import PyKDL as kdl
import rospy
import smach
from geometry_msgs.msg import PointStamped
from hmi import TimeoutException
from robot_skills.util.kdl_conversions import frame_stamped, VectorStamped
from tue_msgs.msg import People


class WaitForCustomer(smach.State):
    """ Wait for the waiving person """

    def __init__(self, robot, caller_id, kitchen_designator):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self._robot = robot
        self._caller_id = caller_id
        self._kitchen_designator = kitchen_designator
        self._people_sub = rospy.Subscriber(robot.robot_name + '/persons', People, self.people_cb, queue_size=1)
        self.people_queue = Queue(maxsize=1)

    def people_cb(self, persons):
        if persons.people:
            rospy.logdebug('Received %d persons in the people cb', len(persons.people))
        self.people_queue.put(persons)

    def execute(self, userdata=None):
        """ Does the actual work

        :param userdata:
        :return:
        """

        self._robot.head.reset()
        self._robot.head.wait_for_motion_done()

        self._robot.speech.speak("I'm waiting for a waving person")

        head_samples = 20
        look_distance = 3.0
        look_angles = [0,
                       0,
                       0,
                       10,
                       -10,
                       20,
                       -20]

        self.clear_queue()

        waving_persons = []
        i = 0
        while not rospy.is_shutdown() and not waving_persons:
            waving_persons = self.wait_for_waving_person(head_samples=head_samples)

            angle = look_angles[i % len(look_angles)]
            rospy.loginfo('Still waiting... looking at %d degrees', angle)
            angle = math.radians(angle)
            head_goal = VectorStamped(x=look_distance * math.cos(angle),
                                      y=look_distance * math.sin(angle), z=1.3,
                                      frame_id="/%s/base_link" % self._robot.robot_name)
            self._robot.head.look_at_point(head_goal)
            i += 1

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
        self._robot.head.wait_for_motion_done()

        return 'succeeded'

    def clear_queue(self):
        while True:
            try:
                self.people_queue.get_nowait()
            except Empty:
                # There is probably an old measurement blocking in the callback thread, also remove that one
                self.people_queue.get()
                return

    def wait_for_waving_person(self, head_samples):
        waving_persons = []
        for i in range(0, head_samples):
            if rospy.is_shutdown():
                return

            people_received = self.wait_for_cb()
            rospy.logdebug('Got sample %d with seq %s', i, people_received.header.seq)
            for person in people_received.people:
                if {'RWave', 'LWave'}.intersection(set(person.tags)):
                    waving_persons.append(person)

            if waving_persons:
                break
        return waving_persons

    def wait_for_cb(self):
        timeout = 1

        people_received = People()
        while not rospy.is_shutdown() and not people_received.people:
            try:
                return self.people_queue.get(timeout=timeout)
            except Empty:
                rospy.logwarn('No people message received within %d seconds', timeout)


class WaitForClickedCustomer(smach.State):
    """ Wait for the waiving person """

    def __init__(self, robot, caller_id, kitchen_designator):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
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


class AskTakeTheOrder(smach.State):
    """ Wait for the waiving person """

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['yes', 'wait', 'timeout'])

        self.robot = robot

    def execute(self, userdata):
        cgrammar = """
        C['yes'] -> amigo take the order
        C['wait'] -> amigo wait
        """
        for i in range(3):
            try:
                speech_result = self.robot.hmi.query(description="Should I get the order?",
                                                     grammar=cgrammar, target="C")
                return speech_result.semantics
            except TimeoutException:
                pass
        return 'timeout'


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

