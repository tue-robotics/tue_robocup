#! /usr/bin/env python

# System
import math
import sys

# ROS
import PyKDL as kdl
import geometry_msgs
import rospy
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.util import kdl_conversions


class FindPerson(smach.State):
    """
    Smach state to find a person. The robot looks around and tries to recognize all faces in view.
    """

    def __init__(self, robot, properties=None, query_entity_designator=None,
                 result_designator=None, look_distance=1.0, speak=False,
                 strict=True, nearest=False, attempts=1, search_timeout=60):
        """ Initialization method
        :param robot: robot api object
        :param properties: (dict) keyvalue pair of the properties a person must
            possess. None as a value for a property would search for all possible
            values of the property.
        :param query_entity_designator: An entity designator to match all found
            people to
        :param result_designator: A designator to write the search result to.
            The designator always has a list of found people written to it.
        :param look_distance: (float) The distance (radius) which the robot must look at
        :param speak: (bool) If True, the robot will speak while trying to find
            a named person
        :param strict: (bool) If True then only people with all specified
            properties is returned else all people with at least one true property
        :param nearest: (bool) If True, selects the person nearest to the robot
        :param attempts: (int) Max number of search attempts
        :param search_timeout: (float) maximum time the robot is allowed to search
        """
        smach.State.__init__(self, outcomes=['found', 'failed'])

        self._robot = robot

        self._properties = properties
        self._query_entity_designator = query_entity_designator
        self._look_distance = look_distance
        self._speak = speak
        self._strict = strict
        self._nearest = nearest

        self._search_timeout = search_timeout

        if result_designator:
            ds.is_writeable(result_designator)
        self._result_designator = result_designator

    def execute(self, userdata=None):
        look_angles = None
        if not self._properties:
            look_angles = [0]
        else:
            look_angles = [f * math.pi / d if d != 0 else 0.0 for f in [-1, 1] for d in [0, 6, 4, 2.3]]  # Magic numbers
            try:
                person_label = self._properties["id"]
                person_label = person_label.resolve() if hasattr(person_label, 'resolve') else person_label

                rospy.loginfo("Trying to find {}".format(person_label))
                if self._speak:
                    self._robot.speech.speak(
                            "{}, please look at me while I am looking for you".format(
                            person_label),
                        block=False)
            except:
                pass

        start_time = rospy.Time.now()

        head_goals = [kdl_conversions.VectorStamped(x=self._look_distance * math.cos(angle),
                                                    y=self._look_distance * math.sin(angle),
                                                    z=1.5,
                                                    frame_id="/%s/base_link" % self._robot.robot_name)
                      for angle in look_angles]

        i = 0
        attempts = 0

        rate = rospy.Rate(2)
        while not rospy.is_shutdown() and
            attempts < self._attempts and
            (rospy.Time.now() - start_time).to_sec() < self._search_timeout:

            if self.preempt_requested():
                return 'failed'

            self._robot.head.look_at_point(head_goals[i])
            i += 1

            if i == len(head_goals):
                i = 0
                attempts += 1

            self._robot.head.wait_for_motion_done()

            self._image_data = self._robot.perception.get_rgb_depth_caminfo()
            success, found_people_ids = self._robot.ed.detect_people(*self._image_data)
            found_people = [self._robot.ed.get_entity(eid) for eid in found_people_ids]

            result_people = None


            if self._query_entity_designator:
                # TODO: Check if query_entity_designator is actually a
                # designator
                query_entity = self._query_entity_designator.resolve()
                result_people = filter(lambda x:
                        query_entity.in_volume(x.pose.extractVectorStamped(),
                        'in'), found_people)

                # If people not in query_entity then try if query_entity in
                # people
                if not result_people:
                    try:
                        result_people = filter(lambda x:
                            x.in_volume(query_entity.pose.extractVectorStamped(),
                                'in'), found_people)
                    except:
                        pass


            if result_people:
                #self._robot.speech.speak("I think I found {}.".format(person_label, block=False))
                self._robot.head.close()

                if self._result_designator:
                    self._result_designator.write(result_people)

                return 'found'
            else:
                rospy.logwarn("Could not find people meeting the requirements")
                rate.sleep()

        self._robot.head.close()
        rospy.sleep(2.0)
        return 'failed'
