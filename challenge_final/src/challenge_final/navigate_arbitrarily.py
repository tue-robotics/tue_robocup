import math
import os

import rospy
from pykdl_ros import VectorStamped
from smach.state import CBState
from smach.state_machine import StateMachine
from smach.util import cb_interface

from robot_skills import get_robot


class NavigateArbitrarily(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])
        self._robot = robot

        with self:
            @cb_interface(outcomes=["done"])
            def _look_for_person(_):
                start_time = rospy.Time.now()
                head_goals = [VectorStamped.from_xyz(x=self._look_distance * math.cos(angle),
                                                     y=self._look_distance * math.sin(angle),
                                                     z=1.5,
                                                     stamp=start_time,
                                                     frame_id=self._robot.base_link_frame)
                              for angle in self._look_angles]

                i = 0
                while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < self._search_timeout:
                    if self.preempt_requested():
                        return 'failed'

                    self._robot.head.look_at_point(head_goals[i])
                    i += 1

                    if i == len(head_goals):
                        i = 0

                    self._robot.head.wait_for_motion_done()

                    found_people_ids = []
                    for _ in range(2):  # TODO: parametrize
                        self._image_data = self._robot.perception.get_rgb_depth_caminfo()
                        if self._image_data:
                            success, found_ids = self._robot.ed.detect_people(*self._image_data)
                        else:
                            rospy.logwarn("Could not get_rgb_depth_caminfo")
                            success, found_ids = False, []
                        found_people_ids += found_ids

                    # Use only unique IDs in the odd case ED sees the same people twice
                    found_people = [self._robot.ed.get_entity(eid) for eid in set(found_people_ids)]

                    rospy.loginfo("Found {} people: {}".format(len(found_people), found_people))
                    found_people = [p for p in found_people if p]
                    rospy.loginfo("{} people remaining after None-check".format(len(found_people)))

                    robot_pose = self._robot.base.get_location()
                    found_people = [p for p in found_people if
                                    (p.pose.frame.p - robot_pose.frame.p).Norm() <= self._look_distance]

                    rospy.loginfo(
                        "{} people remaining after distance < {}-check".format(len(found_people), self._look_distance))

                    if self._properties:
                        for k, v in self._properties.items():
                            found_people = [x for x in found_people if self._check_person_property(x, k, v)]
                            rospy.loginfo("{} people remaining after {}={} check".format(len(found_people), k, v))

                    result_people = found_people

                    if result_people:
                        result_people.sort(key=lambda e: (e.pose.frame.p - robot_pose.frame.p).Norm())

                        self._robot.head.close()

                        person = result_people[0]
                        self._robot.ed.update_entity(uuid="victim", frame_stamped=person.pose)

                        return 'found'
                    else:
                        rospy.logwarn("Could not find people meeting the requirements")

            self.add("PICK", CBState(_look_for_person), transitions={"done": "done"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    NavigateArbitrarily(robot_instance).execute()
