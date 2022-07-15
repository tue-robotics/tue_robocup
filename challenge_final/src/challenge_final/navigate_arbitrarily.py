import math
import os

import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from pykdl_ros import VectorStamped
from smach.state import CBState
from smach.state_machine import StateMachine
from smach.util import cb_interface
from std_msgs.msg import Header, ColorRGBA
from tf_conversions import toMsg
from visualization_msgs.msg import MarkerArray, Marker

from robot_skills import get_robot

from robot_smach_states.designator_iterator import IterateDesignator
from robot_smach_states.navigation.navigation import ForceDrive
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToRoom
from robot_smach_states.util.designators import EntityByIdDesignator, VariableDesignator


ROOMS = ['kitchen', 'bedroom', 'office', 'living_room']


class NavigateArbitrarily(StateMachine):
    def __init__(self, robot, look_range=(np.pi * 0.28, -np.pi * 0.28), look_steps=7):
        StateMachine.__init__(self, outcomes=["done", "preempted"])
        self._robot = robot
        self._look_distance = 3.0
        self._look_angles = np.linspace(look_range[0], look_range[1], look_steps)
        self._visualization_marker_pub = rospy.Publisher('/markers', MarkerArray, queue_size=1)

        # navigation
        self.room_designator = VariableDesignator(ROOMS[0])
        room_designators = [EntityByIdDesignator(robot, room) for room in ROOMS]
        self.room_collection_designator = VariableDesignator(room_designators)

        with self:
            self.add("NAVIGATE_TO_ROOM", NavigateToRoom(robot, self.room_designator),
                     transitions={"arrived": "LOOK_FOR_VICTIM", "unreachable": "RECOVER", "goal_not_defined": "RECOVER"}
                     )

            @cb_interface(outcomes=["done"])
            def _look_for_victim(_):
                start_time = rospy.Time.now()
                head_goals = [VectorStamped.from_xyz(x=self._look_distance * math.cos(angle),
                                                     y=self._look_distance * math.sin(angle),
                                                     z=1.0,
                                                     stamp=start_time,
                                                     frame_id=self._robot.base_link_frame)
                              for angle in self._look_angles]

                i = 0
                while not rospy.is_shutdown():
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

                    result_people = [p for p in found_people if
                                     "RWave" in p.person_properties.tags or "LWave" in p.person_properties.tags]

                    if result_people:
                        result_people.sort(key=lambda e: (e.pose.frame.p - robot_pose.frame.p).Norm())

                        self._robot.head.close()

                        person = result_people[0]
                        self._robot.ed.update_entity(uuid="victim", frame_stamped=person.pose)

                        self._visualization_marker_pub.publish(MarkerArray(
                            markers=[
                                Marker(
                                    header=Header(
                                        frame_id="map",
                                        stamp=rospy.Time.now(),
                                    ),
                                    ns="victim",
                                    type=Marker.SPHERE,
                                    pose=toMsg(person.pose.frame),
                                    scale=Vector3(1.5, 1.5, 1.5),
                                    color=ColorRGBA(a=0.5, r=1, g=0, b=0),
                                )
                            ]
                        ))

                        return 'done'
                    else:
                        rospy.logwarn("Could not find people meeting the requirements")

            # spin in a circle when navigation is stuck
            self.add("RECOVER", ForceDrive(robot, 0, 0, 1.0, math.pi*4), transitions={"done": "NAVIGATE_TO_ROOM"})

            self.add("LOOK_FOR_VICTIM", CBState(_look_for_victim), transitions={"done": "done"})

            self.add("ITERATE_NEXT_ROOM", IterateDesignator(self.room_collection_designator,
                                                            self.room_designator.writeable(),
                                                            transitions={"next":"NAVIGATE_TO_ROOM", "stop_iteration": })
                     )

            @cb_interface(outcomes=["done"])
            def _reset_iterator(_):
                self.room_collection_designator = VariableDesignator([EntityByIdDesignator(robot, room) for room in ROOMS])


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    NavigateArbitrarily(robot_instance).execute()
