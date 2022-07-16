import math
import os

import numpy as np
import rospy
from pykdl_ros import VectorStamped
from smach.state import CBState
from smach.state_machine import StateMachine
from smach.util import cb_interface
from visualization_msgs.msg import MarkerArray

from robot_skills import get_robot
from robot_smach_states.navigation import NavigateToSymbolic
from robot_smach_states.navigation import NavigateToWaypoint, ForceDrive
from robot_smach_states.util.designators import EntityByIdDesignator
from robot_smach_states.world_model import Inspect


class NavigateArbitrarily(StateMachine):
    def __init__(self, robot, look_range=(np.pi * 0.28, -np.pi * 0.28), look_steps=7):
        StateMachine.__init__(self, outcomes=["done", "preempted"])
        self._robot = robot
        self._look_distance = 3.0
        self._look_angles = np.linspace(look_range[0], look_range[1], look_steps)
        self._visualization_marker_pub = rospy.Publisher('/markers', MarkerArray, queue_size=1)

        self.waypoint = EntityByIdDesignator(robot, uuid="hand_me_that_kitchen")
        self.victim = EntityByIdDesignator(robot, uuid="victim")
        self._cupboard_des = EntityByIdDesignator(robot, uuid="cupboard")
        self._bedroom_des = EntityByIdDesignator(robot, uuid="bedroom")
        self._kitchen_des = EntityByIdDesignator(robot, uuid="kitchen")
        self._dinner_table_des = EntityByIdDesignator(robot, uuid="dinner_table")

        with self:

            @cb_interface(outcomes=["done"])
            def _look_around(_):
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
                        break

                    self._robot.head.wait_for_motion_done()

                return "done"

            self.add("LOOK_AROUND_OFFICE", CBState(_look_around), transitions={"done": "NAVIGATE_BEDROOM"})
            self.add("NAVIGATE_BEDROOM", NavigateToSymbolic(robot,
                                                            {self._cupboard_des: "in_front_of"},
                                                            entity_lookat_designator=self._bedroom_des),
                     transitions={"arrived": "LOOK_AROUND_BEDROOM",
                                  "unreachable": "LOOK_AROUND_BEDROOM",
                                  "goal_not_defined": "preempted"})
            self.add("LOOK_AROUND_BEDROOM", CBState(_look_around), transitions={"done": "NAVIGATE_WAYPOINT1"})

            self.add('NAVIGATE_WAYPOINT1', NavigateToWaypoint(robot, self.waypoint, 0.3, self._kitchen_des),
                     transitions={'arrived': 'INSPECT_DINNER_TABLE',
                                  'unreachable': 'NAVIGATE_WAYPOINT_FAILED1',
                                  'goal_not_defined': 'preempted'})

            self.add('NAVIGATE_WAYPOINT_FAILED1', ForceDrive(robot, 0, 0, 0.5, (2 * math.pi) / 0.5),
                     transitions={'done': 'NAVIGATE_WAYPOINT1'})

            self.add("INSPECT_DINNER_TABLE", Inspect(robot, self._dinner_table_des, navigation_area="in_front_of"),
                     transitions={"done": "NAVIGATE_WAYPOINT2",
                                  "failed": "NAVIGATE_WAYPOINT2"})

            self.add('NAVIGATE_WAYPOINT2', NavigateToWaypoint(robot, self.waypoint, 0.3, self._kitchen_des),
                     transitions={'arrived': 'done',
                                  'unreachable': 'NAVIGATE_WAYPOINT_FAILED2',
                                  'goal_not_defined': 'preempted'})

            self.add('NAVIGATE_WAYPOINT_FAILED2', ForceDrive(robot, 0, 0, 0.5, (2 * math.pi) / 0.5),
                     transitions={'done': 'done'})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    NavigateArbitrarily(robot_instance).execute()
