import math
import os

import rospy
from smach.state_machine import StateMachine

import robot_smach_states.util.designators as ds
from challenge_where_is_this.inform_machine import GuideToRoomOrObject
from robot_skills import get_robot
from robot_smach_states.human_interaction import Say
from robot_smach_states.human_interaction.human_interaction import WaitForPersonInFront
from robot_smach_states.navigation import ForceDrive
from robot_smach_states.navigation.navigate_to_waypoint import NavigateToWaypoint


class NavigateToTheDoorAndGuideNeighborToVictim(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])

        waypoint_door = {'id': 'entry_door', 'radius': 0.2}
        door_waypoint = ds.EntityByIdDesignator(robot, uuid=waypoint_door['id'])
        victim_entity = ds.EntityByIdDesignator(robot, uuid='victim')

        with self:
            StateMachine.add('NAVIGATE_DOOR', NavigateToWaypoint(robot, door_waypoint, waypoint_door['radius']),
                             transitions={'arrived': 'SAY_PLEASE_COME_IN',
                                          'unreachable': 'SAY_NAVIGATE_TO_DOOR_FALLBACK',
                                          'goal_not_defined': 'preempted'})

            StateMachine.add('SAY_NAVIGATE_TO_DOOR_FALLBACK',
                             Say(robot, "Help, lets try it another way, hope this goes "
                                        "faster because we are in a hurry",
                                 block=False),
                             transitions={'spoken': 'TURN_AROUND'})

            StateMachine.add('TURN_AROUND', ForceDrive(robot, 0, 0, 0.5, (2 * math.pi) / 0.5),
                             transitions={'done': 'NAVIGATE_DOOR'})

            StateMachine.add('SAY_PLEASE_COME_IN',
                             Say(robot, ["Please come in soon, I'm waiting for you"],
                                 block=True, look_at_standing_person=True),
                             transitions={'spoken': 'WAIT_FOR_GUEST'})

            StateMachine.add("WAIT_FOR_GUEST", WaitForPersonInFront(robot, attempts=30, sleep_interval=1),
                             transitions={'success': 'SAY_FOLLOW_ME',
                                          'failed': 'SAY_PLEASE_COME_IN'})

            StateMachine.add('SAY_FOLLOW_ME',
                             Say(robot, ["Please follow me fast to Arpit"],
                                 block=False, look_at_standing_person=True),
                             transitions={'spoken': 'GUIDE_OPERATOR'})

            StateMachine.add(
                "GUIDE_OPERATOR",
                GuideToRoomOrObject(robot, victim_entity),
                transitions={
                    "arrived": "done",
                    "unreachable": "preempted",
                    "goal_not_defined": "preempted",
                    "lost_operator": "preempted",
                    "preempted": "preempted",
                },
            )


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    NavigateToTheDoorAndGuideNeighborToVictim(robot_instance).execute()
