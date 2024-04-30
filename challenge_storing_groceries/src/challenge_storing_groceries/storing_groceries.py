# ROS
import smach

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_smach_states.util.geometry_helpers import *

from robot_smach_states.startup import StartChallengeRobust
from robot_smach_states.utility import CheckBool
from robocup_knowledge import load_knowledge
from robot_skills.classification_result import ClassificationResult

# Challenge storing groceries
from challenge_storing_groceries.inspect_shelves import InspectAreas
from challenge_storing_groceries.manipulate_machine import StoreItems
from challenge_storing_groceries.open_door import OpenDoorMachine

challenge_knowledge = load_knowledge('challenge_storing_groceries')


def setup_statemachine(robot):
    state_machine = smach.StateMachine(outcomes=['Done', 'Failed', 'Aborted'])

    skip_door = rospy.get_param("~skip_door", False)
    skip_inspect = rospy.get_param("~skip inspect", False)
    shelf_des = ds.EntityByIdDesignator(robot, uuid=challenge_knowledge.shelf)
    table_des = ds.EntityByIdDesignator(robot, uuid=challenge_knowledge.table)
    objects_des = ds.VariableDesignator(resolve_type=[ClassificationResult])
    room_des = ds.EntityByIdDesignator(robot, challenge_knowledge.room, name="room_designator")

    with state_machine:

        smach.StateMachine.add('START_CHALLENGE_ROBUST',
                               StartChallengeRobust(robot, challenge_knowledge.starting_point),
                               transitions={'Done': 'SKIP_DOOR',
                                            'Aborted': 'Aborted',
                                            'Failed': 'Failed'})

        smach.StateMachine.add("SKIP_DOOR",
                               CheckBool(skip_door),
                               transitions={'true': "NAV_TO_START",
                                            'false': "OPEN_DOOR"})

        # open the door of the cabinet
        smach.StateMachine.add("OPEN_DOOR",
                               OpenDoorMachine(robot, shelf_des, cabinet_inspect_area=challenge_knowledge.cabinet_inspect_area),
                               transitions={'succeeded': 'SAY_CLOSE_DOOR',
                                            'failed': 'SAY_UNABLE_TO_OPEN_DOOR'})

        smach.StateMachine.add("SAY_CLOSE_DOOR",
                               states.human_interaction.Say(robot, "I might close the door later,"
                                                                   "do not be shocked by it"),
                               transitions={'spoken': 'NAV_TO_START'})

        smach.StateMachine.add('SAY_UNABLE_TO_OPEN_DOOR',
                               states.human_interaction.Say(robot, "I am unable to open the shelf door, "
                                                                   "can you please open it for me?"),
                               transitions={'spoken': 'NAV_TO_START'})

        # Inspect shelf
        smach.StateMachine.add("NAV_TO_START",
                               states.navigation.NavigateToSymbolic(robot,
                                                                    {shelf_des: "in_front_of"},
                                                                    shelf_des, room=room_des),
                               transitions={'arrived': 'SKIP_INSPECT',
                                            'unreachable': 'SKIP_INSPECT',
                                            'goal_not_defined': 'SKIP_INSPECT'})

        smach.StateMachine.add("SKIP_INSPECT",
                               CheckBool(skip_inspect),
                               transitions={'true': 'STORE_GROCERIES',
                                            'false': 'INSPECT_SHELVES'})

        smach.StateMachine.add("INSPECT_SHELVES",
                               InspectAreas(robot, shelf_des, objects_des, room_des, knowledge=challenge_knowledge,
                                            navigation_area='in_front_of'),
                               transitions={'done': 'RESET_ARM',
                                            'failed': 'Failed'})

        @smach.cb_interface(outcomes=["done"])
        def reset_arm(userdata=None):
            """ Set the robots arm to reset pose"""
            arm = robot.get_arm(required_goals=['reset'])
            arm.send_joint_goal('reset')
            return "done"
        smach.StateMachine.add("RESET_ARM",
                               smach.CBState(reset_arm),
                               transitions={'done': 'STORE_GROCERIES'})

        # store items
        smach.StateMachine.add("STORE_GROCERIES",
                               StoreItems(robot, table_des, shelf_des, objects_des, challenge_knowledge, room=room_des),
                               transitions={'succeeded': 'AT_END',
                                            'preempted': 'Aborted',
                                            'failed': 'Failed'}
                               )

        smach.StateMachine.add('AT_END',
                               states.human_interaction.Say(robot, "Goodbye"),
                               transitions={'spoken': 'Done'})

        ds.analyse_designators(state_machine, "manipulation")

    return state_machine
