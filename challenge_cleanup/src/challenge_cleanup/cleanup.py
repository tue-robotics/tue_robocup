#!/usr/bin/python

"""
Clean UP [Housekeeper] challenge
This challenge is described in the 2019 RoboCup@Home Rulebook / Draft version

Main goal
Upon entrance, the robot requests the operator which room shall be cleaned. All misplaced known objects
found in this room must be taken to their predefined locations and unknown objects thrown in the trash bin.

Number of objects: 5 to 10
Objects can be anywhere, including the floor, seats, and on furniture. All objects are visible from
at least 1.0 m distance (no occlusions) and have the following distributions:
    Known objects: Any two regular and two alike objects
    Unknown objects: One unknown object at grasping distance (i.e. no decorations)

Reward: 1000 pts (100 pts per object)
Bonus: max 500 pts



"""
# ROS
import rospy
import smach
import hmi

# ED
from ed_robocup.srv import FitEntityInImageRequest

# Robot smach states
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_smach_states import Place
from robot_smach_states.util.geometry_helpers import *

# Robot skills
from robot_skills.util.kdl_conversions import VectorStamped
from robot_skills.util import msg_constructors as geom
from robot_skills.util import transformations

# Knowledge has not been made yet.
from robocup_knowledge import knowledge_loader

challenge_knowledge = knowledge_loader.load_knowledge('challenge_cleanup')
STARTPOINT = challenge_knowledge.startpoint
EXPLORE1 = challenge_knowledge.explore1
EXPLORE2 = challenge_knowledge.explore2
EXPLORE3 = challenge_knowledge.explore3
EXPLORE4 = challenge_knowledge.explore4
EXPLORE5 = challenge_knowledge.explore5
EXPLORE_ROUTE = [EXPLORE1, EXPLORE2, EXPLORE3, EXPLORE4, EXPLORE5]

# challenge knowledge needs to contain:
# arena layout (rooms with entry point)
# known objects with respective location
# search route for each room

class InitializeWorldModel(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.ed.reset()

        return "done"


class DummyState(smach.State):
    def __init__(self, outcomes):
        # type: (list)
        smach.State.__init__(self, outcomes=outcomes)

    def execute(self, ud=None):
        while not rospy.is_shutdown():
            print("Outcomes:")
            idx = 1
            for item in self.get_registered_outcomes():
                print("[{}]: {}".format(idx, item))
                idx += 1
            input_string = raw_input("Desired outcome:\n")
            if self.get_registered_outcomes()[int(input_string) - 1] not in self.get_registered_outcomes():
                rospy.logerr("Input '{}'' not in allowed.")
            else:
                rospy.loginfo("Input: '{}'' is correct".format(input_string))
                return self.get_registered_outcomes()[int(input_string) - 1]


# Make statemachine here

class CleanUp(smach.StateMachine):
    """ Cleanup challenge statemachine
    : ivar explore_point: current waypoint, index in EXPLORE_ROUTE
    """

    def __init__(self):
        """ Initialization method

        """
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])
        self.explore_point = 0
        # Request the operator which room has to be cleaned
        #       Need some conversation engine here ( see gpsr.py?)
        #       Say "Which room do you want me to clean"
        #       Wait for a correct answer, but not too long....
        #       Say "I am on  my way to the [location]"

        # Go to the entrance of the requested room
        #       NavigateToWaypoint(StartPoint)

        # Go to the exit of the room
        #       NavigateToWayPoint(ExitPoint)

        # Iterate until all (way)points are visited (Deus ex Machina may apply)

        # Make snapshot with camera and distinguish none/recognized/unrecognized objects
        #       Update the respective lists for the objects
        #       If known/unknown object found

        # class ManipulateSingleItem(smach.StateMachine):
        #
        #           Move to the point to make grasping possible - (NavigateToGrasp())
        #           Grasp the (un)known object - (Grab())
        #           Carry the object to its destination (trashbin or predefined place) - (NavigateToWaypoint(), NavigateToPlace())
        #           Place unrecognized object from gripper in trashbin, - (Place())
        #           or place known object in its predefined location. Do not drop or throw. (Place())
        #
        # Move to the next point - NavigateToWaypoint()

        with self:
            smach.StateMachine.add("MOVE_TO_POINT",
                                   states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=STARTPOINT),
                                                             radius=0.5),
                                   transitions={'arrived': 'TAKE_SNAPSHOT',
                                                'unreachable': 'Aborted',
                                                'goal_not_defined': 'Aborted'})
            smach.StateMachine.add("TAKE_SNAPSHOT",
                                   DummyState(["1", "Done", "Aborted"]),
                                   transitions={"1": "SEGMENT",
                                                "Done": "Done",
                                                "Aborted": "Aborted"})
            smach.StateMachine.add("SEGMENT",
                                   DummyState(["1", "2", "3", "Done", "Aborted"]),
                                   transitions={"1": "OBJECT FOUND",
                                                "2": "OBJECT FOUND", #UNKNOWN object!
                                                "3": "NO_OBJECT",
                                                "Done": "Done",
                                                "Aborted": "Aborted"})
            smach.StateMachine.add("OBJECT FOUND",
                                   DummyState(["1", "Done", "Aborted"]),
                                   transitions={"1": "GRAB_OBJECT",
                                                "Done": "Done",
                                                "Aborted": "Aborted"})
            smach.StateMachine.add("GRAB_OBJECT",
                                   DummyState(["1", "Done", "Aborted"]),
                                   transitions={"1": "MOVE_TO_DEST",
                                                "Done": "Done",
                                                "Aborted": "Aborted"})
            smach.StateMachine.add("MOVE_TO_DEST",
                                   DummyState(["1", "Done", "Aborted"]),
                                   transitions={"1": "PLACE_OBJECT",
                                                "Done": "Done",
                                                "Aborted": "Aborted"})
            smach.StateMachine.add("PLACE_OBJECT",
                                   DummyState(["1", "Done", "Aborted"]),
                                   transitions={"1": "GET_NEXT_POS",
                                                "Done": "Done",
                                                "Aborted": "Aborted"})
            smach.StateMachine.add("NO_OBJECT",
                                   DummyState(["1", "Done", "Aborted"]),
                                   transitions={"1": "GET_NEXT_POS",
                                                "Done": "Done",
                                                "Aborted": "Aborted"})
            smach.StateMachine.add("GET_NEXT_POS",
                                   DummyState(["1", "Done", "Aborted"]),
                                   transitions={"1": "MOVE_TO_POINT",
                                                "Done": "Done",
                                                "Aborted": "Aborted"})


def setup_statemachine(robot):
    sm1 = smach.StateMachine(outcomes=['Done', 'Aborted'])

    with sm1:
        smach.StateMachine.add("INITIALIZE",
                               states.Initialize(robot),
                               transitions={'initialized': 'INIT_WM',
                                            'abort': 'Aborted'})
        smach.StateMachine.add("INIT_WM",
                               InitializeWorldModel(robot),
                               transitions={'done': 'WAIT_FOR_START'})
        smach.StateMachine.add("WAIT_FOR_START",
                               DummyState(["1", "Done", "Aborted"]),
                               transitions={"1": "ASK_FOR_DESTINATION",
                                            "Done": "Done",
                                            "Aborted": "Aborted"})
        smach.StateMachine.add("ASK_FOR_DESTINATION",
                               DummyState(["1", "Done", "Aborted"]),
                               transitions={"1": "NAV_TO_ROOM",
                                            "Done": "Done",
                                            "Aborted": "Aborted"})
        smach.StateMachine.add("NAV_TO_ROOM",
                               states.NavigateToWaypoint(robot, EntityByIdDesignator(robot, id=STARTPOINT),
                                                         radius=0.5),
                               transitions={'arrived': 'SEARCH_AND_DETECT',
                                            'unreachable': 'Aborted',
                                            'goal_not_defined': 'Aborted'})
        smach.StateMachine.add("SEARCH_AND_DETECT", CleanUp(),
                               transitions={"Done": "Done",
                                            "Aborted": "Aborted"})
    return sm1

    #     smach.StateMachine.add('INITIALIZE',
    #                             states.Initialize(robot),
    #                             transitions={'initialized': 'INIT_WM',
    #                                          'abort'      : 'Aborted'})
    #
    #     smach.StateMachine.add('INIT_WM',
    #                             InitializeWorldModel(robot),
    #                             transitions={'done'       : 'AWAIT_START'})
    #
    #     smach.StateMachine.add('AWAIT_START',
    #                             states.AskContinue(robot),
    #                             transitions={'continue'   : 'NAV_TO_ROOM',
    #                                          'no_response': 'AWAIT_START'})
    #
    #     # Ask operator which room has to be cleaned. If answer is OK: NAV_TO_ROOM
    #
    #     smach.StateMachine.add('NAV_TO_ROOM',
    #                             states.NavigateToWayPoint(DesignatedRoom))
    #                             transitions={'arrived'      : 'SEARCH_AND_DETECT'
    #                                          'unreachable'  : ''})
    #
    #     smach.StateMachine.add('SEARCH_AND_DETECT',
    #                             # Iterate over room waypointlist in another statemachine
    #
    #                             transitions={'alldone'       : 'FINISH_UP'
    #                                          'failed'        : 'FAILURE'} )
    #
    #     smach.StateMachine.add('FINISH_UP',
    #
    #                             transitions=)
    #
    #     smach.StateMachine.add('FAILURE',
    #
    #                             transitions=)


if __name__ == '__main__':
    rospy.init_node("clean_up")
    startup(setup_statemachine, challenge_name="clean-up")
