# ROS
import PyKDL as kdl
import robot_smach_states as states
import rospy
import smach

# TU/e Robotics
from robocup_knowledge import knowledge_loader
from robocup_knowledge import knowledge_loader
import robot_smach_states as states
from robot_smach_states.util.designators.ed_designators import EdEntityDesignator
from challenge_storing_groceries.manipulate_machine import PlaceSingleItem

# Challenge final
from pickup_item import PickupItem
from pointing_designator import PointingDesignator
from pointing_detector import PointingDetector
from track_operator import TrackFace

# Load the knowledge
knowledge = knowledge_loader.load_knowledge("challenge_final")





class ChallengeFinal(smach.StateMachine):
    """ State machine for the final challenge """
    def __init__(self, robot):
        """ Constructor
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted"])

        # Designators
        furniture_pick_designator = PointingDesignator(robot=robot)
        furniture_place_designator = PointingDesignator(robot=robot)

        with self:
            # Start challenge
            smach.StateMachine.add("START_CHALLENGE",
                                   states.StartChallengeRobust(robot=robot, initial_pose=knowledge.initial_pose),
                                   transitions={"Done": "NAVIGATE_TO_MEETING_POINT1",
                                                "Aborted": "Aborted",
                                                "Failed": "Aborted"})

            # Move to meeting point
            # ToDo: add challenge knowledge
            smach.StateMachine.add("NAVIGATE_TO_MEETING_POINT1",
                                   states.NavigateToWaypoint(robot=robot,
                                                             waypoint_designator=EdEntityDesignator(
                                                                 robot=robot, id=knowledge.meeting_point), radius=0.15),
                                   transitions={"arrived": "DETECT_POINTING1",
                                                "unreachable": "DETECT_POINTING1",
                                                "goal_not_defined": "DETECT_POINTING1"})

            # Wait for the operator to appear and detect what he's pointing at
            smach.StateMachine.add("DETECT_POINTING1",
                                   PointingDetector(robot=robot, designator=furniture_pick_designator,
                                                    super_type="furniture"),
                                   transitions={"succeeded": "PICKUP_ITEM",
                                                "failed": "PICKUP_ITEM"})

            # Inspect and pickup
            smach.StateMachine.add("PICKUP_ITEM",
                                   PickupItem(robot=robot, furniture_designator=furniture_pick_designator),
                                   transitions={"succeeded": "NAVIGATE_TO_MEETING_POINT2",
                                                "failed": "NAVIGATE_TO_MEETING_POINT2"})

            # Move back to meeting point
            # ToDo: add challenge knowledge
            smach.StateMachine.add("NAVIGATE_TO_MEETING_POINT2",
                                   states.NavigateToWaypoint(robot=robot,
                                                             waypoint_designator=EdEntityDesignator(
                                                                 robot=robot, id=knowledge.meeting_point), radius=0.15),
                                   transitions={"arrived": "DETECT_POINTING2",
                                                "unreachable": "DETECT_POINTING2",
                                                "goal_not_defined": "DETECT_POINTING2"})

            # Wait for the operator to appear and detect what he's pointing at
            smach.StateMachine.add("DETECT_POINTING2",
                                   PointingDetector(robot=robot, designator=furniture_place_designator,
                                                    super_type="furniture"),
                                   transitions={"succeeded": "Done",
                                                "failed": "Done"})

            # Place the object
            smach.StateMachine.add("PLACE_ITEM",
                                   PlaceSingleItem(robot=robot, place_designator=furniture_place_designator),
                                   transitions={"succeeded": "Done",
                                                "failed": "Done"})


            # # Learn operator
            # smach.StateMachine.add("LEARN_OPERATOR",
            #                        states.LearnPerson(robot, person_name="operator", nr_tries=5),
            #                        transitions={"succeeded": "TRACK_OPERATOR",
            #                                     "failed": "Done"})
            #
            # # TrackOperator state
            # smach.StateMachine.add("TRACK_OPERATOR",
            #                        TrackFace(robot=robot),
            #                        transitions={"aborted": "Done",
            #                                     "lost": "Done"})


