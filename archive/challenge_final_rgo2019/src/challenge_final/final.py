import robot_smach_states as states
import rospy
import smach
from challenge_storing_groceries.manipulate_machine import PlaceSingleItem
from robocup_knowledge import knowledge_loader
from robot_smach_states.util.designators.ed_designators import EdEntityDesignator

from pickup_item import PickupItem
from pointing_designator import PointingDesignator
from pointing_detector import PointingDetector

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
        furniture_move_designator = PointingDesignator(robot=robot)
        furniture_pick_designator = PointingDesignator(robot=robot)
        furniture_place_designator = PointingDesignator(robot=robot)

        with self:
            # Start challenge
            smach.StateMachine.add("START_CHALLENGE",
                                   states.StartChallengeRobust(robot=robot, initial_pose=knowledge.initial_pose),
                                   transitions={"Done": "NAVIGATE_TO_MEETING_POINT0",
                                                "Aborted": "Aborted",
                                                "Failed": "Aborted"})

            # Move to meeting point
            # ToDo: add challenge knowledge
            smach.StateMachine.add("NAVIGATE_TO_MEETING_POINT0",
                                   states.NavigateToWaypoint(robot=robot,
                                                             waypoint_designator=EdEntityDesignator(
                                                                 robot=robot, id=knowledge.meeting_point), radius=0.15),
                                   transitions={"arrived": "DETECT_POINTING0",
                                                "unreachable": "DETECT_POINTING0",
                                                "goal_not_defined": "DETECT_POINTING0"})

            # Wait for the operator to appear and detect what he's pointing at
            smach.StateMachine.add("DETECT_POINTING0",
                                   PointingDetector(robot=robot, designator=furniture_move_designator,
                                                    default_entity_id="couch_table",
                                                    super_type="furniture"),
                                   transitions={"succeeded": "MOVE_TO_ITEM",
                                                "failed": "DETECT_POINTING0"})

            # Move to jury table
            smach.StateMachine.add("MOVE_TO_ITEM",
                                   states.NavigateToSymbolic(robot, {furniture_move_designator: "near"},
                                                             furniture_move_designator),
                                   transitions={"arrived": "SAY_HI_TO_JURY",
                                                "unreachable": "SAY_HI_TO_JURY",
                                                "goal_not_defined": "SAY_HI_TO_JURY"})

            smach.StateMachine.add("SAY_HI_TO_JURY",
                                   states.Say(robot, "Hi guys, I'm glad you're here", block=True),
                                   transitions={"spoken": "NAVIGATE_TO_MEETING_POINT1"})

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
                                                    default_entity_id="desk",
                                                    super_type="furniture"),
                                   transitions={"succeeded": "PICKUP_ITEM",
                                                "failed": "DETECT_POINTING1"})

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
                                                    default_entity_id="side_table",
                                                    super_type="furniture"),
                                   transitions={"succeeded": "PLACE_ITEM",
                                                "failed": "DETECT_POINTING2"})

            # Place the object
            smach.StateMachine.add("PLACE_ITEM",
                                   PlaceSingleItem(robot=robot, place_designator=furniture_place_designator),
                                   transitions={"succeeded": "SAY_DONE",
                                                "failed": "SAY_DONE"})

            # Say that we're done
            smach.StateMachine.add("SAY_DONE",
                                   states.Say(robot, "That's it for today, you can have a closer look at my"
                                                     "skills on the screen", block=False),
                                   transitions={"spoken": "RESET_HEAD"})

            @smach.cb_interface(outcomes=["done"])
            def reset_head(userdata=None):
                """ Resets the head """
                print "Resetting head"
                robot.head.reset()
                rospy.sleep(rospy.Duration(1.0))
                return "done"

            smach.StateMachine.add("RESET_HEAD",
                                   smach.CBState(reset_head),
                                   transitions={"done": "Done"})

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
