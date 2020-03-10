# ROS
import rospy
import smach

# TU/e
import robot_skills
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_smach_states.manipulation.place_designator import EmptySpotDesignator

# Challenge set the table
# from entity_description_designator import EntityDescriptionDesignator
from config import MIN_GRAB_OBJECT_HEIGHT, MAX_GRAB_OBJECT_WIDTH
from challenge_set_a_table_states.manipulate_machine import DefaultGrabDesignator, GrabSingleItem, PlaceSingleItem


class ClearManipulateMachine(smach.StateMachine):
    """ The ManipulateMachine state machine performs the manipulation part of the set the table challenge:
    - Inspect the grasp surface
    - State item
    - Grab item
    - State item
    - Grab item
    - Drive to place surface
    - State place shelf
    - Place item
    - State place shelf
    - Place item
    """
    def __init__(self, robot, grasp_furniture_id, place_furniture_id1, place_furniture_id2):
        """
        Constructor

        :param robot: robot object
        :param grasp_furniture_id: string identifying the furniture object where to grasp the objects
        :param place_furniture_id1: string identifying the furniture object where to place objects 1 and 2
        :param place_furniture_id2: string identifying the furniture object where to place object 3
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Create designators
        grasp_furniture_designator = ds.EntityByIdDesignator(robot, id=grasp_furniture_id)
        grasp_designator1 = DefaultGrabDesignator(robot=robot, surface_designator=grasp_furniture_designator,
                                                  area_description="on_top_of")
        grasp_designator2 = DefaultGrabDesignator(robot=robot, surface_designator=grasp_furniture_designator,
                                                  area_description="on_top_of")
        grasp_designator3 = DefaultGrabDesignator(robot=robot, surface_designator=grasp_furniture_designator,
                                                  area_description="on_top_of")

        # TODO use the same designator that is used for setting the table
        arm_designator = ds.UnoccupiedArmDesignator(robot, {})

        place_furniture_designator1 = ds.EntityByIdDesignator(robot, id=place_furniture_id1)
        place_designator1 = EmptySpotDesignator(robot=robot,
                                                place_location_designator=place_furniture_designator1,
                                                arm_designator=arm_designator,
                                                area="on_top_of")

        place_furniture_designator3 = ds.EntityByIdDesignator(robot, id=place_furniture_id2)
        place_designator3 = EmptySpotDesignator(robot=robot,
                                                place_location_designator=place_furniture_designator3,
                                                arm_designator=arm_designator,
                                                area="on_top_of")

        with self:

            # Move to the inspect location
            smach.StateMachine.add("MOVE_TO_GRASP_SURFACE1",
                                   states.NavigateToSymbolic(robot,
                                                             {grasp_furniture_designator: "in_front_of"},
                                                             grasp_furniture_designator),
                                   transitions={'arrived': 'INSPECT_GRASP_SURFACE',
                                                'unreachable': 'MOVE_TO_GRASP_SURFACE2',
                                                'goal_not_defined': 'INSPECT_GRASP_SURFACE'})

            # Backup for moving to inspect location
            smach.StateMachine.add("MOVE_TO_GRASP_SURFACE2",
                                   states.NavigateToSymbolic(robot,
                                                             {grasp_furniture_designator: "large_in_front_of"},
                                                             grasp_furniture_designator),
                                   transitions={'arrived': 'INSPECT_GRASP_SURFACE',
                                                'unreachable': 'INSPECT_GRASP_SURFACE',
                                                'goal_not_defined': 'INSPECT_GRASP_SURFACE'})

            # Inspect grasp furniture
            smach.StateMachine.add("INSPECT_GRASP_SURFACE", states.Inspect(robot=robot,
                                                                           entityDes=grasp_furniture_designator,
                                                                           objectIDsDes=None,
                                                                           searchArea="on_top_of",
                                                                           navigation_area="in_front_of"),
                                   transitions={"done": "GRAB_ITEM_1",
                                                "failed": "failed"})

            # Grasp the first item
            smach.StateMachine.add("GRAB_ITEM_1", GrabSingleItem(robot=robot, grab_designator=grasp_designator1),
                                   transitions={"succeeded": "GRAB_ITEM_2",
                                                "failed": "GRAB_ITEM_2"})

            # Grasp the second item
            smach.StateMachine.add("GRAB_ITEM_2", GrabSingleItem(robot=robot, grab_designator=grasp_designator2),
                                   transitions={"succeeded": "MOVE_TO_PLACE",
                                                "failed": "MOVE_TO_PLACE"})

            # Move to the place location
            smach.StateMachine.add("MOVE_TO_PLACE",
                                   states.NavigateToSymbolic(robot,
                                                             {place_furniture_designator1: "in_front_of"},
                                                             place_furniture_designator1),
                                   transitions={'arrived': 'PLACE_ITEM_1',
                                                'unreachable': 'PLACE_ITEM_1',
                                                'goal_not_defined': 'PLACE_ITEM_1'})

            # Place the first item
            smach.StateMachine.add("PLACE_ITEM_1", PlaceSingleItem(robot=robot, place_designator=place_designator1),
                                   transitions={"succeeded": "PLACE_ITEM_2",
                                                "failed": "PLACE_ITEM_2"})

            # Place the second item
            smach.StateMachine.add("PLACE_ITEM_2", PlaceSingleItem(robot=robot, place_designator=place_designator1),
                                   transitions={"succeeded": "MOVE_TO_GRASP_SURFACE3",
                                                "failed": "MOVE_TO_GRASP_SURFACE3"})

            # Move back to the grasp surface to grasp the third item
            smach.StateMachine.add("MOVE_TO_GRASP_SURFACE3",
                                   states.NavigateToSymbolic(robot,
                                                             {grasp_furniture_designator: "in_front_of"},
                                                             grasp_furniture_designator),
                                   transitions={'arrived': 'GRAB_ITEM_3',
                                                'unreachable': 'MOVE_TO_GRASP_SURFACE4',
                                                'goal_not_defined': 'GRAB_ITEM_3'})

            # Backup for moving back to the grasp location
            smach.StateMachine.add("MOVE_TO_GRASP_SURFACE4",
                                   states.NavigateToSymbolic(robot,
                                                             {grasp_furniture_designator: "large_in_front_of"},
                                                             grasp_furniture_designator),
                                   transitions={'arrived': 'GRAB_ITEM_3',
                                                'unreachable': 'GRAB_ITEM_3',
                                                'goal_not_defined': 'GRAB_ITEM_3'})

            # Grasp the third item
            smach.StateMachine.add("GRAB_ITEM_3", GrabSingleItem(robot=robot, grab_designator=grasp_designator3),
                                   transitions={"succeeded": "MOVE_TO_PLACE_3",
                                                "failed": "MOVE_TO_PLACE_3"})

            # Move to the place location
            smach.StateMachine.add("MOVE_TO_PLACE_3",
                                   states.NavigateToSymbolic(robot,
                                                             {place_furniture_designator3: "in_front_of"},
                                                             place_furniture_designator3),
                                   transitions={'arrived': 'PLACE_ITEM_3',
                                                'unreachable': 'PLACE_ITEM_3',
                                                'goal_not_defined': 'PLACE_ITEM_3'})

            # Place the first item
            smach.StateMachine.add("PLACE_ITEM_3", PlaceSingleItem(robot=robot, place_designator=place_designator3),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "succeeded"})
