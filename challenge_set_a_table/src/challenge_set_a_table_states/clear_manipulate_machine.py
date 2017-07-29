# ROS
import rospy
import smach

# TU/e
import robot_skills
import robot_smach_states as states
import robot_smach_states.util.designators as ds

# Challenge set the table
# from entity_description_designator import EntityDescriptionDesignator
from config import MIN_GRAB_OBJECT_HEIGHT, MAX_GRAB_OBJECT_WIDTH


class ManipulateMachine(smach.StateMachine):
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
    def __init__(self, robot, grasp_furniture_designator, grasp_designator,
                 place_furniture_designator, place_designator):
        """ Constructor
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Create designators
        grasp_furniture_designator = ds.EntityByIdDesignator(robot, id="kitchen_counter")
        grasp_furniture_designator = ds.EntityByIdDesignator(robot, id="kitchen_counter")
        grasp_furniture_designator = ds.EntityByIdDesignator(robot, id="kitchen_rack")
        grab_designator_1 = DefaultGrabDesignator(robot=robot, surface_designator=grasp_furniture_designator,
                                                  area_description=grasp_surface_id)
        grab_designator_2 = DefaultGrabDesignator(robot=robot, surface_designator=grasp_furniture_designator,
                                                  area_description=grasp_surface_id)
        grab_designator_3 = DefaultGrabDesignator(robot=robot, surface_designator=grasp_furniture_designator,
                                                  area_description=grasp_surface_id)

        place_furniture_designator = ds.EntityByIdDesignator(robot, id=place_furniture_id)
        place_designator = ds.EmptySpotDesignator(robot=robot,
                                                  place_location_designator=place_furniture_designator,
                                                  area=place_surface_id)

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
                                                                           searchArea=grasp_designator.area_description,
                                                                           navigation_area="in_front_of"),
                                   transitions={"done": "GRAB_ITEM_1",
                                                "failed": "failed"})

            # Grasp the first item
            smach.StateMachine.add("GRAB_ITEM_1", GrabSingleItem(robot=robot, grab_designator=grasp_designator),
                                   transitions={"succeeded": "GRAB_ITEM_2",
                                                "failed": "GRAB_ITEM_2"})

            # Grasp the second item
            smach.StateMachine.add("GRAB_ITEM_2", GrabSingleItem(robot=robot, grab_designator=grasp_designator),
                                   transitions={"succeeded": "MOVE_TO_PLACE",
                                                "failed": "MOVE_TO_PLACE"})

            # Move to the place location
            smach.StateMachine.add("MOVE_TO_PLACE",
                                   states.NavigateToSymbolic(robot,
                                                             {place_furniture_designator: "in_front_of"},
                                                             place_furniture_designator),
                                   transitions={'arrived': 'PLACE_ITEM_1',
                                                'unreachable': 'PLACE_ITEM_1',
                                                'goal_not_defined': 'PLACE_ITEM_1'})

            # Place the first item
            smach.StateMachine.add("PLACE_ITEM_1", PlaceSingleItem(robot=robot, place_designator=place_designator),
                                   transitions={"succeeded": "PLACE_ITEM_2",
                                                "failed": "PLACE_ITEM_2"})

            # Place the second item
            smach.StateMachine.add("PLACE_ITEM_2", PlaceSingleItem(robot=robot, place_designator=place_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed"})

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
            smach.StateMachine.add("GRAB_ITEM_3", GrabSingleItem(robot=robot, grab_designator=grasp_designator),
                                   transitions={"succeeded": "MOVE_TO_PLACE_3",
                                                "failed": "MOVE_TO_PLACE_3"})

            # Move to the place location
            smach.StateMachine.add("MOVE_TO_PLACE_3",
                                   states.NavigateToSymbolic(robot,
                                                             {place_furniture_designator: "in_front_of"},
                                                             place_furniture_designator),
                                   transitions={'arrived': 'PLACE_ITEM_3',
                                                'unreachable': 'PLACE_ITEM_3',
                                                'goal_not_defined': 'PLACE_ITEM_3'})

            # Place the first item
            smach.StateMachine.add("PLACE_ITEM_3", PlaceSingleItem(robot=robot, place_designator=place_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "succeeded"})
