# ROS
import rospy
import smach

# TU/e
import robot_skills
from robot_skills import arms
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_smach_states.manipulation.place_designator import EmptySpotDesignator

# Challenge set the table
from config import MIN_GRAB_OBJECT_HEIGHT, MAX_GRAB_OBJECT_WIDTH


class DefaultGrabDesignator(ds.Designator):
    """ Designator to pick the closest item on top of the table to grab. This is used for testing

    """
    def __init__(self, robot, surface_designator, area_description):
        """ Constructor

        :param robot: robot object
        :param surface_designator: designator for the object to grab from
        :param area_description: string with id of the area where the object should be located in
        """
        super(DefaultGrabDesignator, self).__init__(resolve_type=robot_skills.util.entity.Entity)

        self._robot = robot
        self._surface_designator = surface_designator
        self.area_description = area_description

    def resolve(self):
        """ Resolves

        :return: entity in the <area_description> of the <surface_designator> that is closest to the robot
        """
        # Get the surface as an entity
        surface = self._surface_designator.resolve()
        if surface is None:
            rospy.logerror("Cannot resolve surface designator")
            return None

        # Get all entities and check which ones are on the table
        all_entities = self._robot.ed.get_entities()
        entities = []
        for e in all_entities:
            point = robot_skills.util.kdl_conversions.VectorStamped(frame_id=e.frame_id, vector=e._pose.p)
            if surface.in_volume(point=point, volume_id=self.area_description):
                entities.append(e)

        # Remove all entities that are too large or too small
        entities = [e for e in entities if (e.shape.z_max - e.shape.z_min) > MIN_GRAB_OBJECT_HEIGHT]
        entities = [e for e in entities if (e.shape.y_max - e.shape.y_min) < MAX_GRAB_OBJECT_WIDTH]
        entities = [e for e in entities if (e.shape.x_max - e.shape.x_min) < MAX_GRAB_OBJECT_WIDTH]

        # Check if there are any
        if not entities:
            return None

        # Sort the entities and return the closest one
        base_pose = self._robot.base.get_location().frame
        entities = sorted(entities, key=lambda e: e.distance_to_2d(base_pose.p))
        return entities[0]


class GrabSingleItem(smach.StateMachine):
    """ Lock an object, announce it and grab it """
    def __init__(self, robot, grab_designator=None):
        """
        Constructor

        :param robot: robot object
        :param grab_designator: EdEntityDesignator designating the item to grab. If not provided, a default one is
            constructed (grabs the closest object in the volume of the surface)
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Create designators
        self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot,
                                                               {"required_trajectories": ["prepare_grasp"],
                                                                "required_goals": ["carrying_pose"],
                                                                "required_gripper_types": [arms.GripperTypes.GRASPING]},
                                                               name="empty_arm_designator")
        self.grab_designator = ds.LockToId(robot=robot, to_be_locked=grab_designator)

        with self:
            @smach.cb_interface(outcomes=["locked"])
            def lock(userdata=None):
                """ 'Locks' a locking designator """
                # This determines that self.current_item cannot not resolve to a new value until it is unlocked again.
                self.grab_designator.lock()
                if self.grab_designator.resolve():
                    rospy.loginfo("Current_item is now locked to {0}".format(self.grab_designator.resolve().id))

                return "locked"

            smach.StateMachine.add("LOCK_ITEM",
                                   smach.CBState(lock),
                                   transitions={'locked': 'GRAB_ITEM'})

            smach.StateMachine.add("GRAB_ITEM",
                                   states.Grab(robot, self.grab_designator, self.empty_arm_designator),
                                   transitions={'done': 'UNLOCK_ITEM_SUCCEED',
                                                'failed': 'UNLOCK_ITEM_FAIL'})

            @smach.cb_interface(outcomes=["unlocked"])
            def unlock(userdata=None):
                """ 'Unlocks' a locking designator """
                self.grab_designator.unlock()

                return "unlocked"

            smach.StateMachine.add("UNLOCK_ITEM_SUCCEED",
                                   smach.CBState(unlock),
                                   transitions={'unlocked': 'succeeded'})

            smach.StateMachine.add("UNLOCK_ITEM_FAIL",
                                   smach.CBState(unlock),
                                   transitions={'unlocked': 'failed'})


class PlaceSingleItem(smach.State):
    """ Tries to place an object. A 'place' statemachine is constructed dynamically since this makes it easier to
     build a statemachine (have we succeeded in grasping the objects?)"""
    def __init__(self, robot, place_designator):
        """ Constructor

        :param robot: robot object
        :param place_designator: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self._robot = robot
        self._place_designator = place_designator

    def execute(self, userdata=None):
        # Try to place the object
        item = ds.EdEntityDesignator(robot=self._robot, id=arm.occupied_by.id)
        arm_designator = ds.OccupiedArmDesignator(self._robot,
                                                  arm_properties={
                                                      "required_trajectories": ["prepare_place"],
                                                      "required_goals": ["reset", "handover_to_human"],
                                                      "required_gripper_types": [arms.GripperTypes.GRASPING]})
        resolved_arm = arm_designator.resolve()
        if resolved_arm is None:
            rospy.logwarn("No arm holding an entity")
            return "failed"

        sm = states.Place(robot=self._robot, item_to_place=item, place_pose=self._place_designator, arm=arm_designator)
        result = sm.execute()

        # If failed, do handover to human in order to continue
        if result != "done":
            sm = states.HandoverToHuman(robot=self._robot, arm_designator=arm_designator)
            sm.execute()

        return "succeeded" if result == "done" else "failed"


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
    def __init__(self, robot, grasp_designator1, grasp_designator2, grasp_designator3,
                 grasp_furniture_id1, grasp_furniture_id2, place_furniture_id):
        """ Constructor
        :param robot: robot object
        :param grasp_designator1: EdEntityDesignator designating the first item to grab.
        :param grasp_designator2: EdEntityDesignator designating the second item to grab.
        :param grasp_designator3: EdEntityDesignator designating the third item to grab.
        :param grasp_furniture_id1: string identifying the location where to grasp objects 1 and 2
        :param grasp_furniture_id3: string identifying the location where to grasp object 3
        :param place_furniture_id: string identifying the location where to place the objects
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Create designators
        grasp_furniture_designator1 = ds.EntityByIdDesignator(robot, id=grasp_furniture_id1)
        grasp_furniture_designator2 = ds.EntityByIdDesignator(robot, id=grasp_furniture_id2)

        place_furniture_designator = ds.EntityByIdDesignator(robot, id=place_furniture_id)
        arm_designator = ds.ArmDesignator(robot, {})
        place_designator = EmptySpotDesignator(robot=robot,
                                               place_location_designator=place_furniture_designator,
                                               arm_designator=arm_designator,
                                               area="on_top_of")

        with self:

            # Move to the inspect location
            smach.StateMachine.add("MOVE_TO_GRASP_SURFACE1",
                                   states.NavigateToSymbolic(robot,
                                                             {grasp_furniture_designator1: "in_front_of"},
                                                             grasp_furniture_designator1),
                                   transitions={'arrived': 'INSPECT_GRASP_SURFACE',
                                                'unreachable': 'MOVE_TO_GRASP_SURFACE2',
                                                'goal_not_defined': 'INSPECT_GRASP_SURFACE'})

            # Backup for moving to inspect location
            smach.StateMachine.add("MOVE_TO_GRASP_SURFACE2",
                                   states.NavigateToSymbolic(robot,
                                                             {grasp_furniture_designator1: "large_in_front_of"},
                                                             grasp_furniture_designator1),
                                   transitions={'arrived': 'INSPECT_GRASP_SURFACE',
                                                'unreachable': 'INSPECT_GRASP_SURFACE',
                                                'goal_not_defined': 'INSPECT_GRASP_SURFACE'})

            # Inspect grasp furniture
            smach.StateMachine.add("INSPECT_GRASP_SURFACE", states.Inspect(robot=robot,
                                                                           entityDes=grasp_furniture_designator1,
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
                                   transitions={"succeeded": "MOVE_TO_GRASP_SURFACE3",
                                                "failed": "MOVE_TO_GRASP_SURFACE3"})

            # Move back to the grasp surface to grasp the third item
            smach.StateMachine.add("MOVE_TO_GRASP_SURFACE3",
                                   states.NavigateToSymbolic(robot,
                                                             {grasp_furniture_designator2: "in_front_of"},
                                                             grasp_furniture_designator2),
                                   transitions={'arrived': 'INSPECT_GRASP_SURFACE2',
                                                'unreachable': 'MOVE_TO_GRASP_SURFACE4',
                                                'goal_not_defined': 'INSPECT_GRASP_SURFACE2'})

            # Backup for moving back to the grasp location
            smach.StateMachine.add("MOVE_TO_GRASP_SURFACE4",
                                   states.NavigateToSymbolic(robot,
                                                             {grasp_furniture_designator2: "large_in_front_of"},
                                                             grasp_furniture_designator2),
                                   transitions={'arrived': 'INSPECT_GRASP_SURFACE2',
                                                'unreachable': 'INSPECT_GRASP_SURFACE2',
                                                'goal_not_defined': 'INSPECT_GRASP_SURFACE2'})

            # Inspect grasp furniture
            smach.StateMachine.add("INSPECT_GRASP_SURFACE2", states.Inspect(robot=robot,
                                                                            entityDes=grasp_furniture_designator2,
                                                                            objectIDsDes=None,
                                                                            searchArea="shelf2",
                                                                            navigation_area="in_front_of"),
                                   transitions={"done": "GRAB_ITEM_3",
                                                "failed": "failed"})

            # Grasp the third item
            smach.StateMachine.add("GRAB_ITEM_3", GrabSingleItem(robot=robot, grab_designator=grasp_designator3),
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
