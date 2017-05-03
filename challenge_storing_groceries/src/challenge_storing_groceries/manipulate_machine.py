# ROS
import rospy
import smach

# TU/e
import robot_skills
import robot_smach_states as states
import robot_smach_states.util.designators as ds

# Challenge storing groceries
from entity_description_designator import EntityDescriptionDesignator
from config import TABLE, GRAB_SURFACE, DEFAULT_PLACE_ENTITY, DEFAULT_PLACE_AREA


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
        self._area_description = area_description

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
            if surface.in_volume(point=point, volume_id=self._area_description):
                entities.append(e)

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
        """ Constructor

        :param robot: robot object
        :param grab_designator: EdEntityDesignator designating the item to grab. If not provided, a default one is
        constructed (grabs the closest object in the volume of the surface)
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Create designators
        self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms, robot.leftArm, name="empty_arm_designator")
        self.grab_designator = ds.LockToId(robot=robot, to_be_locked=grab_designator)

        with self:
            @smach.cb_interface(outcomes=["locked"])
            def lock(userdata):
                """ 'Locks' a locking designator """
                # This determines that self.current_item cannot not resolve to a new value until it is unlocked again.
                self.grab_designator.lock()
                if self.grab_designator.resolve():
                    rospy.loginfo("Current_item is now locked to {0}".format(self.grab_designator.resolve().id))

                return "locked"

            smach.StateMachine.add("LOCK_ITEM",
                                   smach.CBState(lock),
                                   transitions={'locked': 'ANNOUNCE_ITEM'})

            smach.StateMachine.add("ANNOUNCE_ITEM",
                                   states.Say(robot, EntityDescriptionDesignator(self.grab_designator,
                                                                                 name="current_item_desc"),
                                              block=False),
                                   transitions={'spoken': 'GRAB_ITEM'})

            smach.StateMachine.add("GRAB_ITEM",
                                   states.Grab(robot, self.grab_designator, self.empty_arm_designator),
                                   transitions={'done': 'succeeded',
                                                'failed': 'failed'})

            # ToDo: see if this is desired/required
            # @smach.cb_interface(outcomes=["unlocked"])
            # def lock(userdata):
            #     """ 'Locks' a locking designator """
            #     # This determines that self.current_item cannot not resolve to a new value until it is unlocked again.
            #     self.grab_designator.unlock()
            #
            #     return "unlocked"
            #
            # smach.StateMachine.add("UNLOCK_ITEM_SUCCEED",
            #                        smach.CBState(lock),
            #                        transitions={'unlocked': 'succeeded'})
            #
            # smach.StateMachine.add("UNLOCK_ITEM_FAIL",
            #                        smach.CBState(lock),
            #                        transitions={'unlocked': 'failed'})


class PlaceSingleItem(smach.State):
    """ Tries to place an object. A 'place' statemachine is constructed dynamically since this makes it easier to
     build a statemachine (have we succeeded in grasping the objects?)"""
    def __init__(self, robot, place_designator=None):
        """ Constructor

        :param robot: robot object
        :param place_designator: Designator that resolves to the pose to place at. E.g. an EmptySpotDesignator
        """
        smach.State.__init__(self, outcomes=["succeeded", "failed"])

        self._robot = robot
        if place_designator is not None:
            self._place_designator = place_designator
        else:
            place_entity_designator = ds.EdEntityDesignator(robot=robot, id=DEFAULT_PLACE_ENTITY)
            self._place_designator = ds.EmptySpotDesignator(robot=robot,
                                                            place_location_designator=place_entity_designator,
                                                            area=DEFAULT_PLACE_AREA)

        # ToDo: unlock stuff?

    def execute(self, userdata):

        arm = None
        # See if there's an arm holding something
        for k, v in self._robot.arms.iteritems():
            if v.occupied_by is not None:
                arm = v
                break

        if arm is None:
            return "failed"

        item = ds.EdEntityDesignator(robot=self._robot, id=arm.occupied_by.id)
        arm_designator = ds.ArmDesignator(all_arms={arm.side: arm}, preferred_arm=arm)
        sm = states.Place(robot=self._robot, item_to_place=item, place_pose=self._place_designator, arm=arm_designator)
        result = sm.execute()

        # ToDo: if failed: do handover

        return "succeeded" if result == "done" else "failed"


class ManipulateMachine(smach.StateMachine):
    """The ManipulateMachine state machine performs the manipulation part of the storing groceries challenge:
    - Inspect the table
    - State item
    - Grab item
    - State item
    - Grab item
    - Drive to cabinet
    - State place shelf
    - Place item
    - State place shelf
    - Place item
    """
    def __init__(self, robot, grab_designator_1=None, grab_designator_2=None, place_designator=None, pdf_writer=None):
        """ Constructor
        :param robot: robot object
        :param grab_designator_1: EdEntityDesignator designating the item to grab
        :param grab_designator_2: EdEntityDesignator designating the item to grab
        :param pdf_writer: WritePDF object to save images of recognized objects to pdf files
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Create designators
        self._table_designator = ds.EntityByIdDesignator(robot, id=TABLE)
        if grab_designator_1 is None:
            grab_designator_1 = DefaultGrabDesignator(robot=robot, surface_designator=self._table_designator,
                                                      area_description="on_top_of")
        if grab_designator_2 is None:
            grab_designator_2 = DefaultGrabDesignator(robot=robot, surface_designator=self._table_designator,
                                                      area_description="on_top_of")

        with self:
            if pdf_writer:
                smach.StateMachine.add("INSPECT_TABLE", states.Inspect(robot=robot, entityDes=self._table_designator,
                                                                       objectIDsDes=None, searchArea="on_top_of",
                                                                       inspection_area="in_front_of"),
                                       transitions={"done": "WRITE_PDF",
                                                    "failed": "failed"})

                smach.StateMachine.add("WRITE_PDF", pdf_writer, transitions={"done": "GRAB_ITEM_1"})
            else:
                smach.StateMachine.add("INSPECT_TABLE", states.Inspect(robot=robot, entityDes=self._table_designator,
                                                                       objectIDsDes=None, searchArea="on_top_of",
                                                                       inspection_area="in_front_of"),
                                       transitions={"done": "GRAB_ITEM_1",
                                                    "failed": "failed"})

            smach.StateMachine.add("GRAB_ITEM_1", GrabSingleItem(robot=robot, grab_designator=grab_designator_1),
                                   transitions={"succeeded": "GRAB_ITEM_2",
                                                "failed": "GRAB_ITEM_2"})

            smach.StateMachine.add("GRAB_ITEM_2", GrabSingleItem(robot=robot, grab_designator=grab_designator_2),
                                   transitions={"succeeded": "PLACE_ITEM_1",
                                                "failed": "PLACE_ITEM_1"})

            smach.StateMachine.add("PLACE_ITEM_1", PlaceSingleItem(robot=robot, place_designator=place_designator),
                                   transitions={"succeeded": "PLACE_ITEM_2",
                                                "failed": "PLACE_ITEM_2"})

            smach.StateMachine.add("PLACE_ITEM_2", PlaceSingleItem(robot=robot, place_designator=place_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed"})
