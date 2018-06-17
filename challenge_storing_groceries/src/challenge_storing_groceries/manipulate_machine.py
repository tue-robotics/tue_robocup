# system
import math
import numpy as np

# ROS
import rospy
import smach

# TU/e
from cb_planner_msgs_srvs.msg import PositionConstraint
import robot_skills
from robot_skills.util.entity import Entity
import robot_smach_states as states
import robot_smach_states.util.designators as ds
from robot_skills.util.kdl_conversions import FrameStamped, kdl_frame_stamped_from_XYZRPY

# Challenge storing groceries
from entity_description_designator import EntityDescriptionDesignator
# from config import TABLE, GRAB_SURFACE, DEFAULT_PLACE_ENTITY, DEFAULT_PLACE_AREA, CABINET
from config import GRAB_SURFACE
from config import MIN_GRAB_OBJECT_HEIGHT, MAX_GRAB_OBJECT_WIDTH
from inspect_shelves import InspectShelves


class DefaultGrabDesignator(ds.Designator):
    """ Designator to pick the closest item on top of the table to grab. This is used for testing

    """
    def __init__(self, robot, surface_designator, area_description, name=None):
        """ Constructor

        :param robot: robot object
        :param surface_designator: designator for the object to grab from
        :param area_description: string with id of the area where the object should be located in
        """
        super(DefaultGrabDesignator, self).__init__(resolve_type=robot_skills.util.entity.Entity, name=name)

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

        rospy.loginfo("Gathering all entities in {vol} of surface {ent}".format(vol=self._area_description, ent=surface))
        # import ipdb; ipdb.set_trace()
        for e in all_entities:
            point = robot_skills.util.kdl_conversions.VectorStamped(frame_id=e.frame_id, vector=e._pose.p)
            if surface.in_volume(point=point, volume_id=self._area_description):
                entities.append(e)
        rospy.loginfo("{l} entities in {vol} of surface {ent}".format(l=len(entities), vol=self._area_description, ent=surface.id))

        # Remove all entities that are too large or too small
        entities = [e for e in entities if (e.shape.z_max - e.shape.z_min) > MIN_GRAB_OBJECT_HEIGHT]
        entities = [e for e in entities if (e.shape.y_max - e.shape.y_min) < MAX_GRAB_OBJECT_WIDTH]
        entities = [e for e in entities if (e.shape.x_max - e.shape.x_min) < MAX_GRAB_OBJECT_WIDTH]
        rospy.loginfo("Keeping {l} entities in {vol} of surface {ent} that are not too big".format(l=len(entities), vol=self._area_description, ent=surface.id))

        # Check if there are any
        if not entities:
            return None

        # Sort the entities and return the closest one
        base_pose = self._robot.base.get_location().frame
        entities = sorted(entities, key=lambda e: e.distance_to_2d(base_pose.p))
        return entities[0]


class PlaceWithAlikeObjectDesignator(ds.EmptySpotDesignator):
    """Find a FrameStamped where to place an object if a class 'A' besides another object of class 'A'
    """

    def __init__(self, robot, entity_to_place_designator, place_location_designator, areas=None, name=None, debug=False):
        super(PlaceWithAlikeObjectDesignator, self).__init__(robot, place_location_designator, name)

        self.entity_to_place_designator = entity_to_place_designator
        self.areas = areas

        self._debug = debug

    def _resolve(self):
        """
        :return: Where can an object be placed to put it besides an object of the same type
        :returns: FrameStamped
        """
        if self._debug:
            import ipdb; ipdb.set_trace()

        entity_to_place = self.entity_to_place_designator.resolve()
        assert isinstance(entity_to_place, Entity)

        rospy.loginfo("The grasped entity is a '{}'".format(entity_to_place.type))

        all_entities = self.robot.ed.get_entities(parse=False)  # type: List[Entity]
        place_location = self.place_location_designator.resolve()
        entities_inside_placement = []
        for area in self.areas:
            entities_inside_placement += place_location.entities_in_volume(all_entities, area)

        rospy.loginfo("{l} entities inside the place_location '{pl}': {eip}".format(l=len(entities_inside_placement),
                                                                                    pl=place_location.id,
                                                                                    eip=[(e.id[:6], e.type) for e in entities_inside_placement]))

        # Entities that the entity_to_place could be placed besides (of same or similar type)
        candidate_entities_for_pairing = [e for e in entities_inside_placement if self._same_class(entity_to_place, e)]  # type: List[Entity]

        rospy.loginfo("Entities we could pair with: {}".format([(e.id[:6], e.type) for e in candidate_entities_for_pairing]))

        # FrameStamped's the entity_to_place could be placed at. Each maps to the entity it is besides.
        # This way can relate and refer to the entity we pick eventually and say something about it
        all_placement_candidates = {}  # type: List[FrameStamped]

        for candidate_ent in candidate_entities_for_pairing:
            placements = self._generate_placements_beside(candidate_ent)
            for placement in placements:
                all_placement_candidates[placement] = candidate_ent

        unoccupied_placement_candidates = filter(self.is_poi_occupied, all_placement_candidates.keys())

        self.marker_array.markers = [self.create_marker(fs.frame.p.x(), fs.frame.p.y(), fs.frame.p.z()) for fs in unoccupied_placement_candidates]
        self.marker_pub.publish(self.marker_array)

        best_placement = self.select_best_feasible_poi(unoccupied_placement_candidates)
        if best_placement:
            place_besides = all_placement_candidates[best_placement]
            rospy.loginfo("Placing besides {} at {}".format(place_besides, best_placement))
            return best_placement
        else:
            rospy.logwarn("Could not find an entity we could place besides so just placing somewhere")
            return super(PlaceWithAlikeObjectDesignator, self)._resolve()

    def _same_class(self, entity_a, entity_b):
        if entity_a.type == entity_b.type:
            return True
        elif entity_b.super_types and entity_a.is_a(entity_b.super_types[0]):
            return True
        else:
            return False

    def _generate_placements_beside(self, entity):
        placements = self._generate_around(entity.pose, 0.15, 8)

        # TODO: This is an ugly hack to make select_best_feasible_poi work, because super().determine_points_of_interest() also does this.
        for placement in placements:
            setattr(placement, 'edge_score', 0)
        return placements

    @staticmethod
    def _generate_around(framestamped, radius, n):
        angles = np.linspace(0, 2 * np.pi, n)
        vector = framestamped.extractVectorStamped().vector
        x, y, z = vector.x(), vector.y(), vector.z()
        xs = np.cos(angles) + x
        ys = np.sin(angles) + y
        frame_stampeds = [kdl_frame_stamped_from_XYZRPY(xi, yi, z, 0, 0, 0, frame_id=framestamped.frame_id) for
                                   xi, yi in zip(xs, ys)]
        return frame_stampeds


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
            smach.StateMachine.add("ANNOUNCE_ITEM",
                                   states.Say(robot, EntityDescriptionDesignator(self.grab_designator,
                                                                                 name="current_item_desc"),
                                              block=False),
                                   transitions={'spoken': 'GRAB_ITEM'})

            smach.StateMachine.add("GRAB_ITEM",
                                   states.Grab(robot, self.grab_designator, self.empty_arm_designator),
                                   transitions={'done': 'succeeded',
                                                'failed': 'failed'})


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
        if place_designator is not None:
            self.place_designator = place_designator
        # else:
        #     place_entity_designator = ds.EdEntityDesignator(robot=robot, id=DEFAULT_PLACE_ENTITY)
        #     self._place_designator = ds.EmptySpotDesignator(robot=robot,
        #                                                     place_location_designator=place_entity_designator,
        #                                                     area=DEFAULT_PLACE_AREA)

        # ToDo: unlock stuff?

    def execute(self, userdata=None):

        arm = None
        # See if there's an arm holding something
        for k, v in self._robot.arms.iteritems():
            if v.occupied_by is not None:
                arm = v
                break

        if arm is None:
            rospy.logwarn("Arm is None")
            return "failed"

        # Try to place the object
        item = ds.EdEntityDesignator(robot=self._robot, id=arm.occupied_by.id)
        arm_designator = ds.ArmDesignator(all_arms={arm.side: arm}, preferred_arm=arm)
        place = states.Place(robot=self._robot, item_to_place=item, place_pose=self.place_designator, arm=arm_designator)
        result = place.execute()

        # If failed, do handover to human in order to continue
        if result != "done":
            rospy.loginfo("{place} resulted in {out}".format(place=place, out=result))

            handover = states.HandoverToHuman(robot=self._robot, arm_designator=arm_designator)
            handover.execute()

        return "succeeded" if result == "done" else "failed"


class LockToFrameStamped(ds.Designator):
    """A designator for FrameStamped may generate a different FrameStamped every time.
    By locking, it returns the same f.s. everytime while locked"""

    def __init__(self, to_be_locked, name=None):
        """ Constructor

        :param robot: robot object
        :param to_be_locked: designator to be locked
        :param name: (optional) might come in handy for debugging
        """
        super(LockToFrameStamped, self).__init__(resolve_type=to_be_locked.resolve_type, name=name)
        self.to_be_locked = to_be_locked
        self._locked = False

        self._locked_value = None

    def lock(self):
        self._locked = True

    def unlock(self):
        self._locked = False

    def _resolve(self):
        if self._locked:  # If we should resolve to a remembered thing
            if not self._locked_value:  # but we haven't remembered anything yet
                fs = self.to_be_locked.resolve()  # Then find out what we should remember
                if fs:  # If we can find what to remember
                    self._locked_value = fs  # remember!
            else:  # If we do remember something already, recall that remembered ID:
                return self._locked_value
        else:
            fs = self.to_be_locked.resolve()
            rospy.loginfo("{0} resolved to {1}, but is *not locked* to it".format(self, fs))
            return fs

    def __repr__(self):
        return "LockToFrameStamped({})._locked = {}".format(self.to_be_locked, self._locked)


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
        self.table_designator = ds.EntityByIdDesignator(robot, id="temp")  # will be updated later on
        if grab_designator_1 is None:
            grab_designator_1 = DefaultGrabDesignator(robot=robot, surface_designator=self.table_designator,
                                                      area_description=GRAB_SURFACE,
                                                      name="grab_1")
        if grab_designator_2 is None:
            grab_designator_2 = DefaultGrabDesignator(robot=robot, surface_designator=self.table_designator,
                                                      area_description=GRAB_SURFACE,
                                                      name="grab_2")
        self.cabinet = ds.EntityByIdDesignator(robot, id="temp")  # will be updated later on

        self.place_designator1 = LockToFrameStamped(PlaceWithAlikeObjectDesignator(robot=robot,
                                                                entity_to_place_designator=grab_designator_1,
                                                                place_location_designator=self.cabinet,
                                                                areas=['shelf2', 'shelf3'],
                                                                name="place_1",
                                                                debug=False
                                                                ))
        self.place_designator2 = LockToFrameStamped(PlaceWithAlikeObjectDesignator(robot=robot,
                                                                entity_to_place_designator=grab_designator_2,
                                                                place_location_designator=self.cabinet,
                                                                areas=['shelf2', 'shelf3'],
                                                                name="place_2",
                                                                debug=False
                                                                ))

        self.placeaction1 = PlaceSingleItem(robot=robot, place_designator=self.place_designator1)
        self.placeaction2 = PlaceSingleItem(robot=robot, place_designator=self.place_designator2)

        with self:

            smach.StateMachine.add("MOVE_TO_TABLE1",
                                   states.NavigateToSymbolic(robot,
                                                             {self.table_designator: "in_front_of"},
                                                             self.table_designator),
                                   transitions={'arrived': 'INSPECT_TABLE',
                                                'unreachable': 'MOVE_TO_TABLE2',
                                                'goal_not_defined': 'INSPECT_TABLE'})

            smach.StateMachine.add("MOVE_TO_TABLE2",
                                   states.NavigateToSymbolic(robot,
                                                             {self.table_designator: "in_front_of"},
                                                             self.table_designator),
                                   transitions={'arrived': 'INSPECT_TABLE',
                                                'unreachable': 'INSPECT_TABLE',
                                                'goal_not_defined': 'INSPECT_TABLE'})

            if pdf_writer:
                # Designator to store the classificationresults
                # The Inspect-state (INSPECT_TABLE) gathers a list of ClassificationResults for Entities on the table
                # These are passed to the pdf_writer
                class_designator = ds.VariableDesignator(
                    [], resolve_type=[robot_skills.classification_result.ClassificationResult])

                # Add the designator to the pdf writer state
                pdf_writer.set_designator(class_designator)

                smach.StateMachine.add("INSPECT_TABLE", states.Inspect(robot=robot, entityDes=self.table_designator,
                                                                       objectIDsDes=class_designator,
                                                                       searchArea=GRAB_SURFACE,
                                                                       navigation_area="in_front_of"),
                                       transitions={"done": "WRITE_PDF",
                                                    "failed": "failed"})

                smach.StateMachine.add("WRITE_PDF", pdf_writer, transitions={"done": "GRAB_ITEM_1"})
            else:
                smach.StateMachine.add("INSPECT_TABLE", states.Inspect(robot=robot, entityDes=self.table_designator,
                                                                       objectIDsDes=None, searchArea=GRAB_SURFACE,
                                                                       navigation_area="in_front_of"),
                                       transitions={"done": "LOCK_ALL",
                                                    "failed": "failed"})

            @smach.cb_interface(outcomes=["locked"])
            def lock(userdata=None):
                grab_designator1.lock()
                grab_designator2.lock()
                self.place_designator1.lock()
                self.place_designator2.lock()

                rospy.loginfo("All designators locked")

                return "locked"

            @smach.cb_interface(outcomes=["unlocked"])
            def unlock(userdata=None):
                grab_designator1.unlock()
                grab_designator2.unlock()
                self.place_designator1.unlock()
                self.place_designator2.unlock()

                rospy.loginfo("All designators UNlocked")

                return "unlocked"

            smach.StateMachine.add("LOCK_ALL",
                                   smach.CBState(lock),
                                   transitions={'locked': 'GRAB_ITEM_1'})

            smach.StateMachine.add("GRAB_ITEM_1", GrabSingleItem(robot=robot, grab_designator=grab_designator_1),
                                   transitions={"succeeded": "GRAB_ITEM_2",
                                                "failed": "GRAB_ITEM_2"})

            smach.StateMachine.add("GRAB_ITEM_2", GrabSingleItem(robot=robot, grab_designator=grab_designator_2),
                                   transitions={"succeeded": "MOVE_TO_PLACE",
                                                "failed": "MOVE_TO_PLACE"})

            smach.StateMachine.add("MOVE_TO_PLACE",
                                   states.NavigateToSymbolic(robot,
                                                             {self.cabinet: "in_front_of"},
                                                             self.cabinet),
                                   transitions={'arrived': 'INSPECT_SHELVES',
                                                'unreachable': 'INSPECT_SHELVES',
                                                'goal_not_defined': 'INSPECT_SHELVES'})

            smach.StateMachine.add("INSPECT_SHELVES",
                                   InspectShelves(robot, self.cabinet),
                                   transitions={'succeeded': 'WRITE_PDF_SHELVES',
                                                'nothing_found': 'WRITE_PDF_SHELVES',
                                                'failed': 'WRITE_PDF_SHELVES'})

            smach.StateMachine.add("WRITE_PDF_SHELVES", pdf_writer, transitions={"done": "PLACE_ITEM_1"})

            smach.StateMachine.add("PLACE_ITEM_1", self.placeaction1,
                                   transitions={"succeeded": "PLACE_ITEM_2",
                                                "failed": "PLACE_ITEM_2"})

            smach.StateMachine.add("PLACE_ITEM_2", self.placeaction2,
                                   transitions={"succeeded": "UNLOCK_ALL_SUCCEEDED",
                                                "failed": "UNLOCK_ALL_FAILED"})

            smach.StateMachine.add("UNLOCK_ALL_SUCCEEDED",
                                   smach.CBState(unlock),
                                   transitions={'unlocked': 'succeeded'})
            smach.StateMachine.add("UNLOCK_ALL_FAILED",
                                   smach.CBState(unlock),
                                   transitions={'unlocked': 'failed'})


