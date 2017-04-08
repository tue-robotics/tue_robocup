# import rospy
# import smach
# import robot_smach_states as states
# from robot_smach_states import Grab
# from robot_smach_states import Place
# import robot_smach_states.util.designators as ds
# from robot_skills.util import transformations
#
# from empty_shelf_designator import EmptyShelfDesignator
# from entity_description_designator import EntityDescriptionDesignator
# from config import *
# from remove_segmented_entities import RemoveSegmentedEntities
# from segment_shelf import SegmentShelf
# ROS
import PyKDL as kdl
import rospy
import smach

# TU/e
import robot_skills
import robot_smach_states as states
import robot_smach_states.util.designators as ds

# Challenge storing groceries
from entity_description_designator import EntityDescriptionDesignator
from config import ROOM, TABLE


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
        base_pose = self._robot.base.get_location()
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
                                   transitions={'done': 'UNLOCK_ITEM_SUCCEED',
                                                'failed': 'UNLOCK_ITEM_FAIL'})

            @smach.cb_interface(outcomes=["unlocked"])
            def lock(userdata):
                """ 'Locks' a locking designator """
                # This determines that self.current_item cannot not resolve to a new value until it is unlocked again.
                self.grab_designator.unlock()

                return "unlocked"

            smach.StateMachine.add("UNLOCK_ITEM_SUCCEED",
                                   smach.CBState(lock),
                                   transitions={'unlocked': 'succeeded'})

            smach.StateMachine.add("UNLOCK_ITEM_FAIL",
                                   smach.CBState(lock),
                                   transitions={'unlocked': 'failed'})


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
    def __init__(self, robot, grab_designator=None):
        """ Constructor
        :param robot: robot object
        :param grab_designator: EdEntityDesignator designating the item to grab
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        # Create designators
        self._table_designator = ds.EntityByIdDesignator(robot, id=TABLE)
        if grab_designator is None:
            grab_designator = DefaultGrabDesignator(robot=robot, surface_designator=self._table_designator,
                                                    area_description="on_top_of")


        with self:
            smach.StateMachine.add("INSPECT_TABLE", states.Inspect(robot=robot, entityDes=self._table_designator,
                                                                   objectIDsDes=None, searchArea="on_top_of",
                                                                   inspection_area="in_front_of"),
                                   transitions={"done": "GRAB_ITEM_1",
                                                "failed": "failed"})

            smach.StateMachine.add("GRAB_ITEM_1", GrabSingleItem(robot=robot, grab_designator=grab_designator),
                                   transitions={"succeeded": "GRAB_ITEM_2",
                                                "failed": "GRAB_ITEM_2"})

            smach.StateMachine.add("GRAB_ITEM_2", GrabSingleItem(robot=robot, grab_designator=grab_designator),
                                   transitions={"succeeded": "succeeded",
                                                "failed": "failed"})

            # smach.StateMachine.add("NAV_TO_OBSERVE_TABLE",
            #                        states.NavigateToSymbolic(robot, {self._table_designator: "in_front_of",
            #                                                          EntityByIdDesignator(robot, id=ROOM): "in"},
            #                                                  self._table_designator),
            #                        transitions={'arrived': 'succeeded',
            #                                     'unreachable': 'succeeded',
            #                                     'goal_not_defined': 'succeeded'}) #LOOKAT_TABLE

    # def __init__(self, robot, manipulated_items):
    #     """@param manipulated_items is VariableDesignator that will be a list of items manipulated by the robot."""
    #     self.manipulated_items = manipulated_items
    #     smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
    #
    #     self.cabinet = ds.EntityByIdDesignator(robot, id=CABINET, name="pick_shelf")
    #     # self.place_shelf = ds.EntityByIdDesignator(robot, id=PLACE_SHELF, name="place_shelf")
    #
    #     not_manipulated = lambda entity: not entity in self.manipulated_items.resolve()
    #
    #     def entity_z_pos(entity):
    #         """ Checks if the entity is between the minimum and maximum grasp height
    #         :param entity:
    #         :return:
    #         """
    #         if not entity.has_pose:
    #             return False
    #         return MIN_GRASP_HEIGHT < entity.pose.position.z < MAX_GRASP_HEIGHT
    #
    #     # select the entity closest in x direction to the robot in base_link frame
    #     def weight_function(entity):
    #         # TODO: return x coordinate of entity.center_point in base_link frame
    #         p = transformations.tf_transform(entity.pose.position, "/map", robot.robot_name+"/base_link", robot.tf_listener)
    #         return p.x*p.x
    #
    #     self.current_item = ds.LockingDesignator(ds.EdEntityDesignator(robot,
    #                                                                    criteriafuncs=[not_ignored, size,
    #                                                                                   not_manipulated,
    #                                                                                   min_entity_height, entity_z_pos,
    #                                                                                   max_width],
    #                                                                    weight_function=weight_function, debug=False,
    #                                                                    name="item"), name="current_item")
    #
    #     #This makes that the empty spot is resolved only once, even when the robot moves. This is important because the sort is based on distance between robot and constrait-area
    #     # self.place_position = ds.LockingDesignator(ds.EmptySpotDesignator(robot, self.cabinet, name="placement", area=PLACE_SHELF), name="place_position")
    #     self.place_position = ds.LockingDesignator(EmptyShelfDesignator(robot, self.cabinet,
    #                                                                     name="placement", area=PLACE_SHELF),
    #                                                name="place_position")
    #
    #     if PREFERRED_ARM == "left":
    #         prefered_arm = robot.leftArm
    #     elif PREFERRED_ARM == "right":
    #         prefered_arm = robot.rightArm
    #     else:
    #         rospy.logwarn("Impossible preferred arm: {0}, defaulting to left".format(PREFERRED_ARM))
    #         prefered_arm = robot.leftArm
    #
    #     self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms, prefered_arm, name="empty_arm_designator")
    #     self.arm_with_item_designator = ds.ArmHoldingEntityDesignator(robot.arms, self.current_item, name="arm_with_item_designator")
    #
    #     # print "{0} = pick_shelf".format(self.pick_shelf)
    #     # print "{0} = current_item".format(self.current_item)
    #     # print "{0} = place_position".format(self.place_position)
    #     # print "{0} = empty_arm_designator".format(self.empty_arm_designator)
    #     # print "{0} = arm_with_item_designator".format(self.arm_with_item_designator)
    #
    #     with self:
    #         # smach.StateMachine.add( "NAV_TO_OBSERVE_PICK_SHELF",
    #         #                         #states.NavigateToObserve(robot, self.pick_shelf),
    #         #                         states.NavigateToSymbolic(robot, {self.pick_shelf:"in_front_of", EntityByIdDesignator(robot, id=ROOM):"in"}, self.pick_shelf),
    #         #                         transitions={   'arrived'           :'LOOKAT_PICK_SHELF',
    #         #                                         'unreachable'       :'LOOKAT_PICK_SHELF',
    #         #                                         'goal_not_defined'  :'LOOKAT_PICK_SHELF'})
    #
    #         smach.StateMachine.add("REMOVE_ENTITIES",
    #                                    RemoveSegmentedEntities(robot=robot),
    #                                transitions={'done': 'LOOKAT_PICK_SHELF'})
    #
    #         smach.StateMachine.add("LOOKAT_PICK_SHELF",
    #                                  states.LookAtArea(robot, self.cabinet, area=PICK_SHELF),
    #                                  transitions={  'succeeded'         :'SEGMENT_SHELF'})
    #
    #         smach.StateMachine.add("SEGMENT_SHELF",
    #                                SegmentShelf(robot, entity_id=CABINET, area_id=PICK_SHELF),
    #                                transitions={'done': 'LOCK_ITEM'})
    #
    #         @smach.cb_interface(outcomes=['locked'])
    #         def lock(userdata):
    #             self.current_item.lock() #This determines that self.current_item cannot not resolve to a new value until it is unlocked again.
    #             if self.current_item.resolve():
    #                 rospy.loginfo("Current_item is now locked to {0}".format(self.current_item.resolve().id))
    #
    #             self.place_position.lock() #This determines that self.place_position will lock/cache its result after its resolved the first time.
    #             return 'locked'
    #         smach.StateMachine.add('LOCK_ITEM',
    #                                smach.CBState(lock),
    #                                transitions={'locked':'ANNOUNCE_ITEM'})
    #
    #         smach.StateMachine.add( "ANNOUNCE_ITEM",
    #                                 states.Say(robot, EntityDescriptionDesignator(self.current_item,
    #                                                                               name="current_item_desc"),
    #                                            block=False),
    #                                 transitions={   'spoken'            :'GRAB_ITEM'})
    #
    #         smach.StateMachine.add( "GRAB_ITEM",
    #                                 Grab(robot, self.current_item, self.empty_arm_designator),
    #                                 transitions={   'done'              :'STORE_ITEM',
    #                                                 'failed'            :'SAY_GRAB_FAILED'})
    #
    #         smach.StateMachine.add( "SAY_GRAB_FAILED",
    #                                 states.Say(robot, ["I couldn't grab this thing"], mood="sad"),
    #                                 transitions={   'spoken'            :'UNLOCK_ITEM_AFTER_FAILED_GRAB'}) # Not sure whether to fail or keep looping with NAV_TO_OBSERVE_PICK_SHELF
    #
    #         @smach.cb_interface(outcomes=['unlocked'])
    #         def unlock_and_ignore(userdata):
    #             global ignore_ids
    #             # import ipdb; ipdb.set_trace()
    #             if self.current_item.resolve():
    #                 ignore_ids += [self.current_item.resolve().id]
    #                 rospy.loginfo("Current_item WAS now locked to {0}".format(self.current_item.resolve().id))
    #             self.current_item.unlock() #This determines that self.current_item can now resolve to a new value on the next call
    #             self.place_position.unlock() #This determines that self.place_position can now resolve to a new position on the next call
    #             return 'unlocked'
    #         smach.StateMachine.add('UNLOCK_ITEM_AFTER_FAILED_GRAB',
    #                                smach.CBState(unlock_and_ignore),
    #                                transitions={'unlocked'              :'failed'})
    #
    #         @smach.cb_interface(outcomes=['stored'])
    #         def store_as_manipulated(userdata):
    #             # manipulated_items.current += [self.current_item.current]
    #             item_list = manipulated_items.resolve()
    #             item_list += [self.current_item.resolve()]
    #             w = ds.VariableWriter(manipulated_items)
    #             w.write(item_list)
    #             return 'stored'
    #
    #         smach.StateMachine.add('STORE_ITEM',
    #                                smach.CBState(store_as_manipulated),
    #                                transitions={'stored':'LOOKAT_PLACE_SHELF'})
    #
    #         smach.StateMachine.add("LOOKAT_PLACE_SHELF",
    #                                  states.LookAtArea(robot, self.cabinet, area=PLACE_SHELF),
    #                                  transitions={  'succeeded'         :'PLACE_ITEM'})
    #
    #         smach.StateMachine.add( "PLACE_ITEM",
    #                                 Place(robot, self.current_item, self.place_position, self.arm_with_item_designator),
    #                                 transitions={   'done'              :'RESET_HEAD_PLACE',
    #                                                 'failed'            :'RESET_HEAD_HUMAN'})
    #
    #         smach.StateMachine.add( "RESET_HEAD_PLACE",
    #                                 states.CancelHead(robot),
    #                                 transitions={   'done'              :'UNLOCK_ITEM_AFTER_SUCCESSFUL_PLACE'})
    #
    #         smach.StateMachine.add( "RESET_HEAD_HUMAN",
    #                                 states.CancelHead(robot),
    #                                 transitions={   'done'               :'SAY_HANDOVER_TO_HUMAN'})
    #
    #         smach.StateMachine.add('UNLOCK_ITEM_AFTER_SUCCESSFUL_PLACE',
    #                                smach.CBState(unlock_and_ignore),
    #                                transitions={'unlocked'              :'succeeded'})
    #
    #         smach.StateMachine.add( "SAY_HANDOVER_TO_HUMAN",
    #                                 states.Say(robot, ["I'm can't get rid of this item  myself, can somebody help me maybe?"]),
    #                                 transitions={   'spoken'            :'HANDOVER_TO_HUMAN'})
    #
    #         smach.StateMachine.add('HANDOVER_TO_HUMAN',
    #                                states.HandoverToHuman(robot, self.arm_with_item_designator),
    #                                transitions={   'succeeded'         :'UNLOCK_AFTER_HANDOVER',
    #                                                 'failed'           :'UNLOCK_AFTER_HANDOVER'})
    #
    #         smach.StateMachine.add('UNLOCK_AFTER_HANDOVER',
    #                                smach.CBState(unlock_and_ignore),
    #                                transitions={'unlocked'              :'failed'})
