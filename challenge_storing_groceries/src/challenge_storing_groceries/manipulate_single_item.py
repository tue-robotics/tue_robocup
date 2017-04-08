import rospy
import smach
import robot_smach_states as states
from robot_smach_states import Grab
from robot_smach_states import Place
import robot_smach_states.util.designators as ds
from robot_skills.util import transformations

from empty_shelf_designator import EmptyShelfDesignator
from entity_description_designator import EntityDescriptionDesignator
from config import *
from remove_segmented_entities import RemoveSegmentedEntities
from segment_shelf import SegmentShelf


class ManipRecogSingleItem(smach.StateMachine):
    """The ManipRecogSingleItem state machine (for one object) is:
    - Stand of front of the bookcase
    - Look at the bookcase
    - Select an item, which is:
        - inside the bookcase
        - not yet grasped/not on the middle shelve
    - Grab that item
    - Say the class of the grabbed item
    - Place the item in an open spot on the middle shelve. """

    def __init__(self, robot, manipulated_items):
        """@param manipulated_items is VariableDesignator that will be a list of items manipulated by the robot."""
        self.manipulated_items = manipulated_items
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        self.cabinet = ds.EntityByIdDesignator(robot, id=CABINET, name="pick_shelf")
        # self.place_shelf = ds.EntityByIdDesignator(robot, id=PLACE_SHELF, name="place_shelf")

        not_manipulated = lambda entity: not entity in self.manipulated_items.resolve()

        def entity_z_pos(entity):
            """ Checks if the entity is between the minimum and maximum grasp height
            :param entity:
            :return:
            """
            if not entity.has_pose:
                return False
            return MIN_GRASP_HEIGHT < entity.pose.position.z < MAX_GRASP_HEIGHT

        # select the entity closest in x direction to the robot in base_link frame
        def weight_function(entity):
            # TODO: return x coordinate of entity.center_point in base_link frame
            p = transformations.tf_transform(entity.pose.position, "/map", robot.robot_name+"/base_link", robot.tf_listener)
            return p.x*p.x

        self.current_item = ds.LockingDesignator(ds.EdEntityDesignator(robot,
                                                                       criteriafuncs=[not_ignored, size,
                                                                                      not_manipulated,
                                                                                      min_entity_height, entity_z_pos,
                                                                                      max_width],
                                                                       weight_function=weight_function, debug=False,
                                                                       name="item"), name="current_item")

        #This makes that the empty spot is resolved only once, even when the robot moves. This is important because the sort is based on distance between robot and constrait-area
        # self.place_position = ds.LockingDesignator(ds.EmptySpotDesignator(robot, self.cabinet, name="placement", area=PLACE_SHELF), name="place_position")
        self.place_position = ds.LockingDesignator(EmptyShelfDesignator(robot, self.cabinet,
                                                                        name="placement", area=PLACE_SHELF),
                                                   name="place_position")

        if PREFERRED_ARM == "left":
            prefered_arm = robot.leftArm
        elif PREFERRED_ARM == "right":
            prefered_arm = robot.rightArm
        else:
            rospy.logwarn("Impossible preferred arm: {0}, defaulting to left".format(PREFERRED_ARM))
            prefered_arm = robot.leftArm

        self.empty_arm_designator = ds.UnoccupiedArmDesignator(robot.arms, prefered_arm, name="empty_arm_designator")
        self.arm_with_item_designator = ds.ArmHoldingEntityDesignator(robot.arms, self.current_item, name="arm_with_item_designator")

        # print "{0} = pick_shelf".format(self.pick_shelf)
        # print "{0} = current_item".format(self.current_item)
        # print "{0} = place_position".format(self.place_position)
        # print "{0} = empty_arm_designator".format(self.empty_arm_designator)
        # print "{0} = arm_with_item_designator".format(self.arm_with_item_designator)

        with self:
            # smach.StateMachine.add( "NAV_TO_OBSERVE_PICK_SHELF",
            #                         #states.NavigateToObserve(robot, self.pick_shelf),
            #                         states.NavigateToSymbolic(robot, {self.pick_shelf:"in_front_of", EntityByIdDesignator(robot, id=ROOM):"in"}, self.pick_shelf),
            #                         transitions={   'arrived'           :'LOOKAT_PICK_SHELF',
            #                                         'unreachable'       :'LOOKAT_PICK_SHELF',
            #                                         'goal_not_defined'  :'LOOKAT_PICK_SHELF'})

            smach.StateMachine.add("REMOVE_ENTITIES",
                                   RemoveSegmentedEntities(robot=robot),
                                   transitions={'done': 'LOOKAT_PICK_SHELF'})

            smach.StateMachine.add("LOOKAT_PICK_SHELF",
                                     states.LookAtArea(robot, self.cabinet, area=PICK_SHELF),
                                     transitions={  'succeeded'         :'SEGMENT_SHELF'})

            smach.StateMachine.add("SEGMENT_SHELF",
                                   SegmentShelf(robot, entity_id=CABINET, area_id=PICK_SHELF),
                                   transitions={'done': 'LOCK_ITEM'})

            @smach.cb_interface(outcomes=['locked'])
            def lock(userdata):
                self.current_item.lock() #This determines that self.current_item cannot not resolve to a new value until it is unlocked again.
                if self.current_item.resolve():
                    rospy.loginfo("Current_item is now locked to {0}".format(self.current_item.resolve().id))

                self.place_position.lock() #This determines that self.place_position will lock/cache its result after its resolved the first time.
                return 'locked'
            smach.StateMachine.add('LOCK_ITEM',
                                   smach.CBState(lock),
                                   transitions={'locked':'ANNOUNCE_ITEM'})

            smach.StateMachine.add( "ANNOUNCE_ITEM",
                                    states.Say(robot, EntityDescriptionDesignator(self.current_item,
                                                                                  name="current_item_desc"),
                                               block=False),
                                    transitions={   'spoken'            :'GRAB_ITEM'})

            smach.StateMachine.add( "GRAB_ITEM",
                                    Grab(robot, self.current_item, self.empty_arm_designator),
                                    transitions={   'done'              :'STORE_ITEM',
                                                    'failed'            :'SAY_GRAB_FAILED'})

            smach.StateMachine.add( "SAY_GRAB_FAILED",
                                    states.Say(robot, ["I couldn't grab this thing"], mood="sad"),
                                    transitions={   'spoken'            :'UNLOCK_ITEM_AFTER_FAILED_GRAB'}) # Not sure whether to fail or keep looping with NAV_TO_OBSERVE_PICK_SHELF

            @smach.cb_interface(outcomes=['unlocked'])
            def unlock_and_ignore(userdata):
                global ignore_ids
                # import ipdb; ipdb.set_trace()
                if self.current_item.resolve():
                    ignore_ids += [self.current_item.resolve().id]
                    rospy.loginfo("Current_item WAS now locked to {0}".format(self.current_item.resolve().id))
                self.current_item.unlock() #This determines that self.current_item can now resolve to a new value on the next call
                self.place_position.unlock() #This determines that self.place_position can now resolve to a new position on the next call
                return 'unlocked'
            smach.StateMachine.add('UNLOCK_ITEM_AFTER_FAILED_GRAB',
                                   smach.CBState(unlock_and_ignore),
                                   transitions={'unlocked'              :'failed'})

            @smach.cb_interface(outcomes=['stored'])
            def store_as_manipulated(userdata):
                # manipulated_items.current += [self.current_item.current]
                item_list = manipulated_items.resolve()
                item_list += [self.current_item.resolve()]
                w = ds.VariableWriter(manipulated_items)
                w.write(item_list)
                return 'stored'

            smach.StateMachine.add('STORE_ITEM',
                                   smach.CBState(store_as_manipulated),
                                   transitions={'stored':'LOOKAT_PLACE_SHELF'})

            smach.StateMachine.add("LOOKAT_PLACE_SHELF",
                                     states.LookAtArea(robot, self.cabinet, area=PLACE_SHELF),
                                     transitions={  'succeeded'         :'PLACE_ITEM'})

            smach.StateMachine.add( "PLACE_ITEM",
                                    Place(robot, self.current_item, self.place_position, self.arm_with_item_designator),
                                    transitions={   'done'              :'RESET_HEAD_PLACE',
                                                    'failed'            :'RESET_HEAD_HUMAN'})

            smach.StateMachine.add( "RESET_HEAD_PLACE",
                                    states.CancelHead(robot),
                                    transitions={   'done'              :'UNLOCK_ITEM_AFTER_SUCCESSFUL_PLACE'})

            smach.StateMachine.add( "RESET_HEAD_HUMAN",
                                    states.CancelHead(robot),
                                    transitions={   'done'               :'SAY_HANDOVER_TO_HUMAN'})

            smach.StateMachine.add('UNLOCK_ITEM_AFTER_SUCCESSFUL_PLACE',
                                   smach.CBState(unlock_and_ignore),
                                   transitions={'unlocked'              :'succeeded'})

            smach.StateMachine.add( "SAY_HANDOVER_TO_HUMAN",
                                    states.Say(robot, ["I'm can't get rid of this item  myself, can somebody help me maybe?"]),
                                    transitions={   'spoken'            :'HANDOVER_TO_HUMAN'})

            smach.StateMachine.add('HANDOVER_TO_HUMAN',
                                   states.HandoverToHuman(robot, self.arm_with_item_designator),
                                   transitions={   'succeeded'         :'UNLOCK_AFTER_HANDOVER',
                                                    'failed'           :'UNLOCK_AFTER_HANDOVER'})

            smach.StateMachine.add('UNLOCK_AFTER_HANDOVER',
                                   smach.CBState(unlock_and_ignore),
                                   transitions={'unlocked'              :'failed'})
