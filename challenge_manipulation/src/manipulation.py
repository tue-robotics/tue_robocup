#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/Manipulation.tex

In short, the robot starts at 1-1.5m from a bookcase and must wait until started by an operator (by voice or a start button)

This bookcase has a couple of shelves on which some items are placed. 
**The middle shelve starts empty**, this is where the objects need to be placed. 

The robot must take objects form the shelves and place them on the middle shelve and indicate the class of each grasped object. 

After the robot is started by voice or a button, 
    the ManipRecogSingleItem state machine is repeated at least 5 times (for 5 objects). 
Afterwards, a PDF report has to be made:
'After the test is completed or the time has run out, 
    the robot may upload a single PDF report file including the list of recognized objects with a picture showing:
    - the object, 
    - the object name,
    - the bounding box of the object.'
"""

import rospy
import smach
import sys
import random
import math

from robot_smach_states.util.designators import *
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_smach_states import Place
from robot_smach_states.util.geometry_helpers import *
from robot_skills.util import msg_constructors as geom
from robot_skills.util import transformations
import geometry_msgs.msg as gm
from robot_skills.util import transformations 


import pdf

ignore_ids = ['robotics_testlabs']
ignore_types = ['waypoint', 'floor','room']
BOOKCASE = "hallway_couch"
ROOM = "room_hallway"
PLACE_HEIGHT = 1.0


class FormattedSentenceDesignator(Designator):
    """docstring for FormattedSentenceDesignator"""
    def __init__(self, fmt, **kwargs):
        super(FormattedSentenceDesignator, self).__init__(resolve_type=str)
        self.fmt = fmt
        self.kwargs = kwargs
    
    def resolve(self):
        kwargs_resolved = {key:value.resolve() for key,value in self.kwargs.iteritems()}
        return self.fmt.format(**kwargs_resolved)


class EntityDescriptionDesignator(Designator):
    """EntityDescriptionDesignator"""
    def __init__(self, formats, entity_designator):
        super(EntityDescriptionDesignator, self).__init__(resolve_type=str)
        self.entity_designator = entity_designator
        self.formats = formats
    
    def resolve(self):
        entity = self.entity_designator.resolve()
        if not entity:
            return None
        short_id = entity.id[:5]
        typ = entity.type
        fmt = self.formats
        if not isinstance(fmt, str) and isinstance(fmt, list):
            fmt = random.choice(fmt)
        sentence = fmt.format(type=typ, id=short_id)
        return sentence


class EmptySpotDesignator(Designator):
    """Designates an empty spot on the empty placement-shelve. 
    It does this by queying ED for entities that occupy some space. 
        If the result is no entities, then we found an open spot."""
    def __init__(self, robot, closet_designator):
        super(EmptySpotDesignator, self).__init__(resolve_type=gm.PoseStamped)
        self.robot = robot
        self.closet_designator = closet_designator
        self._edge_distance = 0.1                   # Distance to table edge
        self._spacing = 0.15

    def resolve(self):
        closet = self.closet_designator.resolve()

        # points_of_interest = []
        points_of_interest = self.determinePointsOfInterest(closet)

        def is_poi_occupied(poi):
            entities_at_poi = self.robot.ed.get_entities(center_point=poi, radius=self._spacing)
            return not any(entities_at_poi)

        open_POIs = filter(is_poi_occupied, points_of_interest)

        # ToDo: best POI, e.g., based on distance???

        if any(open_POIs):
            placement = geom.PoseStamped(pointstamped=open_POIs[0])
            rospy.loginfo("Placement = {0}".format(placement).replace('\n', ' '))
            return placement
        else:
            rospy.logerr("Could not find an empty spot")
            return None

    def determinePointsOfInterest(self, e):

        points = []

        ch = e.convex_hull
        x = e.pose.position.x
        y = e.pose.position.y

        if len(ch) == 0:
            return []

        ''' Loop over hulls '''
        ch.append(ch[0])
        for i in xrange(len(ch) - 1):
                dx = ch[i+1].x - ch[i].x
                dy = ch[i+1].y - ch[i].y
                length = math.hypot(dx, dy)

                d = self._edge_distance
                while d < (length-self._edge_distance):

                    ''' Point on edge '''
                    xs = ch[i].x + d/length*dx
                    ys = ch[i].y + d/length*dy

                    ''' Shift point inwards and fill message'''
                    ps = geom.PointStamped()
                    ps.header.frame_id = "/map"
                    ps.point.x = xs - dy/length * self._edge_distance
                    ps.point.y = ys + dx/length * self._edge_distance
                    ps.point.z = e.z_max
                    points.append(ps)

                    # ToDo: check if still within hull???
                    d += self._spacing

        return points


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
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        
        bookcase = EdEntityDesignator(robot, id=BOOKCASE)

        # TODO: Designate items that are
        # inside bookcase 
        # and are _not_:
        #   already placed 
        #   on the placement-shelve.
        not_ignored = lambda entity: not entity.type in ignore_types and not entity.id in ignore_ids
        size = lambda entity: abs(entity.z_max - entity.z_min) < 0.2
        not_manipulated = lambda entity: not entity in manipulated_items.resolve()
        has_type = lambda entity: entity.type != ""
        min_height = lambda entity: entity.min_z > 0.3
        def on_top(entity):
            container_entity = bookcase.resolve()
            return onTopOff(entity, container_entity)
        
        # select the entity closest in x direction to the robot in base_link frame            
        def weight_function(entity):
            # TODO: return x coordinate of entity.center_point in base_link frame
            p = transformations.tf_transform(entity.center_point, robot.robot_name+"/base_link", "/map", robot.tf_listener) 
            return p.x*p.x

        # current_item = EdEntityDesignator(robot, id="beer1")  # TODO: For testing only
        # current_item = LockingDesignator(EdEntityDesignator(robot, 
        #     center_point=geom.PointStamped(frame_id="/"+BOOKCASE), radius=2.0,
        #     criteriafuncs=[not_ignored, size, not_manipulated, has_type, on_top], debug=False))
        current_item = LockingDesignator(EdEntityDesignator(robot, 
            criteriafuncs=[not_ignored, size, not_manipulated, has_type, on_top], weight_function=weight_function, debug=False))
        
        place_position = EmptySpotDesignator(robot, bookcase) 
        
        empty_arm_designator = UnoccupiedArmDesignator(robot.arms, robot.leftArm)
        arm_with_item_designator = ArmHoldingEntityDesignator(robot.arms, current_item)

        print "{0} = bookcase".format(bookcase)
        print "{0} = current_item".format(current_item)
        print "{0} = place_position".format(place_position)
        print "{0} = empty_arm_designator".format(empty_arm_designator)
        print "{0} = arm_with_item_designator".format(arm_with_item_designator)

        with self:
            smach.StateMachine.add( "NAV_TO_OBSERVE_BOOKCASE",
                                    #states.NavigateToObserve(robot, bookcase),
                                    states.NavigateToSymbolic(robot, {bookcase:"in_front_of", EdEntityDesignator(robot, id=ROOM):"in"}, bookcase),
                                    transitions={   'arrived'           :'LOOKAT_BOOKCASE',
                                                    'unreachable'       :'LOOKAT_BOOKCASE',
                                                    'goal_not_defined'  :'LOOKAT_BOOKCASE'})

            smach.StateMachine.add( "LOOKAT_BOOKCASE",
                                    states.Say(robot, ["I'm looking at the bookcase to see what items I can find"]),
                                    transitions={   'spoken'            :'LOCK_ITEM'})

            @smach.cb_interface(outcomes=['locked'])
            def lock(userdata):
                current_item.lock() #This determines that current_item cannot not resolve to a new value until it is unlocked again.
                if current_item.resolve():
                    rospy.loginfo("Current_item is now locked to {0}".format(current_item.resolve().id))
                return 'locked'
            smach.StateMachine.add('LOCK_ITEM',
                                   smach.CBState(lock),
                                   transitions={'locked':'ANNOUNCE_ITEM'})

            smach.StateMachine.add( "ANNOUNCE_ITEM",
                                    states.Say(robot, EntityDescriptionDesignator("I'm trying to grab item {id} which is a {type}.", current_item), block=False),
                                    transitions={   'spoken'            :'GRAB_ITEM'})

            smach.StateMachine.add( "GRAB_ITEM",
                                    Grab(robot, current_item, empty_arm_designator),
                                    transitions={   'done'              :'STORE_ITEM',
                                                    'failed'            :'SAY_GRAB_FAILED'})

            smach.StateMachine.add( "SAY_GRAB_FAILED",
                                    states.Say(robot, ["I couldn't grab this thing"], mood="sad"),
                                    transitions={   'spoken'            :'UNLOCK_ITEM_AFTER_FAILED_GRAB'}) # Not sure whether to fail or keep looping with NAV_TO_OBSERVE_BOOKCASE

            @smach.cb_interface(outcomes=['unlocked'])
            def unlock_and_ignore(userdata):
                global ignore_ids
                # import ipdb; ipdb.set_trace()
                if current_item.resolve():
                    ignore_ids += [current_item.resolve().id]
                    rospy.loginfo("Current_item WAS now locked to {0}".format(current_item.resolve().id))
                current_item.unlock() #This determines that current_item can now resolve to a new value on the next call 
                return 'unlocked'
            smach.StateMachine.add('UNLOCK_ITEM_AFTER_FAILED_GRAB',
                                   smach.CBState(unlock_and_ignore),
                                   transitions={'unlocked'              :'failed'})

            @smach.cb_interface(outcomes=['stored'])
            def store_as_manipulated(userdata):
                manipulated_items.current += [current_item.current]
                return 'stored'

            smach.StateMachine.add('STORE_ITEM',
                                   smach.CBState(store_as_manipulated),
                                   transitions={'stored':'ANNOUNCE_CLASS'})

            smach.StateMachine.add( "ANNOUNCE_CLASS",
                                    states.Say(robot, FormattedSentenceDesignator("This is a {item.type}.", item=current_item), block=False),
                                    transitions={   'spoken'            :'PLACE_ITEM'})

            smach.StateMachine.add( "PLACE_ITEM",
                                    Place(robot, current_item, place_position, arm_with_item_designator),
                                    transitions={   'done'              :'UNLOCK_ITEM_AFTER_SUCCESSFUL_PLACE',
                                                    'failed'            :'SAY_HANDOVER_TO_HUMAN'})

            smach.StateMachine.add('UNLOCK_ITEM_AFTER_SUCCESSFUL_PLACE',
                                   smach.CBState(unlock_and_ignore),
                                   transitions={'unlocked'              :'succeeded'})

            smach.StateMachine.add( "SAY_HANDOVER_TO_HUMAN",
                                    states.Say(robot, ["I'm can't get rid of this item  myself, can somebody help me maybe?"]),
                                    transitions={   'spoken'            :'HANDOVER_TO_HUMAN'})

            smach.StateMachine.add('HANDOVER_TO_HUMAN',
                                   states.HandoverToHuman(robot, arm_with_item_designator),
                                   transitions={   'succeeded'         :'UNLOCK_AFTER_HANDOVER',
                                                    'failed'           :'UNLOCK_AFTER_HANDOVER'})

            smach.StateMachine.add('UNLOCK_AFTER_HANDOVER',
                                   smach.CBState(unlock_and_ignore),
                                   transitions={'unlocked'              :'failed'})


def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    start_waypoint = EdEntityDesignator(robot, id="living_room")  # TODO: select proper waypoint
    placed_items = []

    with sm:
        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'AWAIT_START',
                                                'abort':'Aborted'})

        smach.StateMachine.add("AWAIT_START",
                               states.AskContinue(robot),
                               transitions={'continue'                  :'NAV_TO_START',
                                            'no_response'               :'AWAIT_START'})

        smach.StateMachine.add("NAV_TO_START",
                                states.NavigateToWaypoint(robot, start_waypoint),
                                transitions={'arrived'                  :'RANGE_ITERATOR',
                                             'unreachable'              :'RANGE_ITERATOR',
                                             'goal_not_defined'         :'RANGE_ITERATOR'})

        # Begin setup iterator
        range_iterator = smach.Iterator(    outcomes = ['succeeded','failed'], #Outcomes of the iterator state
                                            input_keys=[], output_keys=[],
                                            it = lambda: range(5),
                                            it_label = 'index',
                                            exhausted_outcome = 'succeeded') #The exhausted argument should be set to the preffered state machine outcome

        with range_iterator:
            single_item = ManipRecogSingleItem(robot, VariableDesignator(placed_items, list))

            smach.Iterator.set_contained_state( 'SINGLE_ITEM', 
                                                single_item, 
                                                loop_outcomes=['succeeded','failed'])

        smach.StateMachine.add('RANGE_ITERATOR', range_iterator,
                        {   'succeeded'                                     :'EXPORT_PDF',
                            'failed'                                        :'Aborted'})
        # End setup iterator

        @smach.cb_interface(outcomes=["exported"])
        def export_to_pdf(userdata):
            rospy.loginfo("Placed_items: {0}".format([e.id for e in placed_items]))
            pdf.items2markdown(robot, placed_items)
            return "exported"
        smach.StateMachine.add('EXPORT_PDF',
                                smach.CBState(export_to_pdf),
                                transitions={'exported':'AT_END'})

        smach.StateMachine.add('AT_END',
                               states.Say(robot, "Goodbye"),
                               transitions={'spoken': 'Done'})
    return sm


############################## initializing program ######################
if __name__ == '__main__':
    rospy.init_node('manipulation_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE MANIPULATION] Please provide robot name as argument."
        exit(1)

    startup(setup_statemachine, robot_name=robot_name)
