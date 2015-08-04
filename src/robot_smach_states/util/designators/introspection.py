#! /usr/bin/env python

import rospy
import smach
from robot_smach_states.util.designators.core import Designator
import gc
import pprint

__author__ = 'loy'

def analyse_designators():

    designators = [obj for obj in gc.get_objects() if isinstance(obj, Designator)]
    resolve_types = {desig:desig.resolve_type for desig in designators}

    states = [obj for obj in gc.get_objects() if isinstance(obj, smach.State)]
    label2state = smach.StateMachine.get_children(smach.StateMachine._currently_opened_container())
    #Do smach.StateMachine._currently_opened_container().get_children()["FIND_CROWD_CONTAINER"].get_children() recursively to get all labels but5 namespace them to their parent.
    state2label = {state:label for label,state in label2state.iteritems()}
    state_designator_relations = [] #List of (state, designator)-tuples
    
    # import ipdb; ipdb.set_trace()
    for state in states:
        for slot_key, slot_value in state.__dict__.iteritems():
            if slot_value in designators: #Dunno which is faster/simpler: I can also lookup which __slots__ are instance fo designator again
                name = state2label.get(state, state) #Get the abel of state, if not possible, just default to ugly __repr__
                state_designator_relations += [(name, slot_value, slot_key, slot_value.resolve_type)]

    pprint.pprint(resolve_types)
    print "-" * 10
    pprint.pprint(state_designator_relations)
    print "-" * 10
