#! /usr/bin/env python

import rospy
import smach
from robot_smach_states.util.designators.core import Designator
import gc
import pprint

"""
"""

__author__ = 'loy'

def flatten(tree, parentname=None, sep="."):
    flat = []
    for branchname, branch in tree.get_children().iteritems():
        if isinstance(branch, smach.StateMachine):
            flat.extend(flatten(branch, parentname=branchname, sep=sep))
        else:
            name = parentname+sep+branchname if parentname else branchname
            flat += [(name, branch)]
    # print flat
    return flat

def analyse_designators():

    designators = [obj for obj in gc.get_objects() if isinstance(obj, Designator)]
    resolve_types = {desig:desig.resolve_type for desig in designators}

    # states = [obj for obj in gc.get_objects() if isinstance(obj, smach.State)]
    # label2state = smach.StateMachine.get_children(smach.StateMachine._currently_opened_container())
    
    #Do smach.StateMachine._currently_opened_container().get_children()["FIND_CROWD_CONTAINER"].get_children() recursively to get all labels but5 namespace them to their parent
    label2state = dict(flatten(smach.StateMachine._currently_opened_container()))
    states = label2state.values()

    state2label = {state:label for label,state in label2state.iteritems()}
    state_designator_relations = []  # List of (state, designator, resolve_type)-tuples
    parent_child_relations = []  # List of (parent-designator, child-designator, role-of-child, resolve_type)-tuples
    
    from graphviz import Digraph
    dot = Digraph(comment='Designators')

    # import ipdb; ipdb.set_trace()
    for state in states:
        for designator_role, desig in state.__dict__.iteritems(): #Iterate the self.xxx members of each state
            # If the member is also a designator, then process it.
            if desig in designators: #Dunno which is faster/simpler: I can also lookup which of __dict__ are instance fo designator again
                statelabel = state2label.get(state, state) #Get the label of state, if not possible, just default to ugly __repr__
                state_designator_relations += [(statelabel, desig, designator_role, desig.resolve_type)]

                desig_name = format_designator(desig)

                dot.edge(   desig_name.replace("=", "_"), 
                            str(statelabel).replace("=", "_"), 
                            label=str(designator_role))

    for parent_designator in designators:
        for child_role, child_designator in parent_designator.__dict__.iteritems(): #Iterate the self.xxx members of each designator
            # If the member is also a designator, then process it.
            if child_designator in designators: #Dunno which is faster/simpler: I can also lookup which of __dict__ are instance fo designator again
                parent_child_relations += [(parent_designator, child_designator, child_role, child_designator.resolve_type)]

                parent_name = format_designator(parent_designator)
                child_name = format_designator(child_designator)

                dot.edge(   parent_name.replace("=", "_"),
                            child_name.replace("=", "_"),
                            label=str(child_role))

    pprint.pprint(resolve_types)
    print "-" * 10
    pprint.pprint(state_designator_relations)
    print "-" * 10

    dot.save('designators.dot')
    dot.render('designators.png')


def format_designator(desig):
    resolve_type_format = desig.resolve_type.__name__
    # import ipdb; ipdb.set_trace()
    # if (desig.resolve_type == list or desig.resolve_type == tuple) and len(desig.resolve_type) >= 1:
    #     # If the resolve_type is a collection, then show the type of the collection elements
    #     resolve_type_format = "[{}]".format(desig.resolve_type[0])

    desig_name = "{name}@{addr}\n[{resolve_type}]".format(name=desig.__class__.__name__,
                                                          addr=hex(id(desig)),
                                                          resolve_type=resolve_type_format)
    return desig_name