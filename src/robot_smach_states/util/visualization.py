"""Plot a smach state machine"""

import os
import smach
from robot_smach_states.util.designators.core import Designator, VariableWriter, VariableDesignator
from graphviz import Digraph

class State(object):
    def __init__(self, state_obj):
        self.state_obj

    def add_graphviz_edge(self, graph):
        graph.node(self.state_obj)

class StateMachine(State):
    def __init__(self, sm_obj):
        self.sm_obj = sm_obj

    def add_graphviz_edge(self, graph):
        machine = Digraph(self.name)
        for child in self.children:
            child.add_graphviz_edge(machine)

        graph.subgraph(machine)

def flatten(tree, parentname=None, sep="."):
    flat = []
    for branch_name, branch in tree.get_children().iteritems():
        this_name = parentname + sep + branch_name if parentname else branch_name
        if isinstance(branch, smach.StateMachine) or isinstance(branch, smach.Iterator):
            flat += [(this_name, branch)]
            flat.extend(flatten(branch, parentname=this_name, sep=sep))
        else:
            flat += [(this_name, branch)]
    return flat

def _visualize_state(name, state, graph):
    graph.node(name)

def _visualize_machine(name, machine, graph):
    subgraph = Digraph(name)

    for branch_name, branch in machine.get_children().iteritems():
        #TODO: Add transitions
        if isinstance(branch, smach.StateMachine) or isinstance(branch, smach.Iterator):
            _visualize_machine(branch_name, branch, graph)
        else:
            _visualize_state(branch_name, branch, graph)

    graph.subgraph(subgraph)


def visualize(statemachine, statemachine_name, , save_dot=False, fmt='png'):
    dot = Digraph(comment=statemachine_name, format=fmt)
    
    dot.graph_attr['label'] = statemachine_name
    dot.graph_attr['labelloc'] ="t"

    import ipdb; ipdb.set_trace()
    _visualize_machine("ROOT", statemachine, dot)

    # dot.subgraph(make_legend())

    if save_dot:
        dot.save(statemachine_name + '_statemachine.dot')
    dot.render(statemachine_name + '_statemachine')

    os.remove(statemachine_name + '_statemachine')
