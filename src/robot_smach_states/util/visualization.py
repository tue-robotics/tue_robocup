#! /usr/bin/env python
"""Plot a smach state machine"""

import os
import smach
from robot_smach_states.util.designators.core import Designator, VariableWriter, VariableDesignator
from graphviz import Digraph

def gv_safe(string):
    return str(string).replace("=", "_")

class State(object):
    def __init__(self, state_obj, parent):
        self.state_obj = state_obj
        self.parent = parent

    def add_to_graph(self, graph):
        graph.node(str(self.get_name()))

    def get_name(self):
        names = {v:k for k,v  in self.parent.get_children().iteritems()}
        name = names[self.state_obj]
        return name

class Transition(object):
    def __init__(self, from_, to, label):
        self.from_ = from_
        self.to = to
        self.label = label

    def add_to_graph(self, graph):
        graph.edge(str(self.from_.get_name()), str(self.to.get_name()), label=self.label)

class ContainerOutcome(object):
    def __init__(self, name, parent):
        self.name = name
        self.parent = parent

    def add_to_graph(self, graph):
        graph.node(str(self.name))

    def get_name(self):
        return self.name

class StateMachine(State):
    def __init__(self, sm_obj, parent):
        self.sm_obj = sm_obj
        self.parent = parent

    def add_to_graph(self, graph):
        subgraphname = "ROOT"
        if self.parent:
            names = {v:k for k,v  in self.parent.get_children().iteritems()}
            subgraphname = names[self.sm_obj]
        machine = Digraph(subgraphname)

        for outcome in self.sm_obj._outcomes:
            outcomeviz = ContainerOutcome(outcome, self)
            outcomeviz.add_to_graph(machine)

        for childname, child in self.sm_obj.get_children().iteritems():
            childviz = State(child, self.sm_obj)
            for transition, to_name in self.sm_obj._transitions[childname].iteritems():
                if not to_name in self.sm_obj._outcomes:
                    to = self.sm_obj.get_children()[to_name]
                    to_viz = State(to, self.sm_obj)
                else:
                    to_viz = ContainerOutcome(to_name, self)

                transitionviz = Transition(childviz, to_viz, transition)
                transitionviz.add_to_graph(machine)
            childviz.add_to_graph(machine)

        machine.body.append('color=blue')
        graph.subgraph(machine)

    def get_name(self):
        names = {v:k for k,v  in self.parent.get_children().iteritems()}
        name = names[self.sm_obj]
        return name

class Iterator(State):
    def __init__(self, iterator_obj, parent):
        self.iterator_obj = iterator_obj
        self.parent = parent

    def add_to_graph(self, graph):
        machine = Digraph()
        machine.node(gv_safe(self.iterator_obj))
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
    graph.node(gv_safe(state), label=name)

def _visualize_iterator(name, iterator, graph):
    _visualize_machine(name, iterator._state, graph)

def _visualize_machine(name, machine, graph):
    subgraph = Digraph(name)

    for branch_name, branch in machine.get_children().iteritems():
        for outcome,next_ in machine._transitions[branch_name].iteritems():
            if not next_ in machine._outcomes:
                next_ = gv_safe(machine.get_children().get(next_, "None"))

            subgraph.edge(branch_name, next_, label=outcome if outcome else "None")

        if isinstance(branch, smach.Iterator):
            _visualize_iterator(branch_name, branch, graph)
        elif isinstance(branch, smach.StateMachine):
            _visualize_machine(branch_name, branch, graph)
        else:
            _visualize_state(branch_name, branch, graph)

    subgraph.body.append('color=blue')
    graph.subgraph(subgraph)


def visualize(statemachine, statemachine_name, save_dot=False, fmt='png'):
    dot = Digraph(comment=statemachine_name, format=fmt)
    
    dot.graph_attr['label'] = statemachine_name
    dot.graph_attr['labelloc'] ="t"

    import ipdb; ipdb.set_trace()
    viz  = StateMachine(statemachine, None)
    viz.add_to_graph(dot)
    #_visualize_machine("ROOT", statemachine, dot)

    # dot.subgraph(make_legend())

    if save_dot:
        dot.save(statemachine_name + '_statemachine.dot')
    dot.render(statemachine_name + '_statemachine')

    os.remove(statemachine_name + '_statemachine')


if __name__ == "__main__":
    import smach

    sm = smach.StateMachine(outcomes=['Done', 'Aborted'])
    with sm:
        @smach.cb_interface(outcomes=["succeeded"])
        def execute(userdata):
            return "succeeded"
        smach.StateMachine.add('TEST1',
                                smach.CBState(execute),
                                transitions={'succeeded':'TEST2'})
        smach.StateMachine.add('TEST2',
                                smach.CBState(execute),
                                transitions={'succeeded':'Done'})

    visualize(sm, "test")
