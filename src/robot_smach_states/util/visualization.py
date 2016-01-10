#! /usr/bin/env python
"""Plot a smach state machine"""

import os
import smach
from robot_smach_states.util.designators.core import Designator, VariableWriter, VariableDesignator
from graphviz import Digraph

def gv_safe(string):
    return str(string).replace("=", "_")

class StateViz(object):
    def __init__(self, smach_obj, parent):
        assert type(smach_obj) not in visualization_classes
        assert type(parent) in visualization_classes

        self.smach_obj = smach_obj
        self.parent = parent

    def add_to_graph(self, graph):
        graph.node(str(self.get_name()))

    def get_name(self):
        names = {v:k for k,v  in self.parent.smach_obj.get_children().iteritems()}
        name = names[self.smach_obj]
        return name

class TransitionViz(object):
    def __init__(self, from_, to, label):
        self.from_ = from_
        self.to = to
        self.label = label

    def add_to_graph(self, graph):
        graph.edge(str(self.from_.get_name()), str(self.to.get_name()), label=self.label)

class ContainerOutcomeViz(object):
    def __init__(self, name, parent):
        assert type(parent) in visualization_classes
        self.name = name
        self.parent = parent

    def add_to_graph(self, graph):
        graph.node(str(self.name))

    def get_name(self):
        return self.name

class StateMachineViz(StateViz):
    def __init__(self, smach_obj, parent):
        assert type(smach_obj) not in visualization_classes
        assert type(parent) in visualization_classes
        self.smach_obj = smach_obj
        self.parent = parent

    def add_to_graph(self, graph):
        machine = Digraph(self.get_name())

        for outcome in self.smach_obj._outcomes:
            outcomeviz = ContainerOutcomeViz(outcome, self)
            outcomeviz.add_to_graph(machine)

        for childname, child in self.smach_obj.get_children().iteritems():
            childviz = StateViz(child, self)
            for transition, to_name in self.smach_obj._transitions[childname].iteritems():
                if not to_name in self.smach_obj._outcomes:
                    # if to_name == "RANGE_ITERATOR": import ipdb; ipdb.set_trace()
                    to = self.smach_obj.get_children()[to_name]

                    if isinstance(to, smach.Iterator):
                        to_viz = IteratorViz(to, self)
                    elif isinstance(to, smach.StateMachine):
                        to_viz = StateMachineViz(to, self)
                    else:
                        to_viz = StateViz(to, self)
                else:
                    to_viz = ContainerOutcomeViz(to_name, self)

                to_viz.add_to_graph(machine)

                transitionviz = TransitionViz(childviz, to_viz, transition)
                transitionviz.add_to_graph(machine)
            childviz.add_to_graph(machine)

        machine.body.append('color=blue')
        graph.subgraph(machine)

    def get_name(self):
        try:
            names = {v:k for k,v  in self.parent.smach_obj.get_children().iteritems()}
            name = names[self.smach_obj]
            return name
        except AttributeError: #TODO: This happens when the parent is an Iterator, Instead a child inferring its name, instead its parent for its name
            return "CHILD"

class IteratorViz(StateViz):
    def __init__(self, smach_obj, parent):
        assert type(smach_obj) not in visualization_classes
        assert type(parent) in visualization_classes
        self.smach_obj = smach_obj
        self.parent = parent

    def add_to_graph(self, graph):
        machine = Digraph()
        childviz = StateMachineViz(self.smach_obj._state, self)
        # import ipdb; ipdb.set_trace()
        childviz.add_to_graph(machine)
        graph.subgraph(machine)

visualization_classes = [type(None), StateViz, StateMachineViz, TransitionViz, ContainerOutcomeViz, IteratorViz]

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
    viz  = StateMachineViz(statemachine, None)
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
