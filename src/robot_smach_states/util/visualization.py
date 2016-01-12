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
        graph.node(self.get_node_identifier(), label=self.get_name())

    def get_name(self):
        names = {v:k for k,v  in self.parent.smach_obj.get_children().iteritems()}
        name = names[self.smach_obj]
        return name

    def get_node_identifier(self):
        return "{}_{}".format(self.get_name(), gv_safe(self.smach_obj))

class TransitionViz(object):
    def __init__(self, from_, to, label):
        self.from_ = from_
        self.to = to
        self.label = label

    def add_to_graph(self, graph):

        to_identifier = self.to.get_node_identifier()
        if isinstance(self.to, StateMachineViz):
            to_identifier = self.to.make_childviz(self.to.smach_obj.get_children()[
                self.to.smach_obj._initial_state_label]).get_node_identifier()

        graph.edge(self.from_.get_node_identifier(),
                   to_identifier,
                   label=self.label)

class ContainerOutcomeViz(object):
    def __init__(self, name, parent):
        assert type(parent) in visualization_classes
        assert isinstance(name, str)
        self.name = name
        self.parent = parent

    def add_to_graph(self, graph):
        graph.node(self.get_node_identifier(), label=self.name)

    def get_name(self):
        return self.name

    def get_node_identifier(self):
        return "{}_{}".format(gv_safe(self.name), gv_safe(self.parent))

class ContainerViz(StateViz):
    def make_childviz(self, child):
        if isinstance(child, smach.Iterator):
            childviz = IteratorViz(child, self)
        elif isinstance(child, smach.StateMachine):
            childviz = StateMachineViz(child, self)
        else:
            childviz = StateViz(child, self)
        return childviz

class StateMachineViz(ContainerViz):
    def __init__(self, smach_obj, parent):
        assert type(smach_obj) not in visualization_classes
        assert type(parent) in visualization_classes
        self.smach_obj = smach_obj
        self.parent = parent

    def add_to_graph(self, graph):
        machine = Digraph(self.get_node_identifier())
        machine.body.append('label = "{}"'.format(self.get_name()))
        machine.body.append('color=blue')

        for outcome in self.smach_obj._outcomes:
            outcomeviz = ContainerOutcomeViz(outcome, self)
            outcomeviz.add_to_graph(machine)

        for childname, child in self.smach_obj.get_children().iteritems():
            childviz = self.make_childviz(child)

            for transition, to_name in self.smach_obj._transitions[childname].iteritems():
                if not to_name in self.smach_obj._outcomes:
                    # if to_name == "RANGE_ITERATOR": import ipdb; ipdb.set_trace()
                    if to_name == None:
                        print "ERROR: Transition {} of {} to None".format(transition, childname)
                        continue
                    to = self.smach_obj.get_children()[to_name]
                    to_viz = self.make_childviz(to)
                else:
                    to_viz = ContainerOutcomeViz(to_name, self)

                to_viz.add_to_graph(machine)

                transitionviz = TransitionViz(childviz, to_viz, transition)
                transitionviz.add_to_graph(machine)
            childviz.add_to_graph(machine)
        graph.subgraph(machine)

    def get_name(self):
        if self.parent:
            names = {v:k for k,v  in self.parent.smach_obj.get_children().iteritems()}
            name = names[self.smach_obj]
            return name
        else:
            return "CHILD"

class IteratorViz(ContainerViz):
    def __init__(self, smach_obj, parent):
        assert type(smach_obj) not in visualization_classes
        assert type(parent) in visualization_classes
        self.smach_obj = smach_obj
        self.parent = parent

    def add_to_graph(self, graph):
        machine = Digraph()
        machine.body.append('color=red')

        #import ipdb; ipdb.set_trace()
        for outcome in self.smach_obj._outcomes:
            outcomeviz = ContainerOutcomeViz(outcome, self)
            outcomeviz.add_to_graph(machine)

        for childname, child in self.smach_obj.get_children().iteritems():
            childviz = self.make_childviz(child)

            for transition, from_, to_name in self.smach_obj.get_internal_edges():
                if not to_name in self.smach_obj._outcomes:
                    to = self.smach_obj.get_children()[to_name]
                    to_viz = self.make_childviz(to)
                else:
                    to_viz = ContainerOutcomeViz(to_name, self)

                to_viz.add_to_graph(machine)

                transitionviz = TransitionViz(childviz, to_viz, transition)
                transitionviz.add_to_graph(machine)
            childviz.add_to_graph(machine)
        graph.subgraph(machine)

visualization_classes = [type(None), StateViz, StateMachineViz, TransitionViz, ContainerOutcomeViz, IteratorViz]

def visualize(statemachine, statemachine_name, save_dot=False, fmt='png'):
    dot = Digraph(comment=statemachine_name, format=fmt)
    
    dot.graph_attr['label'] = statemachine_name
    dot.graph_attr['labelloc'] ="t"

    #import ipdb; ipdb.set_trace()
    viz  = StateMachineViz(statemachine, None)
    viz.add_to_graph(dot)
    #_visualize_machine("ROOT", statemachine, dot)

    # dot.subgraph(make_legend())

    if save_dot:
        dot.save(statemachine_name + '_statemachine.dot')
    dot.render(statemachine_name + '_statemachine')

    os.remove(statemachine_name + '_statemachine')

def testcase1():
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

    visualize(sm, "testcase1")

def testcase2():
    import smach

    toplevel = smach.StateMachine(outcomes=['Done', 'Aborted'])
    with toplevel:
        @smach.cb_interface(outcomes=["succeeded", 'error'])
        def execute(userdata):
            return "succeeded"
        smach.StateMachine.add('TEST1',
                                smach.CBState(execute),
                                transitions={'succeeded':'SUBLEVEL1',
                                             'error'    :"Aborted"})

        sublevel1 = smach.StateMachine(outcomes=['Finished', 'Failed'])
        with sublevel1:
            smach.StateMachine.add('SUBTEST1',
                                    smach.CBState(execute),
                                    transitions={'succeeded':'SUBTEST2',
                                                 'error'    :'Failed'})
            smach.StateMachine.add('SUBTEST2',
                                    smach.CBState(execute),
                                    transitions={'succeeded':'Finished',
                                                 'error'    :'Failed'})

        smach.StateMachine.add('SUBLEVEL1',
                                sublevel1,
                                transitions={'Finished' :'Done',
                                             'Failed'   :'Aborted'})

    visualize(toplevel, "testcase2", save_dot=True)

if __name__ == "__main__":
    testcase1()
    testcase2()
