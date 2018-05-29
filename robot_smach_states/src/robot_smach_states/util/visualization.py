#! /usr/bin/env python
"""Plot a smach state machine"""

# System
from functools import wraps
from graphviz import Digraph
import os

# ROS
import smach


def gv_safe(string):
    return str(string).replace("=", "_").replace(" ", "_").replace(".", "_").replace(">", "").replace("<", "")


def make_calls_unique(f):
    @wraps(f)
    def wrapper(*args, **kwargs):
        pass


class StateViz(object):
    def __init__(self, smach_obj, parent):
        assert type(smach_obj) not in visualization_classes
        assert type(parent) in visualization_classes

        self.smach_obj = smach_obj
        self.parent = parent

    def add_to_graph(self, graph, level=0):
        graph.node(self.get_node_identifier(), label=self.get_name())

    def get_name(self):
        names = {v:k for k,v  in self.parent.smach_obj.get_children().iteritems()}
        name = names[self.smach_obj]
        return name

    def get_node_identifier(self):
        return str(id(self.smach_obj)) #"{}_{}".format(self.get_name(), gv_safe(self.smach_obj))

    def __eq__(self, other):
        return self.smach_obj == other.smach_obj

    def __ne__(self, other):
        return not self == other

    def __hash__(self):
        return hash(self.smach_obj) + hash(self.parent)

class TransitionViz(object):
    def __init__(self, from_, to, label):
        self.from_ = from_
        self.to = to
        self.label = label

    def add_to_graph(self, graph, level=0):

        to_identifier = self.to.get_node_identifier()

        #Link external transition to state machine to state machine's internal first state
        if isinstance(self.to, StateMachineViz):
            to_identifier = self.to.make_childviz(self.to.smach_obj.get_children()[
                self.to.smach_obj._initial_state_label]).get_node_identifier()

        if isinstance(self.from_, StateMachineViz):
            #If we're coming from a state machine, don't draw from the state machine directly but from the internal Container outcome that is linked to this external
            outcome = ContainerOutcomeViz(self.label, self.from_)
            graph.edge(outcome.get_node_identifier(), self.to.get_node_identifier())
            return

        # print "TransitionViz: graph.edge({},{}, label={})".format(self.from_.get_node_identifier(), to_identifier, self.label)
        graph.node(self.from_.get_node_identifier(), label=self.from_.get_name())
        graph.node(self.to.get_node_identifier(), label=self.to.get_name())

        graph.edge(self.from_.get_node_identifier(),
                   to_identifier,
                   label=self.label)

    def __eq__(self, other):
        return self.from_ == other.from_ and self.to == other.to

    def __ne__(self, other):
        return not self == other

    def __hash__(self):
        return hash(self.from_) + hash(self.to)

class ContainerOutcomeViz(object):
    def __init__(self, name, parent):
        assert type(parent) in visualization_classes
        assert isinstance(name, str)
        self.name = name
        self.parent = parent

    def add_to_graph(self, graph, level=0):
        graph.node(self.get_node_identifier(), label=self.name, shape='square')

    def get_name(self):
        return self.name

    def get_node_identifier(self):
        return "{}_{}".format(gv_safe(self.name), gv_safe(self.parent.get_node_identifier()))

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
    def __init__(self, smach_obj, parent, name="CHILD"):
        assert type(smach_obj) not in visualization_classes
        assert type(parent) in visualization_classes
        self.smach_obj = smach_obj
        self.parent = parent
        self.name = name

    def add_to_graph(self, graph, level=0):
        my_subgraph = Digraph(self.get_name())
        print level*'\t' + "StateMachineViz.add_to_graph({}) adding my_subgraph {} ".format(id(graph), id(my_subgraph))

        # for outcome in self.smach_obj._outcomes:
        #     outcomeviz = ContainerOutcomeViz(outcome, self)
        #     outcomeviz.add_to_graph(machine)

        # for childname, child in self.smach_obj.get_children().iteritems():
        #     childviz = self.make_childviz(child)

        print level*'\t' + "self.smach_obj._transitions = {}".format(self.smach_obj._transitions)
        element_viz_set = set()
        for childname in self.smach_obj._transitions.keys():
            print level*'\t' + "child {}".format(childname)
            child = self.smach_obj.get_children()[childname]
            childviz = self.make_childviz(child)
            # childviz.add_to_graph(my_subgraph, level=level+1)
            element_viz_set.add(childviz)

            for transition, to_name in self.smach_obj._transitions[childname].iteritems():
                print level*'\t' + "transition {} --{}--> {}".format(childname, transition, to_name)
                if not to_name in self.smach_obj._outcomes:
                    # if to_name == "RANGE_ITERATOR": import ipdb; ipdb.set_trace()
                    if to_name == None:
                        print level*'\t' + "ERROR: Transition {} of {} to None".format(transition, childname)
                        continue
                    to = self.smach_obj.get_children()[to_name]
                    to_viz = self.make_childviz(to)
                else:
                    to_viz = ContainerOutcomeViz(to_name, self)

                element_viz_set.add(to_viz)

                # print "TransitionViz({}, {}, {})".format(childviz, to_viz, transition)
                transitionviz = TransitionViz(childviz, to_viz, transition)
                element_viz_set.add(transitionviz)

        for elem_viz in element_viz_set:
            if isinstance(elem_viz, TransitionViz):
                print "Adding transition: {} --> {}".format(elem_viz.to.get_node_identifier(), elem_viz.from_.get_node_identifier())
            elem_viz.add_to_graph(my_subgraph, level=level+1)

        my_subgraph.body.append('label = "{}"'.format(self.get_name()))
        my_subgraph.body.append('color=blue')


        graph.subgraph(my_subgraph)


    def get_name(self):
        if self.parent:
            names = {v:k for k,v  in self.parent.smach_obj.get_children().iteritems()}
            name = names[self.smach_obj]
            return name
        else:
            return self.name

class IteratorViz(ContainerViz):
    def __init__(self, smach_obj, parent):
        assert type(smach_obj) not in visualization_classes
        assert type(parent) in visualization_classes
        self.smach_obj = smach_obj
        self.parent = parent

    def add_to_graph(self, graph, level=0):
        machine = Digraph()

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

        machine.body.append('color=red')
        graph.subgraph(machine)

visualization_classes = [type(None), StateViz, StateMachineViz, TransitionViz, ContainerOutcomeViz, IteratorViz]

def visualize(statemachine, statemachine_name, save_dot=False, fmt='png'):
    dot = Digraph(statemachine_name, comment=statemachine_name, format=fmt)

    dot.graph_attr['label'] = statemachine_name
    dot.graph_attr['labelloc'] ="t"

    viz  = StateMachineViz(statemachine, None, statemachine_name)
    # import ipdb; ipdb.set_trace()
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
        def execute(userdata=None):
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
        def execute(userdata=None):
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

def testcase3():
    import smach

    testcase3 = smach.StateMachine(outcomes=['Done'])
    with testcase3:
        @smach.cb_interface(outcomes=["succeeded"])
        def execute(userdata=None):
            return "succeeded"
        smach.StateMachine.add('TEST1',
                                smach.CBState(execute),
                                transitions={'succeeded':'SUBLEVEL1'})




        sublevel1 = smach.StateMachine(outcomes=['Finished'])
        with sublevel1:
            smach.StateMachine.add('SUBTEST1',
                                    smach.CBState(execute),
                                    transitions={'succeeded':'SUBTEST2'})
            smach.StateMachine.add('SUBTEST2',
                                    smach.CBState(execute),
                                    transitions={'succeeded':'Finished'})

        smach.StateMachine.add('SUBLEVEL1',
                                sublevel1,
                                transitions={'Finished' :'Done'})
    visualize(testcase3, "testcase3", save_dot=True)

def testcase4():
    import smach

    testcase4 = smach.StateMachine(outcomes=['Done'])
    with testcase4:
        @smach.cb_interface(outcomes=["succeeded", 'error'])
        def execute(userdata=None):
            return "succeeded"
        smach.StateMachine.add('TEST1',
                                smach.CBState(execute),
                                transitions={'succeeded':'SUBLEVEL1',
                                             'error'    :'SUBLEVEL1'})

        sublevel1 = smach.StateMachine(outcomes=['Finished', 'Failed'])
        with sublevel1:
            smach.StateMachine.add('SUBTEST1',
                                    smach.CBState(execute),
                                    transitions={'succeeded':'SUBTEST2',
                                                 'error'    :'SUBTEST2'})
            smach.StateMachine.add('SUBTEST2',
                                    smach.CBState(execute),
                                    transitions={'succeeded':'Finished',
                                                 'error'    :'Failed'})

        smach.StateMachine.add('SUBLEVEL1',
                                sublevel1,
                                transitions={'Finished' :'Done',
                                             'Failed'   :'Done'})
    visualize(testcase4, "testcase4", save_dot=True)


def draw_subgraph():
    g = Digraph('G', format='png')

    c0 = Digraph('cluster_0')
    c0.body.append('style=filled')
    c0.body.append('color=lightgrey')
    c0.node_attr.update(style='filled', color='white')
    c0.edge('a0', 'a1')
    c0.edge('a1', 'a2')
    c0.edge('a2', 'a3')
    c0.body.append('label = "process #1"')

    c1 = Digraph('cluster_1')
    c1.node_attr.update(style='filled')
    c1.edge('b0', 'b1')
    c1.edge('b1', 'b2')
    c1.edge('b2', 'b3')
    c1.edge('b3', 'b2')
    c1.body.append('label = "process #2"')
    c1.body.append('color=blue')

    g.subgraph(c0)
    g.subgraph(c1)

    g.edge('start', 'a0', lhead='cluster_1')
    g.edge('start', 'b0', lhead='cluster_1')
    g.edge('a1', 'b3')
    g.edge('b2', 'a3')
    g.edge('a3', 'a0')
    g.edge('a3', 'end')
    g.edge('b3', 'end')

    g.node('start', shape='Mdiamond')
    g.node('end', shape='Msquare')

    g.save('draw_subgraph.dot')
    g.render('draw_subgraph')

    # g.view()


if __name__ == "__main__":
    # testcase1()
    # testcase2()
    # testcase3()
    testcase4()

    draw_subgraph()
