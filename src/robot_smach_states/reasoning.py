#! /usr/bin/env python
import roslib; roslib.load_manifest("robot_smach_states")
import rospy
import smach
import util

from psi import Compound, Conjunction

# Wait_for_door state thought the reasoner
@util.deprecated
class Wait_for_door_reasoner(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['door_open'])
        
        self.robot = robot
        
    def execute(self,userdata=None):
        r = self.robot.reasoner
        query = Compound("state", "door1", "open")  # maybe change door1 to entrance_door would be better

        while not self.preempt_requested():
            rospy.Rate(5).sleep()
            answer = r.query(query)
            if answer:
                return 'door_open'        

# Wait_for_door state thought the reasoner 
class Ask_query_true(smach.State):
    def __init__(self, robot, query):
        smach.State.__init__(self, outcomes=['query_false','query_true','waiting','preempted'])
        
        self.robot = robot

        self.preempted = False
        
        self.defined_query = query

        # time before query should be true:
        self.waittime = rospy.Duration(0.1)

        self.tries = 0


    def execute(self,userdata=None):
        waittime = self.waittime
        starttime = rospy.Time.now()
        time2 = rospy.Time.now()
        self.tries = self.tries + 1

        ## Plot try nr.
        #rospy.loginfo("Try nr. {0} ".format(self.tries))
        
        if self.tries > 1200 : #= about 120 seconds
            self.tries = 0
            return 'waiting'

        while not self.preempt_requested() and ((time2-starttime) < waittime):
            rospy.Rate(5).sleep()
            answer = self.robot.reasoner.query(self.defined_query)
            time2 = rospy.Time.now()
            
            if answer:
                self.tries = 0
                return 'query_true'
        else:
            if self.preempt_requested():
                self.tries = 0
                self.service_preempt()
                return 'preempted'
            else:
                return 'query_false'

class Wait_query_true(smach.State):
    def __init__(self, robot, query, timeout=10, pre_callback=None, post_callback=None):
        smach.State.__init__(self, outcomes=['query_true','timed_out','preempted'])
        assert hasattr(robot, "reasoner")

        self.robot = robot
        self.query = query
        self.query_is_true = False
        self.timeout = rospy.Duration(timeout)

        def decorate_with_pre_post(func):
            def execute_wrapper(*args, **kwargs):
                if callable(pre_callback):
                    pre_callback(*args, **kwargs)

                output = func(*args, **kwargs)
                
                if callable(post_callback):
                    post_callback(*args, **kwargs)

                return output
            return execute_wrapper

        self.execute = decorate_with_pre_post(self.execute)

    def execute(self, userdata=None):
        starttime = rospy.Time.now()
        while (rospy.Time.now()-starttime) < self.timeout:
            answers = self.robot.reasoner.query(self.query)
            if answers:
                return 'query_true'
            else:
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'
            rospy.sleep(0.1)
        return 'timed_out'

class Retract_facts(smach.State):
    def __init__(self, robot, facts):
        smach.State.__init__(self, outcomes=['retracted'])
        assert hasattr(robot, "reasoner")

        self.robot = robot
        self.facts = facts

    def execute(self, userdata=None):
        for fact in self.facts:
            self.robot.reasoner.query(Compound("retractall", fact))
        return 'retracted'


