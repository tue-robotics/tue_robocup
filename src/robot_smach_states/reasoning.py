#! /usr/bin/env python
import roslib; 
import rospy
import smach
import util
import util.reasoning_helpers as urh

from psi import Compound, Conjunction, Sequence

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
                rospy.loginfo("answers to query: '{0}' = {1}".format(self.query, answers))
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

class Execute_query(smach.State):
    def __init__(self, robot, facts):
        smach.State.__init__(self, outcomes=['executed'])
        assert hasattr(robot, "reasoner")

        self.robot = robot
        self.facts = facts

    def execute(self, userdata=None):
        for fact in self.facts:
            self.robot.reasoner.query(fact)
        return 'executed'

class Select_object(smach.State):
    #TODO 3-12-2014: Select object can be large,ly/completely replaced by a (Variable)Designator
    """Selects an object based on some optional criteria and sorting mechanism"""

    def __init__(self, robot, candidate_query, selection_predicate, 
        sortkey=lambda answer: 1, 
        minmax=min, 
        criteria=None, 
        retract_previous=True, 
        object_variable="ObjectID"):
        smach.State.__init__(self, outcomes=['selected', 'no_answers'])
        """Select an answer from an object_query and assert it to the reasoner as being the selected object, using the selection_predicate.
        The selection is done using the sortkey, ordered with minmax.
        You can also optionally specify some criteria-functions.
        By default, all facts of selection_predicate are retracted before asserting a new selection. This can be overridden by setting retract_previous=False.

        The candidate_query must have a variable that is unified to the object ID we want to select. By default, object_variable=="ObjectID" but can be overridden
        """
        self.robot = robot
        self.candidate_query = candidate_query
        self.selection_predicate = selection_predicate
        self.sortkey = sortkey
        self.criteria = criteria
        self.minmax = minmax
        self.retract_previous = retract_previous
        self.object_variable = object_variable

    def execute(self, userdata=None):
        #Get all answers. object_answers will be a list of dictionaries
        object_answers = self.robot.reasoner.query(self.candidate_query)
        
        try:
            #delegate the actual selection to the reasoning_helpers
            selected_answer = urh.select_answer(object_answers, 
                                                    keyfunc=self.sortkey, 
                                                    minmax=self.minmax,
                                                    criteria=self.criteria)

            rospy.loginfo("Select_object has {0} answers for query {1}. Selected_answer: {2}".format(len(object_answers), self.candidate_query, selected_answer))
            selected_id = selected_answer[self.object_variable]

            if self.retract_previous:
                rospy.logdebug("Retracting previous selections...")
                self.robot.reasoner.query(Compound("retractall", Compound(self.selection_predicate, "X")))

            selection_fact = Compound(self.selection_predicate, selected_id)
            rospy.loginfo("Asserting selected object: {0}".format(selection_fact))
            self.robot.reasoner.assertz(selection_fact)
            return 'selected'
        except ValueError, e:
            rospy.logerr(e)
            return 'no_answers'
        except Exception, e:
            rospy.logerr(e)
            return 'no_answers'

    @staticmethod
    def test(robot):
        #We start with an empty database, with just *wire running*. Or at least none of the used facts asserted.
        #Run astart-fast, amiddle_fast, amigo-console
        #In the console, run: "import roslib; roslib.load_manifest("robot_smach_states"); import robot_smach_states as states; states.Select_object.test(amigo)"
        #This should return "success!"

        robot.reasoner.query(Compound("retractall", Compound("object", "Id", "Pos")))
        robot.reasoner.query(Compound("retractall", Compound("selected_id", "Id")))
        robot.reasoner.query(Compound("retractall", Compound("ignored", "Id")))

        person1 = Compound("object", "id1", Compound("position", Sequence(1,1,1)))
        person2 = Compound("object", "id2", Compound("position", Sequence(2,2,2)))
        person3 = Compound("object", "id3", Compound("position", Sequence(3,3,3)))
        person4 = Compound("object", "id4", Compound("position", Sequence(4,4,4)))
        robot.reasoner.assertz(person1, person2, person3, person4)
        robot.reasoner.assertz(Compound("ignored", "nothing")) #also make the ignored predicate existant

        candidates_query = Conjunction(
                        Compound("object", "ObjectID", Compound("position", Sequence("X", "Y", "Z"))),
                        Compound("not", Compound("ignored", "ObjectID")))
        selected_candidate_query = Compound("selected_id", "ObjectID")
        
        #This state selects an object that is not ignored.
        selector = Select_object(robot, candidates_query, "selected_id", sortkey=lambda answer: str(answer["ObjectID"]))
        ignorer = Select_object(robot, selected_candidate_query, "ignored", retract_previous=False) #

        #import ipdb; ipdb.set_trace()

        selector.execute()
        assert len(robot.reasoner.query(Compound("selected_id", "id1"))) == 1 #The selector should select the first object. Only one object may be selected at a time
        
        ignorer.execute()
         #The first object should be ignored. On each selector-ignorer iteration, we should have more ignored object as we do not retract them 
        assert len(robot.reasoner.query(Compound("ignored", "X"))) == 1+1 #1 extra for making sure the predicate exists

        selector.execute()
        assert len(robot.reasoner.query(Compound("selected_id", "id2"))) == 1
        
        ignorer.execute()
        assert len(robot.reasoner.query(Compound("ignored", "X"))) == 2+1 #1 extra for making sure the predicate exists

        selector.execute()
        assert len(robot.reasoner.query(Compound("selected_id", "id3"))) == 1
        
        ignorer.execute()
        assert len(robot.reasoner.query(Compound("ignored", "X"))) == 3+1 #1 extra for making sure the predicate exists

        selector.execute()
        assert len(robot.reasoner.query(Compound("selected_id", "id4"))) == 1
        
        ignorer.execute()
        assert len(robot.reasoner.query(Compound("ignored", "X"))) == 4+1 #1 extra for making sure the predicate exists

        rospy.loginfo("DEAR TESTER: There are 4 objects, but this is the 5th select-call. Will fail as intended in this test")
        assert selector.execute() == "no_answers"
        
        return "success!"