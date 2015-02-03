#! /usr/bin/env python
import smach

class RobotState(smach.State):
    def __init__(self, *args, **kwargs):
        smach.State.__init__(self, outcomes=kwargs['outcomes'])
        self.__dict__['init_arguments'] = args

    def execute(self, userdata):
        print "Resolving .... "
        resolved_arguments = [(value.resolve() if hasattr(value, "resolve") else value) for value in self.__dict__['init_arguments'][0].iteritems()]
        print resolved_arguments
        print "Done .... "

        return self.run( *resolved_arguments[1:] )

# Wait_for_door state thought the reasoner 
class TestState(RobotState):
    def __init__(self, robot, sentence, blaat):
        RobotState.__init__(self, locals(), outcomes=['yes', 'no'])
        
    def run(self, robot, sentence, blaat):

        print robot, sentence, blaat
        return "yes"

class Test(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['yes','no'])

        with self:
            smach.StateMachine.add('TEST_STATE', TestState("banana", "sentence, hoi ik werk", "bladiabla"))

sm = Test()
sm.execute()
