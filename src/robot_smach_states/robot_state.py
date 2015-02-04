#! /usr/bin/env python
import smach

class RobotState(smach.State):
    def __init__(self, *args, **kwargs):
        smach.State.__init__(self, outcomes=kwargs['outcomes'])
        self.__dict__['init_arguments'] = args

    def execute(self, userdata):
        resolved_arguments = [(value.resolve() if hasattr(value, "resolve") else value) for value 
            in self.__dict__['init_arguments'][0].iteritems()]
        resolved_arguments_dict = dict(resolved_arguments[1:] )

        return self.run( **resolved_arguments_dict )

# Wait_for_door state thought the reasoner 
class TestState(RobotState):
    def __init__(self, robot, sentence, blaat):
        RobotState.__init__(self, locals(), outcomes=['yes', 'no'])
        
    def run(self, robot, sentence, blaat):
        print robot, sentence, blaat
        return "yes"

class Test(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:
            smach.StateMachine.add( 'TEST_STATE', 
                                    TestState("Yes", "this", "works"),
                                    transitions={'yes':'succeeded', 'no':'failed'})

sm = Test()
sm.execute()
