#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_demo_2014')
import rospy
import sys

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_skills.reasoner import Conjunction, Compound, Sequence
from robot_smach_states.util.startup import startup
import robot_smach_states.util.reasoning_helpers as urh
import std_msgs.msg
import perception_srvs.srv

class NavigateToStartRobust(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, 
                             outcomes=['arrived', 'unreachable',  'preempted', 'goal_not_defined'])
        self.robot = robot
        self.query_start = Compound("waypoint", "start",     Compound("pose_2d", "X", "Y", "Phi"))

        self.query_start1 = Conjunction(  Compound("=", "Waypoint",        Compound("start", "a")),
                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))
        self.query_start2 = Conjunction(  Compound("=", "Waypoint",        Compound("start", "b")),
                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))
        self.query_start3 = Conjunction(  Compound("=", "Waypoint",        Compound("start", "c")),
                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))

        with self:

            smach.StateMachine.add('NAVIGATE_TRY_ONE',
                                    states.NavigateGeneric(self.robot, goal_query=self.query_start1),
                                    transitions={   "arrived":          'arrived',
                                                    "unreachable":      'SAY_NAVIGATE_FAILED_ONE',
                                                    "preempted":        'preempted',
                                                    "goal_not_defined": 'SAY_NAVIGATE_FAILED_ONE'})
            
            smach.StateMachine.add("SAY_NAVIGATE_FAILED_ONE", 
                                    states.Say(robot,"I was not able to reach my goal, I will try again", block=False),
                                    transitions={   'spoken':'NAVIGATE_TRY_TWO'})

            smach.StateMachine.add('NAVIGATE_TRY_TWO',
                                    states.NavigateGeneric(self.robot, goal_query=self.query_start2),
                                    transitions={   "arrived":          'arrived',
                                                    "unreachable":      'SAY_NAVIGATE_FAILED_TWO',
                                                    "preempted":        'preempted',
                                                    "goal_not_defined": 'SAY_NAVIGATE_FAILED_TWO'})
            
            smach.StateMachine.add("SAY_NAVIGATE_FAILED_TWO", 
                                    states.Say(robot,"I was not able to reach my goal, I will try again", block=False),
                                    transitions={   'spoken':'NAVIGATE_TRY_THREE'})

            smach.StateMachine.add('NAVIGATE_TRY_THREE',
                                    states.NavigateGeneric(self.robot, goal_query=self.query_start3),
                                    transitions={   "arrived":          'arrived',
                                                    "unreachable":      'SAY_NAVIGATE_FAILED_THREE',
                                                    "preempted":        'preempted',
                                                    "goal_not_defined": 'SAY_NAVIGATE_FAILED_THREE'})
            
            smach.StateMachine.add("SAY_NAVIGATE_FAILED_THREE", 
                                    states.Say(robot,"I was not able to reach my goal, sorry for that", block=False),
                                    transitions={   'spoken':'unreachable'})

class WaitForTrigger(smach.State):

    def __init__(self, robot, triggers):
        smach.State.__init__(self, 
                             outcomes=triggers+['preempted'])
        self.robot = robot
        self.triggers = triggers

        # Get the ~private namespace parameters from command line or launch file.
        self.rate = float(rospy.get_param('~rate', '1.0'))
        topic     = rospy.get_param('~topic', 'trigger')
        
        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

        rospy.loginfo('topic: /%s', topic)
        rospy.loginfo('rate:  %d Hz', self.rate)

    def execute(self, userdata):
        self.trigger_received = False

        while not rospy.is_shutdown() and not self.trigger_received:
            rospy.sleep(1/self.rate)

        if self.trigger_received:
            return self.trigger_received
        else:
            return 'preempted'

    def callback(self, data):
        # Simply print out values in our custom message.
        rospy.loginfo('trigger_received: %s', data.data)
        if data.data in self.triggers:
            self.trigger_received = data.data

class PackagePose(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        
        self.robot = robot

    def execute(self, gl):        
        rospy.loginfo("start moving to package pose")   

        resultL = self.robot.leftArm.send_joint_goal(-0.05,0.3,0.5,1.8,0.3,0.5,0.0)
        resultR = self.robot.rightArm.send_joint_goal(-0.05,0.3,0.5,1.8,0.3,0.5,0.0)

        if not resultL:
            rospy.loginfo("error sending leftArm goal")
            return 'failed'
        if not resultR:
            rospy.loginfo("error sending rightArm goal")
            return 'failed'
        return 'succeeded' 

class WaitForOwner(smach.State):

    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['person_found','timed_out'])
        self.robot = robot
        self.timeout = 30.0
        # ToDo: fill
        self.query = Conjunction(Compound("property_expected", "ObjectID", "class_label", "person"),
                                 Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))
        # ToDo: don't hardcode
        self.roipos = (2.5, 0.0)

    def execute(self, gl):
        #rospy.loginfo('start_perception: modules=%s' % str(self.modules))
        self.robot.perception.toggle(['ppl_detection', 'ppl_detection_external'])

        rospy.loginfo("Waiting for a person to appear in the region of interest, make sure this happens within 30 seconds!!!")

        starttime = rospy.Time.now()
        person_found = False

        while (not person_found and (rospy.Time.now() - starttime) < rospy.Duration(self.timeout) and not rospy.is_shutdown()):

            ''' Do query '''
            answers = self.robot.reasoner.query(self.query)

            ''' Check for region of interest '''
            try:
                selected_answer = urh.select_answer(answers, 
                                                lambda answer: urh.xy_dist(answer, self.roipos), 
                                                minmax=min,
                                                criteria=[lambda answer: urh.xy_dist(answer, self.roipos) < 1.0])
            except ValueError:
                selected_answer = None

            if selected_answer:
                ''' Assert result '''
                self.robot.reasoner.query(Compound("retractall", Compound("current_person", "X")))
                person_id = selected_answer["ObjectID"]
                rospy.loginfo("Asserting new ID: {0}".format(person_id))
                # assert new object id
                self.robot.reasoner.assertz(Compound("current_person", person_id))

                person_found = True

            rospy.sleep(rospy.Duration(0.1))

        if person_found:
            return 'person_found'
        else:
            return 'timed_out'

class ReceivePackage(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        query_door1 = Conjunction(  Compound("=", "Waypoint",        Compound("behind_door", "a")),
                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))
        query_door2 = Conjunction(  Compound("=", "Waypoint",        Compound("behind_door", "b")),
                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))

        with self:

            smach.StateMachine.add('NAVIGATE_TO_START',
                                    NavigateToStartRobust(robot),
                                    transitions={   "arrived":"SAY_START_REACHED",
                                                    "unreachable":'SAY_GOAL_UNREACHABLE',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'SAY_GOAL_NOT_DEFINED'})
            
            smach.StateMachine.add("SAY_START_REACHED", 
                                    states.Say(robot,"I'm going to wait for further instructions", block=False),
                                    transitions={   'spoken':'WAIT_FOR_TRIGGER'})

            smach.StateMachine.add("WAIT_FOR_TRIGGER", 
                                    WaitForTrigger(robot, ['allow', 'deny', 'doorbell']),
                                    transitions={   'allow':    'SAY_TRIGGER_ALLOW',
                                                    'deny':     'SAY_TRIGGER_DENY',
                                                    'doorbell' : 'WAIT_FOR_DOORBELL_SOUND',
                                                    'preempted':'failed'})

            smach.StateMachine.add( 'WAIT_FOR_DOORBELL_SOUND',
                        states.Wait_time(robot, waittime=2),
                                    transitions={   'waited':'SAY_SOMEONE_AT_THE_DOOR',
                                                    'preempted':'failed'})
            
            smach.StateMachine.add("SAY_SOMEONE_AT_THE_DOOR", 
                                    states.Say(robot,"There is somebody at the door, I will notify my owner.", block=True),
                                    transitions={   'spoken':'WAIT_FOR_TRIGGER'})
            
            smach.StateMachine.add("SAY_TRIGGER_ALLOW", 
                                    states.Say(robot,"My owned said I could open the door, I'm on my way!", block=False),
                                    transitions={   'spoken':'NAVIGATE_TO_DOOR'})

            smach.StateMachine.add("SAY_TRIGGER_DENY", 
                                    states.Say(robot,"I dont know that guy, my owner told me not to open the door for strangers", block=False),
                                    transitions={   'spoken':'WAIT_FOR_TRIGGER'})

            smach.StateMachine.add('NAVIGATE_TO_DOOR',
                                    states.NavigateGeneric(robot, goal_query=query_door1),
                                    transitions={   "arrived":"SAY_DOOR_REACHED",
                                                    "unreachable":'NAVIGATE_TO_DOOR_RETRY',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'NAVIGATE_TO_DOOR_RETRY'})

            smach.StateMachine.add('NAVIGATE_TO_DOOR_RETRY',
                                    states.NavigateGeneric(robot, goal_query=query_door2),
                                    transitions={   "arrived":"SAY_DOOR_REACHED",
                                                    "unreachable":'SAY_GOAL_UNREACHABLE',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'SAY_GOAL_NOT_DEFINED'})

            smach.StateMachine.add("SAY_DOOR_REACHED", 
                                    states.Say(robot,"Can I receive your package?", block=False),
                                    transitions={   'spoken':'RECEIVE_POSE'})
            
            smach.StateMachine.add("RECEIVE_POSE", 
                                    PackagePose(robot),
                                    transitions={   'succeeded':'WAIT_FOR_LOAD',
                                                    'failed':'WAIT_FOR_LOAD'})

            smach.StateMachine.add( 'WAIT_FOR_LOAD',
                        states.Wait_time(robot, waittime=5),
                                    transitions={   'waited':'SAY_PACKAGE_RECEIVED',
                                                    'preempted':'failed'})
            
            smach.StateMachine.add("SAY_PACKAGE_RECEIVED", 
                                    states.Say(robot,"Thank you, I will hold on to the package until my owner comes home.", block=False),
                                    transitions={   'spoken':'NAVIGATE_TO_START_2'})

            smach.StateMachine.add('NAVIGATE_TO_START_2',
                                    NavigateToStartRobust(robot),
                                    transitions={   "arrived":"succeeded",
                                                    "unreachable":'SAY_GOAL_UNREACHABLE',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'SAY_GOAL_NOT_DEFINED'})

            # navigation states
            smach.StateMachine.add("SAY_GOAL_UNREACHABLE", 
                                    states.Say(robot,"I am sorry but I cannot figure it out, the goal is unreachable"),
                                    transitions={   'spoken':'failed'})

            smach.StateMachine.add("SAY_GOAL_NOT_DEFINED", 
                                    states.Say(robot,"I am sorry, I don't know where to go"),
                                    transitions={   'spoken':'failed'})

class GivePackage(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot

        self.query_start1 = Conjunction(  Compound("=", "Waypoint",        Compound("start", "a")),
                                            Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))

        query_backup= Compound("waypoint", "user_backup",   Compound("pose_2d", "X", "Y", "Phi"))
        query_owner = Conjunction(Compound("current_person", "ObjectID"),
                                  Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))
        with self:

            smach.StateMachine.add('WAIT_FOR_OWNER',
                                    WaitForOwner(robot),
                                    transitions={   "person_found":"SAY_OWNER_FOUND",
                                                    "timed_out":"SAY_PERSON_TIMEOUT"})

            smach.StateMachine.add("SAY_OWNER_FOUND", 
                                    states.Say(robot,"Hey, the house notified me that my owned entered. I will deliver the package to him.", block=False),
                                    transitions={   'spoken':'NAVIGATE_TO_OWNER'})

            smach.StateMachine.add("SAY_PERSON_TIMEOUT", 
                                    states.Say(robot,"It took too long, I better go to where he usually is.", block=False),
                                    transitions={   'spoken':'NAVIGATE_TO_OWNER_BACKUP'})

            smach.StateMachine.add('NAVIGATE_TO_OWNER',
                                    states.NavigateGeneric(robot, lookat_query=query_owner, refresh_freq=1),
                                    transitions={   "arrived":"SAY_PERSON_FOUND",
                                                    "unreachable":'SAY_PERSON_UNREACHABLE',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'SAY_PERSON_LOST'})

            smach.StateMachine.add("SAY_PERSON_LOST", 
                                    states.Say(robot,"I lost my operator, I better go where he usually is", block=False),
                                    transitions={   'spoken':'NAVIGATE_TO_OWNER_BACKUP'})

            smach.StateMachine.add("SAY_PERSON_UNREACHABLE", 
                                    states.Say(robot,"I lost my operator, I better go where he usually is", block=False),
                                    transitions={   'spoken':'NAVIGATE_TO_OWNER_BACKUP'})

            smach.StateMachine.add('NAVIGATE_TO_OWNER_BACKUP',
                                    states.NavigateGeneric(robot, goal_query=query_backup),
                                    transitions={   "arrived":"SAY_PERSON_FOUND",
                                                    "unreachable":'SAY_GOAL_UNREACHABLE',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'SAY_GOAL_NOT_DEFINED'})

            smach.StateMachine.add("SAY_PERSON_FOUND", 
                                    states.Say(robot,"Hello owner, I have a package for you..", block=True),
                                    transitions={   'spoken':'SAY_CALL_ME_WHEN_YOU_NEED_ME_AGAIN'})

            smach.StateMachine.add("SAY_CALL_ME_WHEN_YOU_NEED_ME_AGAIN", 
                                    states.Say(robot,"Call me when you need me again.", block=False),
                                    transitions={   'spoken':'NAVIGATE_TRY_ONE'})

            smach.StateMachine.add('NAVIGATE_TRY_ONE',
                                    states.NavigateGeneric(self.robot, goal_query=self.query_start1),
                                    transitions={   "arrived":          'succeeded',
                                                    "unreachable":      'SAY_GOAL_UNREACHABLE',
                                                    "preempted":        'failed',
                                                    "goal_not_defined": 'SAY_GOAL_NOT_DEFINED'})

            # navigation states
            smach.StateMachine.add("SAY_GOAL_UNREACHABLE", 
                                    states.Say(robot,"I am sorry but I cannot figure it out, the goal is unreachable"),
                                    transitions={   'spoken':'failed'})

            smach.StateMachine.add("SAY_GOAL_NOT_DEFINED", 
                                    states.Say(robot,"I am sorry, I don't know where to go"),
                                    transitions={   'spoken':'failed'})

class ChallengeDemo2014(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        #retract old facts
        #TODO: maybe retract more facts like other challenges?
        robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))

        #Load database
        robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

        #Assert the current challenge.
        robot.reasoner.query(Compound("assertz",Compound("challenge", "challenge_demo_2014")))

        with self:

            smach.StateMachine.add("INITIALIZE", 
                                    states.Initialize(robot),
                                    transitions={   'initialized':  'RECEIVE_PACKAGE',
                                                    'abort':        'Aborted'})


            smach.StateMachine.add("RECEIVE_PACKAGE", 
                                    ReceivePackage(robot),
                                    transitions={   'succeeded': 'GIVE_PACKAGE',
                                                    'failed':    'GIVE_PACKAGE'})

            smach.StateMachine.add("GIVE_PACKAGE", 
                                    GivePackage(robot),
                                    transitions={   'succeeded': 'Done',
                                                    'failed':    'Done'})
            if  len(sys.argv) > 1:
                initial_state = sys.argv[1]
                rospy.logwarn("Setting initial state to {0}, please make sure the reasoner is reset and the robot is localized correctly".format(initial_state))
                self.set_initial_state([initial_state])


if __name__ == "__main__":
    rospy.init_node('challenge_demo_2014_exec')

    startup(ChallengeDemo2014)
