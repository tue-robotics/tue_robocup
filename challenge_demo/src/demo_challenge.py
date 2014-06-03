#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_demo')
import rospy
import sys

import smach

from robot_skills.amigo import Amigo
import robot_skills.util.msg_constructors as msgs

import robot_smach_states as states

from robot_smach_states.util.startup import startup

grasp_arm = "left"

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
        self.robot.perception.toggle(['ppl_detection_external'])

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

class WaitForExternalCamera(smach.StateMachine):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['present', 'not_present'])
        self.object_list = []
        self.rate = 1.0

        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

        rospy.loginfo('topic: /%s', topic)
        rospy.loginfo('rate:  %d Hz', self.rate)

    def execute(self, gl):

        





class ReceivePackage(smach.StateMachine):

    def __init__(self, robot, arm):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        query_door1 = Conjunction(  Compound("=", "Waypoint",        Compound("behind_door", "a")),
                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))
        query_door2 = Conjunction(  Compound("=", "Waypoint",        Compound("behind_door", "b")),
                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))

        with self:

            smach.StateMachine.add("WAIT_FOR_DOORBELL", 
                                    states.WaitForTrigger(robot, ['doorbell']),
                                    transitions={   'doorbell' : 'WAIT_FOR_DOORBELL_SOUND',
                                                    'preempted':'failed'})

            smach.StateMachine.add( 'WAIT_FOR_DOORBELL_SOUND',
                                    states.Wait_time(robot, waittime=2),
                                    transitions={   'waited':'SAY_SOMEONE_AT_THE_DOOR',
                                                    'preempted':'failed'})
            
            smach.StateMachine.add("SAY_SOMEONE_AT_THE_DOOR", 
                                    states.Say(robot,"There is somebody at the door, I should go over there.", block=True),
                                    transitions={   'spoken':'NAVIGATE_TO_DOOR'})
            

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
            
            smach.StateMachine.add("RECEIVE_OBJECT", 
                                    states.Human_handover(arm, robot),
                                    transitions={   'succeeded':'SAY_PACKAGE_RECEIVED',
                                                    'failed':'SAY_PACKAGE_RECEIVED'})
            
            smach.StateMachine.add("SAY_PACKAGE_RECEIVED", 
                                    states.Say(robot,"Thank you, I will bring this to the man of the house", block=False),
                                    transitions={   'spoken':'succeeded'})

            # navigation states
            smach.StateMachine.add("SAY_GOAL_UNREACHABLE", 
                                    states.Say(robot,"I am sorry but I cannot figure it out, the goal is unreachable"),
                                    transitions={   'spoken':'failed'})

            smach.StateMachine.add("SAY_GOAL_NOT_DEFINED", 
                                    states.Say(robot,"I am sorry, I don't know where to go"),
                                    transitions={   'spoken':'failed'})

class FetchObject(smach.StateMachine):

	def __init__(self, robot, arm):

		smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

		# ToDo: fill/check queries
		roi_query = 
		object_query = 

		query_owner = Conjunction(Compound("current_person", "ObjectID"),
                                  Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

		with self:

			# ToDo: update Ramon's list?
			# ToDo: listen to selected object and assert current object to reasoner
			# ToDo: determine whether object is present and decide whether to go and get it or to receive it at the door

            # ToDo: don't hardcode the following four states and do it a bit more intelligent
            smach.StateMachine.add("WAIT_FOR_ORDER", 
                                    states.WaitForTrigger(robot, ['coke', 'sprite', 'fanta']),
                                    transitions={   'coke'      : 'ASSERT_COKE',
                                                    'sprite'    : 'ASSERT_SPRITE',
                                                    'fanta'     : 'ASSERT_FANTA',
                                                    'preempted' : 'failed'})

            @smach.cb_interface(input_keys=['object'],
                    output_keys=[],
                    outcomes=['succeeded', 'failed'])
            def assert_object(ud):
                desired_object = ud.object

                # ToDo: assert this to the reasoner!
                
                return 'succeeded'

            smach.StateMachine.add('ASSERT_COKE', CBState(assert_object,
                                    cb_args=['coke'],
                                    transitions={   'succeeded' : 'GET_OBJECT_SHELF',
                                                    'failed'    : 'SAY_FAIL'})

            smach.StateMachine.add('ASSERT_SPRITE', CBState(assert_object,
                                    cb_args=['sprite'],
                                    transitions={   'succeeded' : 'GET_OBJECT_SHELF',
                                                    'failed'    : 'SAY_FAIL'})

            smach.StateMachine.add('ASSERT_FANTA', CBState(assert_object,
                                    cb_args=['fanta'],
                                    transitions={   'succeeded' : 'GET_OBJECT_SHELF',
                                                    'failed'    : 'SAY_FAIL'})



			smach.StateMachine.add('GET_OBJECT_SHELF',
									states.GetObject(robot, arm, roi_query, object_query),
									transitions={	'Done'		: 'SAY_KNOW_OWNER',
													'Aborted'	: 'SAY_FAIL',
													'Failed'	: 'SAY_FAIL',
													'Timeout'	: 'SAY_FAIL' })

			smach.StateMachine.add('GET_OBJECT_DOOR',
				            		ReceivePackage(robot, arm),
				            		transitions={	'succeeded'	: 'SAY_KNOW_OWNER',
				            						'failed'	: 'SAY_KNOW_OWNER_FAIL'})

			smach.StateMachine.add('SAY_KNOW_OWNER',
									states.Say(robot, "Luckily I know where my owner is", block=False),
									transitions={	'spoken'	: 'NAVIGATE_TO_OWNER'})

			smach.StateMachine.add('NAVIGATE_TO_OWNER',
                                    states.NavigateGeneric(robot, lookat_query=query_owner, refresh_freq=1),
                                    transitions={   "arrived":"SAY_HANDOVER",
                                                    "unreachable":'SAY_CANNOT_REACH_OWNER',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'SAY_LOST_OWNER'})

			smach.StateMachine.add('SAY_KNOW_OWNER_FAIL',
									states.Say(robot, "Luckily I know where my owner is", block=False),
									transitions={	'spoken'	: 'NAVIGATE_TO_OWNER_FAIL'})

			smach.StateMachine.add('NAVIGATE_TO_OWNER_FAIL',
                                    states.NavigateGeneric(robot, lookat_query=query_owner, refresh_freq=1),
                                    transitions={   "arrived":"HANDOVER_TO_HUMAN",
                                                    "unreachable":'SAY_CANNOT_REACH_OWNER',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'SAY_LOST_OWNER'})

			smach.StateMachine.add('SAY_FAIL',
									states.Say(robot, "I am terribly sorry, but I could not get what you wanted", block=False, mood="sad"),
									transitions={	'spoken'	: 'failed'})

			smach.StateMachine.add('SAY_LOST_OWNER',
									states.Say(robot, "Despite the additional lasers, I cannot find my boss"),
									transitions={	'spoken'	: 'failed'})

			smach.StateMachine.add('SAY_CANNOT_REACH_OWNER',
									states.Say(robot, "I cannot seem to go to my boss"),
									transitions={	'spoken'	: 'failed'})

			smach.StateMachine.add('SAY_HANDOVER',
									states.Say(robot, "Hi, I got you exactly what you wanted", block=False),
									transitions={ 'spoken'		: 'HANDOVER_TO_HUMAN'})

			smach.StateMachine.add('HANDOVER_TO_HUMAN',
									states.HandoverToHuman(arm, robot),
									transitions={	'succeeded'	:	'succeeded',
													'failed'	:	'failed'})

class ChallengeDemo2014(smach.StateMachine):

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        if grasp_arm == "right": 
            arm = robot.rightArm
        else:            
            arm = robot.leftArm

        #retract old facts
        #TODO: maybe retract more facts like other challenges?
        robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))

        #Load database
        robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

        #Assert the current challenge.
        robot.reasoner.query(Compound("assertz",Compound("challenge", "challenge_demo")))

        # ToDo: don't hardcode!!!
        external_ppl_target   = msgs.PointStamped(4.1, 1.8, 1.0, frame_id="/map")
        external_ppl_lenght_x = 4.0
        external_ppl_lenght_y = 3.5
        external_ppl_lenght_z = 1.0

        with self:

            smach.StateMachine.add("INITIALIZE", 
                                    states.Initialize(robot),
                                    transitions={   'initialized':  'TOGGLE_EXTERNAL_PPL',
                                                    'abort':        'Aborted'})

            # ToDo: make sure it does not get switched off again
            # ToDo: assert ID
            #smach.StateMachine.add("TOGGLE_EXTERNAL_PPL", states.ToggleModules(robot,['ppl_detection_external'],
            #	                                                               external_ppl_target,
            #	                                                               external_ppl_lenght_x,
            # 	                                                               external_ppl_lenght_y,
            #	                                                               external_ppl_lenght_z),
            #	                    transitions={	'toggled':		'Done'})

            smach.StateMachine.add('WAIT_FOR_OWNER',
                                    WaitForOwner(robot),
                                    transitions={   "person_found":	"Done",
                                                    "timed_out":	"SAY_ABORT"})

            smach.StateMachine.add('FETCH_FIRST_OBJECT',
            	                    FetchObject(robot, arm),
            	                    transitions={	'succeeded' :	'Done',
            	                    				'failed'	:	'SAY_ABORT'})

            smach.StateMachine.add('SAY_ABORT',
            						states.Say(robot,
            							"Something went terribly wrong. Unfortunately I cannot complete my task so I will stop now",
            							mood="sad",
            							block=False),
            						transitions={	'spoken'	:	'Aborted'})

            if  len(sys.argv) > 1:
                initial_state = sys.argv[1]
                rospy.logwarn("Setting initial state to {0}, please make sure the reasoner is reset and the robot is localized correctly".format(initial_state))
                self.set_initial_state([initial_state])

if __name__ == "__main__":
    rospy.init_node('challenge_demo_2014_exec')

    startup(ChallengeDemo2014)