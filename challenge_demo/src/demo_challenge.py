#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_demo')
import rospy
import sys

import smach
import std_msgs
import robot_smach_states.util.reasoning_helpers as urh

from robot_skills.amigo import Amigo
import robot_skills.util.msg_constructors as msgs

import robot_smach_states as states

from robot_smach_states.util.startup import startup

from psi import *

grasp_arm = "left"
class_names = ['milk', 'pringles', 'noodles', 'biscuits']
amount_of_objects = 4

class AwaitTriggerAndAssertValue(smach.State):
    '''
    This state will block execution until a suitable trigger command is received on the channel /trigger
    It will receive std_msgs.String and will compare it to the strings in the array that is given.

    Example to wait for one of the strings 'allow' or 'deny' (could be sent from a gui):

        AwaitTriggerAndAssertValue(robot, ['allow', 'deny'], assert_as='some_unary_predicate', retract_first=False), 
                       transitions={    'allow':     'DO_SOMETHING',
                                        'deny':      'DO_SOMETHING',
                                        'preempted': 'failed'})
    '''

    def __init__(self, robot, triggers, assert_as="", retract_first=True):
        """
        @param assert_as: the value of the trigger will be asserted to the reasoner with this unary predicate. 
        @retract_first to False makes it add a fact for every trigger. Default is true, so it only remembers the last"""
        smach.State.__init__(self, outcomes=['triggered', 'preempted'])
        self.robot = robot
        self.triggers = triggers
        self.assert_as = assert_as
        self.retract_first = retract_first

        # Get the ~private namespace parameters from command line or launch file.
        self.rate = float(rospy.get_param('~rate', '1.0'))
        topic     = rospy.get_param('~topic', 'trigger')
        
        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

        rospy.loginfo('topic: /%s', topic)
        rospy.loginfo('rate:  %d Hz', self.rate)

    def execute(self, userdata):
        self.trigger_received = False
        #rospy.logwarn("Waiting for trigger (any of {0}) on topic /trigger".format(self.triggers))
        while not rospy.is_shutdown() and not self.trigger_received:
            rospy.sleep(1/self.rate)

        if self.trigger_received:
            if self.retract_first:
                self.robot.reasoner.query(Compound("retractall", Compound(self.assert_as, "X")))
            self.robot.reasoner.assertz(Compound(self.assert_as, str(self.trigger_received)))

            return 'triggered'
        else:
            return 'preempted'

    def callback(self, data):
        # Simply print out values in our custom message.
        if data.data in self.triggers:
            rospy.loginfo('trigger received: %s', data.data)
            self.trigger_received = data.data
        else:
            rospy.logwarn('wrong trigger received: %s', data.data)

class WaitForOwner(smach.State):

    def __init__(self, robot, timeout = 30, roi_pos_x = 5.0, roi_pos_y = 2.0, detecting_range = 1.5):
        smach.State.__init__(self, outcomes=['person_found','timed_out'])
        self.robot = robot
        self.timeout = int(timeout)
        self.query = Conjunction(Compound("property_expected", "ObjectID", "class_label", "person"),
                                 Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        self.roipos = (int(roi_pos_x), int(roi_pos_y))
        self.detecting_range = int(detecting_range)

    def execute(self, gl):
        #rospy.loginfo('start_perception: modules=%s' % str(self.modules))
        self.robot.perception.toggle_always_on(['ppl_detection_external'])

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
                                                criteria=[lambda answer: urh.xy_dist(answer, self.roipos) < self.detecting_range])
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

class WaitForExternalCamera(smach.State):

    def __init__(self, robot, object_query):
        smach.State.__init__(self,
                             outcomes=['present', 'not_present', 'failed'])

        self.robot = robot
        self.object_query = object_query

        # Get the ~private namespace parameters from command line or launch file.
        self.rate = float(rospy.get_param('~rate', '1.0'))
        topic     = rospy.get_param('~topic', '/detected_objects')

        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

        rospy.loginfo('topic: /%s', topic)
        rospy.loginfo('rate:  %d Hz', self.rate)

        self.object_list = []

    def execute(self, gl):

        answers = self.robot.reasoner.query(self.object_query)

        if not answers:
            rospy.logerr("Dont know which object to get")
            return 'failed'
        if not self.object_list:
            rospy.logerr("Object list is empty!!!")
            return 'not_present'
        
	obj = answers[0]["Desired_object"]
	obj = str(obj) # very important !!!

        if obj in self.object_list:
            return 'present'
        else:
            return 'not_present'

    def callback(self, msg):
        temp_object_list = msg.data.split("|")

        for obj in temp_object_list:
            if not obj in self.object_list:
                self.object_list.append(obj)

	print '\tcurrent object_list:', str(self.object_list)

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
                                    transitions={   'spoken':'RECEIVE_OBJECT'})

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

        self.robot = robot
        self.arm = arm

        roi_query = Conjunction(    Compound("=", "ROI_Location", Compound("demo_object_location", "W")),
                                    Compound("point_of_interest", "ROI_Location", Compound("point_3d", "X", "Y", "Z")))

        object_identifier_query = "ROI_Location"

        object_query = Conjunction( Compound("demo_get_object", "Desired_object"),
                                    Compound("property_expected", "ObjectID", "class_label", "Desired_object"),
                                    Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        camera_object_query = Compound("demo_get_object", "Desired_object")

        query_owner = Conjunction(Compound("current_person", "ObjectID"),
                                  Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        query_failure_owner1 = Conjunction(  Compound("=", "Waypoint",        Compound("backup_room_owner", "a")),
                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))
        query_failure_owner2 = Conjunction(  Compound("=", "Waypoint",        Compound("backup_room_owner", "b")),
                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")))

        with self:
            # ToDo: don't hardcode the following four states and do it a bit more intelligent
            smach.StateMachine.add("WAIT_FOR_ORDER",
                                    AwaitTriggerAndAssertValue(  robot, class_names, assert_as="demo_get_object"),
                                    transitions={   'triggered' : 'IS_PRESENT',
                                                    'preempted' : 'failed'})

            smach.StateMachine.add('IS_PRESENT', WaitForExternalCamera(robot, camera_object_query),
                                    transitions={   'present'   : 'SAY_PRESENT',
                                                    'not_present':'SAY_NOT_PRESENT',
                                                    'failed'    : 'SAY_FAIL'})

            smach.StateMachine.add('SAY_PRESENT', states.Say(robot, "We still have that in stock, I will go and get it", block=False),
                                    transitions={   'spoken'    : 'RETRACT_VISITED'})

            smach.StateMachine.add("RETRACT_VISITED",
                                    states.Retract_facts(robot, [Compound("visited", "X")]),
                                    transitions={'retracted':'RETRACT_UNREACHABLE'})

            smach.StateMachine.add("RETRACT_UNREACHABLE",
                                    states.Retract_facts(robot, [Compound("unreachable", "X")]),
                                    transitions={'retracted':'GET_OBJECT_SHELF'})

            smach.StateMachine.add('GET_OBJECT_SHELF',
                                    states.GetObject(robot, side=arm,  roi_query=roi_query, roi_identifier="ROI_Location", object_query=object_query, object_identifier=object_identifier_query, max_duration=rospy.Duration(180)),
                                    transitions={   'Done'      : 'SAY_KNOW_OWNER',
                                                    'Aborted'   : 'SAY_FAIL',
                                                    'Failed'    : 'SAY_FAIL',
                                                    'Timeout'   : 'SAY_FAIL' })

            smach.StateMachine.add('SAY_NOT_PRESENT', states.Say(robot, "That is not in stock, I better order it online", block=False),
                                    transitions={   'spoken'    : 'GET_OBJECT_DOOR'})

            smach.StateMachine.add('GET_OBJECT_DOOR',
                                    ReceivePackage(robot, arm),
                                    transitions={   'succeeded' : 'SAY_KNOW_OWNER',
                                                    'failed'    : 'SAY_FAIL'})

            smach.StateMachine.add('SAY_KNOW_OWNER',
                                    states.Say(robot, "Luckily I know where my owner is", block=False),
                                    transitions={   'spoken'    : 'NAVIGATE_TO_OWNER'})

            smach.StateMachine.add('NAVIGATE_TO_OWNER',
                                    states.NavigateGeneric(robot, lookat_query=query_owner, refresh_freq=1),
                                    transitions={   "arrived":"SAY_HANDOVER",
                                                    "unreachable":'SAY_CANNOT_REACH_OWNER',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'SAY_LOST_OWNER'})

            smach.StateMachine.add('SAY_FAIL',
                                    states.Say(robot, "I am terribly sorry, but I could not get what you wanted", block=False, mood="sad"),
                                    transitions={   'spoken'    : 'RESET_ARMS_SPINDLE_HEAD_UPON_FAILURE'})

            smach.StateMachine.add('SAY_LOST_OWNER',
                                    states.Say(robot, "Despite the additional lasers, I cannot find my boss, I will just go to the living room", block=False),
                                    transitions={   'spoken'    : 'NAVIGATE_TO_ROOM_OWNER'})

            smach.StateMachine.add('SAY_CANNOT_REACH_OWNER',
                                    states.Say(robot, "I cannot seem to go to my boss, I will just go to the living room", block=False),
                                    transitions={   'spoken'    : 'NAVIGATE_TO_ROOM_OWNER'})


            smach.StateMachine.add('NAVIGATE_TO_ROOM_OWNER',
                                    states.NavigateGeneric(robot, goal_query=query_failure_owner1),
                                    transitions={   "arrived":"SAY_HANDOVER_FAILURE",
                                                    "unreachable":'NAVIGATE_TO_ROOM_OWNER_RETRY',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'NAVIGATE_TO_ROOM_OWNER_RETRY'})

            smach.StateMachine.add('NAVIGATE_TO_ROOM_OWNER_RETRY',
                                    states.NavigateGeneric(robot, goal_query=query_failure_owner2),
                                    transitions={   "arrived":"SAY_HANDOVER_FAILURE",
                                                    "unreachable":'SAY_HANDOVER_FAILURE',
                                                    "preempted":'failed',
                                                    "goal_not_defined":'SAY_HANDOVER_FAILURE'})

            smach.StateMachine.add('SAY_HANDOVER',
                                    states.Say(robot, "Hi, here is your request!", block=False),
                                    transitions={ 'spoken'      : 'HANDOVER_TO_HUMAN'})

            smach.StateMachine.add('SAY_HANDOVER_FAILURE',
                                    states.Say(robot, "I will just give you the desired object here"),
                                    transitions={ 'spoken'      : 'HANDOVER_TO_HUMAN_FAILURE'})

            smach.StateMachine.add('HANDOVER_TO_HUMAN',
                                    states.HandoverToHuman(arm, robot),
                                    transitions={   'succeeded' :   'RESET_ARMS_SPINDLE_HEAD_UPON_SUCCESS',
                                                    'failed'    :   'RESET_ARMS_SPINDLE_HEAD_UPON_FAILURE'})

            smach.StateMachine.add('HANDOVER_TO_HUMAN_FAILURE',
                                    states.HandoverToHuman(arm, robot),
                                    transitions={   'succeeded' :   'RESET_ARMS_SPINDLE_HEAD_UPON_FAILURE',
                                                    'failed'    :   'RESET_ARMS_SPINDLE_HEAD_UPON_FAILURE'})


            smach.StateMachine.add("RESET_ARMS_SPINDLE_HEAD_UPON_SUCCESS",
                                    states.ResetArmsSpindleHead(robot),
                                    transitions={'done':'succeeded'})

            smach.StateMachine.add("RESET_ARMS_SPINDLE_HEAD_UPON_FAILURE",
                                    states.ResetArmsSpindleHead(robot),
                                    transitions={'done':'failed'})

class ChallengeDemo2014(smach.StateMachine):

    def __init__(self, robot=None):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted'])

        if grasp_arm == "right":
            arm = robot.rightArm
        else:
            arm = robot.leftArm

        #retract old facts
        robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
        robot.reasoner.query(Compound("retractall", Compound("demo_get_object", "X")))
        robot.reasoner.query(Compound("retractall", Compound("visited", "X")))
        robot.reasoner.query(Compound("retractall", Compound("registered", "X")))
        robot.reasoner.query(Compound("retractall", Compound("current_person", "X")))
        robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
        robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))

        #Load database
        robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))
        robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))

        #Assert the current challenge.
        robot.reasoner.query(Compound("assertz",Compound("challenge", "challenge_demo")))

        # # ToDo: don't hardcode!!! (erik: not used anymore(?))
        # external_ppl_target   = msgs.PointStamped(4.1, 1.8, 1.0, frame_id="/map")
        # external_ppl_lenght_x = 4.0
        # external_ppl_lenght_y = 3.5
        # external_ppl_lenght_z = 1.0

        with self:

            smach.StateMachine.add("INITIALIZE",
                                    states.Initialize(robot),
                                    transitions={   'initialized':  'WAIT_FOR_OWNER',
                                                    'abort':        'Aborted'})

            # ToDo : in the end stop ppl detection

            smach.StateMachine.add('WAIT_FOR_OWNER', # pose_2d(1.964, 8.018, 2.82)
                                    WaitForOwner(robot, timeout = 30, roi_pos_x = 2.0, roi_pos_y = 8.0, detecting_range = 1.0),
                                    transitions={   "person_found": "FETCH_OBJECTS",
                                                    "timed_out":    "FETCH_OBJECTS"})

            fetch_object_iterator = smach.Iterator(outcomes=['succeeded', 'failed'],
                                                   input_keys=[],
                                                   output_keys=[],
                                                   it=lambda: range(amount_of_objects), 
                                                   it_label='object_count', 
                                                   exhausted_outcome='exhausted')
            
            with fetch_object_iterator:
                fetch_one_object =  FetchObject(robot, arm)

                fetch_object_iterator.set_contained_state('FIND_UNKNOWN', 
                                          fetch_one_object, 
                                          loop_outcomes=['succeeded', 'failed'], 
                                          break_outcomes=[])
            
            smach.StateMachine.add('FETCH_OBJECTS', 
                                   fetch_object_iterator, 
                                   transitions={'succeeded':'Done', 
                                                'failed':'SAY_ABORT',
                                                'exhausted':'Done'})

            smach.StateMachine.add('SAY_ABORT',
                                    states.Say(robot,
                                        "Something went terribly wrong. Unfortunately I cannot complete my task so I will stop now",
                                        mood="sad",
                                        block=False),
                                    transitions={   'spoken'    :   'Aborted'})

            if  len(sys.argv) > 1:
                initial_state = sys.argv[1]
                rospy.logwarn("Setting initial state to {0}, please make sure the reasoner is reset and the robot is localized correctly".format(initial_state))
                self.set_initial_state([initial_state])

if __name__ == "__main__":
    rospy.init_node('challenge_demo_2014_exec')

    startup(ChallengeDemo2014)
