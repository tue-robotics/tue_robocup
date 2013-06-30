#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_final')
import rospy
import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_smach_states.util.startup import startup
import robot_smach_states.util.reasoning_helpers as urh

# ToDo: replace GetCleanup
#from speech_interpreter.srv import GetCleanup
from speech_interpreter.srv import GetYesNo
from speech_interpreter.srv import GetInfo

from robot_skills.reasoner import Conjunction, Compound, Sequence

from robot_skills.arms import State as ArmState
import geometry_msgs

import assert_operator

grasp_arm = "left"

class Ask_drink(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done" , "failed"])
        self.robot = robot
        self.get_drink_service = rospy.ServiceProxy('interpreter/get_info_user', GetInfo)
        self.person_learn_failed = 0
        self.drink_learn_failed = 0

    def execute(self, userdata=None):
        self.response = self.get_drink_service("drink_final", 3 , 120)  # This means that within 4 tries and within 60 seconds an answer is received. 
        # Check available options from rulebook!
        
        #import ipdb; ipdb.set_trace()
        self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", "coke"))))
        return "done"

class Ask_yes_no(smach.State):
    def __init__(self, robot, tracking=True):
        smach.State.__init__(self, outcomes=["yes", "preempted", "no"])

        self.robot = robot
        self.preempted = False
        self.get_yes_no_service = rospy.ServiceProxy('interpreter/get_yes_no', GetYesNo)

    def execute(self, userdata=None):

        self.response = self.get_yes_no_service(2 , 8) # 3 tries, each max 10 seconds

        if self.response.answer == "true":
            return "yes"
        else:
            return "yes" # THIS WAS "no", in this case bring sevenup to trashbin, so yes.

class ScanTablePosition(smach.State):
    def __init__(self, robot, timeout_duration):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot
        self.timeout_duration = timeout_duration

    def execute(self, gl):

        rospy.loginfo("Trying to detect tables")

        answers = self.robot.reasoner.query(Compound('region_of_interest', 
            'large_table_position', Compound('point_3d', 'X', 'Y', 'Z'), Compound('point_3d', 'Length_x', 'Length_y', 'Length_z')))
        
        ''' Remember current spindle position '''      
        spindle_pos = self.robot.spindle.get_position()

        rospy.loginfo("Timeout = {0}".format(self.timeout_duration))
        #import ipdb; ipdb.set_trace()

        if answers:
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            target_point = geometry_msgs.msg.PointStamped()
            target_point.header.frame_id = "/map"
            target_point.header.stamp = rospy.Time()
            target_point.point.x = float(answer["X"])
            target_point.point.y = float(answer["Y"])
            target_point.point.z = float(answer["Z"])

            ''' If height is feasible for LRF, use this. Else: use head and tabletop/clustering '''
            if self.robot.spindle.send_laser_goal(float(answer["Z"]), timeout=self.timeout_duration):
                self.robot.perception.toggle(["table_detector_2d"])
                #rospy.logwarn("Here we should keep track of the uncertainty, how can we do that? Now we simply use a sleep")
                rospy.loginfo("Tracking table for {0}".format(self.timeout_duration))
                self.robot.speech.speak("Hey guys, can I do anything for you?")
                rospy.sleep(rospy.Duration(self.timeout_duration))
            else:
                rospy.logerr("Can't scan on spindle height, either the spindle timeout exceeded or ROI too low. Will have to move to prior location")
            
            ''' Reset head and stop all perception stuff '''
            self.robot.perception.toggle([])
            self.robot.spindle.send_goal(spindle_pos, waittime=self.timeout_duration)
        else:
            rospy.logerr("No table location found...")

        return 'succeeded'



class ScanTables(smach.State):
    def __init__(self, robot, timeout_duration):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot
        self.timeout_duration = timeout_duration

    def execute(self, gl):

        rospy.loginfo("Trying to detect objects on tables")

        answers = self.robot.reasoner.query(Compound('region_of_interest', 
            'large_table_1', Compound('point_3d', 'X', 'Y', 'Z'), Compound('point_3d', 'Length_x', 'Length_y', 'Length_z')))
        
        ''' Remember current spindle position '''      
        spindle_pos = self.robot.spindle.get_position()


        if answers:
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base? 
            target_point = geometry_msgs.msg.PointStamped()
            target_point.header.frame_id = "/map"
            target_point.header.stamp = rospy.Time()
            target_point.point.x = float(answer["X"])
            target_point.point.y = float(answer["Y"])
            target_point.point.z = float(answer["Z"])

            ''' If height is feasible for LRF, use this. Else: use head and tabletop/clustering '''
            if self.robot.spindle.send_laser_goal(float(answer["Z"]), timeout=self.timeout_duration):
                self.robot.speech.speak("I will scan the tables for objects", block=False)
                self.robot.perception.toggle_perception_2d(target_point, answer["Length_x"], answer["Length_y"], answer["Length_z"])
                rospy.logwarn("Here we should keep track of the uncertainty, how can we do that? Now we simply use a sleep")
                rospy.logwarn("Waiting for 2.0 seconds for laser update")
                rospy.sleep(rospy.Duration(2.0))
            else:
                rospy.logerr("Can't scan on spindle height, either the spindle timeout exceeded or ROI too low. Will have to move to prior location")
            
            ''' Reset head and stop all perception stuff '''
            self.robot.perception.toggle([])
            self.robot.spindle.send_goal(spindle_pos, waittime=self.timeout_duration)
        else:
            rospy.logerr("No table location found...")

        return 'succeeded'

class DetermineGoal(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"], input_keys=['object_locations'], output_keys=['object_locations'])
        self.robot = robot
        self.preempted = False

    def execute(self, userdata):

        query = Conjunction(
                 Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")), 
                 Compound( "not", Compound("property_expected", "ObjectID", "class_label", "Class")))

        answers = self.robot.reasoner.query(query)

        self.robot.speech.speak("I have found {0} possible object locations".format(len(answers)))

        (position, orientation) = self.robot.base.get_location()
        counter = 0
        location_list = []
        for answer in answers:
            location_list.append({'X' : answer['X'], 'Y' : answer['Y'], 'Z': answer['Z']})
            location_list = sorted(location_list, key=lambda p: (position.y - float(p['Y']))**2 + (position.x - float(p['X']))**2)
        for point in location_list:
            self.robot.reasoner.assertz(Compound("goal_location", ("a" + str(counter)), Compound("point_3d", point["X"], point["Y"], point["Z"])))
            counter += 1
        
        return "done"

class ScanForPersons(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.robot = robot
        self.preempted = False

    def execute(self, userdata):
        self.robot.speech.speak("My torso laser will also find the operator", block=False)
        self.robot.perception.toggle_recognition(people=True)
        rospy.sleep(2)
        self.robot.perception.toggle_recognition()
        return "done"

class LookForServeObject(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["found", "not_found"])
        self.robot = robot
        self.preempted = False
        self.side = robot.leftArm

    def execute(self, userdata=None):
        look_at_query = Compound("base_grasp_point", "ObjectID", Compound("point_3d", "X", "Y", "Z"))
        answers = self.robot.reasoner.query(look_at_query)

        lookat_point = geometry_msgs.msg.Point()
        if answers:
            lookat_point.x = float(answers[0]["X"])
            lookat_point.y = float(answers[0]["Y"])
            lookat_point.z = float(answers[0]["Z"])
        else:
            rospy.logerr("World model is empty, while at grasp location")
            return 'not_found'

        spindle_target = max(0.15, min(lookat_point.z - 0.41, self.robot.spindle.upper_limit))
        rospy.loginfo("Target height: {0}, spindle_target: {1}".format(lookat_point.z, spindle_target))

        self.robot.head.send_goal(lookat_point, keep_tracking=True)
        self.robot.spindle.send_goal(spindle_target,waittime=5.0)

        rospy.loginfo("Start object recognition")
        self.robot.perception.toggle_recognition(objects=True)
        rospy.sleep(2.5)
        rospy.loginfo("Stop object recognition")

        self.robot.perception.toggle_recognition(objects=False)

        #Select object we are looking for
        serve_object = Compound("goal", Compound("serve", "Object"))
        answers = self.robot.reasoner.query(serve_object)
        print answers
        object_class = ""
        if answers:
            object_class = answers[0]["Object"]
            is_object_there = Conjunction(Compound("instance_of", "ObjectID", object_class),
                                        Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))
            object_query_answers = self.robot.reasoner.query(is_object_there)
            if object_query_answers:
                self.robot.speech.speak("I have found what I have been looking for, a " + str(object_class))
                self.robot.reasoner.query(Compound("retractall", Compound("base_grasp_point", "ObjectID", "A")))
                self.robot.reasoner.assertz(Compound("base_grasp_point", object_query_answers[0]['ObjectID'], Compound("point_3d", object_query_answers[0]["X"], object_query_answers[0]["Y"], object_query_answers[0]["Z"])))
                return "found"
            else:
                self.robot.speech.speak("I have not yet found what I am looking for")
                return "not_found"

        else:
            rospy.logerr("I Forgot what I have been looking for")
            return 'not_found'


class GetNextLocation(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "no_locations"])
        self.robot = robot

    def execute(self, userdata):
        self.robot.speech.speak("I will determine the next goal location")
        answers = self.robot.reasoner.query(Compound("goal_location", "Counter", Compound("point_3d", "X", "Y", "Z")))
        print ""
        print answers
        print ""
        if(answers):
            answer = answers[0]
            self.robot.reasoner.query(Compound("retract", Compound("goal_location", answer["Counter"], "A")))
            self.robot.reasoner.query(Compound("retractall", Compound("base_grasp_point", "ObjectID", "A")))
            self.robot.reasoner.assertz(Compound("base_grasp_point", "unkown", Compound("point_3d", answer["X"], answer["Y"], answer["Z"])))
            return 'done'
        else:
            self.robot.speech.speak("Ah, there are no more locations to explore")
            return 'no_locations'

class MoveToTable(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["done", "failed_navigate", "no_tables_left"])
        self.robot = robot

        with self:
            smach.StateMachine.add("GET_LOCATION", 
                GetNextLocation(self.robot),
                transitions={'done':'NAVIGATE_TO', 'no_locations':'no_tables_left'})

            smach.StateMachine.add("NAVIGATE_TO", states.NavigateGeneric(robot, 
                lookat_query=Compound("base_grasp_point", "ObjectID", Compound("point_3d", "X", "Y", "Z"))), 
                transitions={'unreachable' : 'failed_navigate', 'preempted' : 'NAVIGATE_TO', 
                'arrived' : 'done', 'goal_not_defined' : 'failed_navigate'})


class PersonOrPrior(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["at_prior", "at_person", "failed"])
        self.robot = robot
        self.query_human = Conjunction(Compound("instance_of", "ID", "person"), 
            Compound("property_expected", "ID", "position", Sequence("X", "Y", "Z")))
        self.query_roi = Compound('region_of_interest', 'people_area', Compound('point_3d', 'X', 'Y', 'Z'), Compound('point_3d', 'Length_x', 'Length_y', 'Length_z'))

    def execute(self, userdata=None):
        #Check if person is available, else scan at prior
        humans = self.robot.reasoner.query(self.query_human)
        roi = self.robot.reasoner.query(self.query_roi)
        
        def in_roi(answerdict):
            """answerdict is a dict with answrs to Compound("property_expected", "ID", "position", Sequence("X", "Y", "Z")) 
            rx etc are the corner of the ROI and rlx are the dimensions"""
            #import ipdb; ipdb.set_trace()
            hx,hy,hz = float(answerdict["X"]), float(answerdict["Y"]),  float(answerdict["Z"])
            rx,ry,rz = float(roi[0]["X"]),  float(roi[0]["Y"]), float(roi[0]["Z"])
            rlx, rly, rlz = float(roi[0]["Length_x"]),  float(roi[0]["Length_y"]),  float(roi[0]["Length_z"])

            x_ok = rx-(rlx/2) < hx < rx+(rlx/2)
            y_ok = ry-(rly/2) < hy < ry+(rly/2)

            ok = x_ok and y_ok
            rospy.loginfo("{0} is {1}".format(answerdict, ok))
            return ok

        #import ipdb; ipdb.set_trace()
        #for debugging: humans = [{'Y': '-2.78225655853', 'X': '8.38399997379', 'Z': '1.02274683455', 'ID': '0a73f86d5e4a99b18910fb86f20a8149'}]
        try:
            humans_in_roi = filter(in_roi, humans)
        except IndexError:
            humans_in_roi = []

        if humans_in_roi:
            self.robot.speech.speak("I am going to return the object to a person!")
            self.robot.reasoner.query(Compound("retractall", Compound("deliver_goal", "A")))
            self.robot.reasoner.assertz(Compound("deliver_goal", Compound("point_3d", float(humans_in_roi[0]['X']) - 0.5, humans_in_roi[0]['Y'], humans_in_roi[0]['Z'])))
            return 'at_person'
        else:
            query_prior = Compound("waypoint", "prior",  Compound("pose_2d", "X", "Y", "Phi"))
            answers_prior = self.robot.reasoner.query(query_prior)
            if answers_prior:
                self.robot.speech.speak("I have not found a person yet, but I will try at a prior location")
                rospy.loginfo("Prior pose: X: {0}, Y: {1}, Phi: {2}".format(float(answers_prior[0]['X']), float(answers_prior[0]['Y']), float(answers_prior[0]['Phi'])))
                #self.robot.reasoner.query(Compound("retractall", Compound("deliver_pose", "A")))
                return 'at_prior'
            else:
                self.robot.speech.speak("I don't know of any persons or prior locations")
                return 'failed'

class MoveToGoal(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=["succeeded_person" ,"succeeded_prior", "failed"])
        self.robot = robot
        with self:
            smach.StateMachine.add("PERSON_OR_PRIOR",
                PersonOrPrior(self.robot),
                transitions={'at_prior': 'NAVIGATE_TO_PRIOR', 'at_person' : 'NAVIGATE_TO_PERSON', 'failed': 'failed'})
            smach.StateMachine.add("NAVIGATE_TO_PERSON", states.NavigateGeneric(robot, 
                lookat_query=Compound("deliver_goal", Compound("point_3d", "X", "Y", "Z"))), 
                transitions={'unreachable' : 'failed', 'preempted' : 'NAVIGATE_TO_PRIOR', 
                'arrived' : 'succeeded_person', 'goal_not_defined' : 'failed'})
            smach.StateMachine.add("NAVIGATE_TO_PRIOR", states.NavigateGeneric(robot, 
                goal_query=Compound("waypoint", "prior",  Compound("pose_2d", "X", "Y", "Phi"))), 
                transitions={'unreachable' : 'failed', 'preempted' : 'NAVIGATE_TO_PERSON', 
                'arrived' : 'succeeded_prior', 'goal_not_defined' : 'failed'})

class AskGraspObject(smach.StateMachine):
    def __init__(self, robot, side):
        smach.StateMachine.__init__(self, outcomes=["done"])
        self.robot = robot
        self.side = side
        with self:
            smach.StateMachine.add("SAY_FAIL", states.Say(robot, ["Unable to grasp please insert the object into the gripper"]),
                transitions={   'spoken':'done'})
            smach.StateMachine.add("OPEN_GRIPPER", states.SetGripper(self.robot, self.side, gripperstate=ArmState.OPEN),
                transitions={   'succeeded':'CLOSE_GRIPPER', 'failed': 'done'})
            smach.StateMachine.add("CLOSE_GRIPPER", states.SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE),
                transitions={   'succeeded':'CLOSE_GRIPPER', 'failed': 'done'})


class DecideAction(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['known_object', 'unkown_object', 'finished'])
        self.robot = robot
        self.second_run = False
    def execute(self, userdata=None):
        #Check if finished
        answers = self.robot.reasoner.query(Compound("goal", Compound("serve","Class")))
        #Second run
        if answers:
            #Check if already known
            is_object_there = self.robot.reasoner.query(Conjunction(Compound("instance_of", "ObjectID", answers[0]['Class']),
                                        Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z"))))
            if is_object_there:
                self.robot.speech.speak("I know where the {0}, is located".format(str(answers[0]['Class'])))
                self.robot.reasoner.query(Compound("retractall", Compound("base_grasp_point", "ObjectID", "X")))
                self.robot.reasoner.assertz(Compound("base_grasp_point", is_object_there[0]['ObjectID'], Compound("point_3d", is_object_there[0]['X'], is_object_there[0]['Y'], is_object_there[0]['Z'])))

                #Retract grasped object
                self.robot.reasoner.query(Compound("retractall", Compound("goal", Compound("serve", "Class"))))
                return 'known_object'
            else:
                self.robot.speech.speak("I do not know what the other object is, but I better go and get it".format(str(answers[0]['Class'])))
                return 'unkown_object'
        else:
            self.robot.speech.speak("The table is clean, let's go home")
            return 'finished'

class HandoverToKnownHuman(smach.StateMachine):
    def __init__(self, robot, side):
        smach.StateMachine.__init__(self, outcomes=["done"])
        self.arm = side
        self.robot = robot
        with self:
            smach.StateMachine.add( 'PRESENT_DRINK',
                                    states.Say(self.robot, ["I'm going to hand over your drink now", "Here you go! Handing over your drink"],block=False),
                                    transitions={"spoken":"POSE"})

            smach.StateMachine.add( 'POSE',
                                    states.Handover_pose(self.arm, self.robot),
                                    transitions={   'succeeded':'PLEASE_TAKE',
                                                    'failed':'PLEASE_TAKE'})
            
            smach.StateMachine.add( 'PLEASE_TAKE',
                                    states.Say(self.robot, ["Please hold the drink, I'm going to let it go.", "Please take the drink, I'll let it go"]),
                                    transitions={"spoken":"OPEN_GRIPPER"})

            smach.StateMachine.add( "OPEN_GRIPPER", 
                                    states.SetGripper(self.robot, self.arm, gripperstate=0, drop_from_frame="/grippoint_left"), #open
                                    transitions={   'succeeded':'CLOSE_AFTER_DROP',
                                                    'failed':'CLOSE_AFTER_DROP'})
            smach.StateMachine.add( 'CLOSE_AFTER_DROP',
                                    states.SetGripper(self.robot, self.arm, gripperstate=1), #close
                                    transitions={   'succeeded':'RESET_ARM',
                                                    'failed':'RESET_ARM'})
            smach.StateMachine.add('RESET_ARM', 
                                    states.ArmToPose(self.robot, self.arm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), 
                                    transitions={   'done':'RESET_TORSO',
                                                    'failed':'RESET_TORSO'})
            smach.StateMachine.add('RESET_TORSO',
                                    states.ResetTorso(self.robot),
                                    transitions={   'succeeded':'SAY_ENJOY',
                                                    'failed'   :'SAY_ENJOY'})

            smach.StateMachine.add( 'SAY_ENJOY',
                                    states.Say(self.robot, ["Enjoy your drink!", "I hope your thirsty, enjoy!"]),
                                    transitions={"spoken":"done"})

class ToggleBinDetector(smach.State):
    '''Enables or disables bin detector '''
    def __init__(self, robot, roi_query=None, length_x=3.0, length_y=3.0, length_z=1.0, timeout=2.0):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.roi_query = roi_query
        self.length_x = length_x
        self.length_y = length_y
        self.length_z = length_z
        self.timeout = timeout

    def execute(self, userdata=None):

        poi = geometry_msgs.msg.PointStamped()

        '''Enabling bin detection '''
        answers = self.robot.reasoner.query(self.roi_query)
        #rospy.loginfo('Answers = {0}'.format(answers))
        #import ipdb
        #ipdb.set_trace()

        if answers:
            answer = answers[0]
            rospy.loginfo("Answer(0) = {0}".format(answer))
            
            poi.header.frame_id = "/map"
            poi.header.stamp = rospy.Time()
            poi.x = float(answer["X"])
            poi.point.y = float(answer["Y"])
            poi.z = float(answer["Z"])
            #rospy.loginfo("Grasp_point = {0}".format(self.grasp_point))
            response = self.robot.perception.toggle_bin_detection(poi, length_x=self.length_x, length_y=self.length_y, length_z=self.length_z)
            if not response.suc:
                return 'failed'
        else: 
            return 'failed'

        rospy.sleep(rospy.Duration(self.timeout))

        '''Disabling bin detection with negative size'''
        self.robot.perception.toggle_bin_detection(poi, length_x=-1.0, length_y=-1.0, length_z=-1.0)
        return 'succeeded'

def setup_statemachine(robot):
    side = robot.leftArm
    
    robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
    robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
    
    #Assert the current challenge.
    robot.reasoner.assertz(Compound("challenge", "final"))
    robot.reasoner.query(Compound("retractall", Compound("goal_location", "X", "Y")))
    robot.reasoner.query(Compound("retractall", Compound("goal", Compound("serve", "Counter", "X"))))
    robot.reasoner.query(Compound("retractall", Compound("goal", Compound("bringTo", "X"))))
    robot.reasoner.query(Compound("retractall", Compound("base_grasp_point", "ObjectID", "X")))
    robot.reasoner.query(Compound("retractall", Compound("deliver_pose", "A")))
    robot.reasoner.query(Compound("retractall", Compound("deliver_goal", "A")))
   

    query_living_room1 = Compound("waypoint", "living_room1", Compound("pose_2d", "X", "Y", "Phi"))
    query_living_room2 = Compound("waypoint", "living_room2", Compound("pose_2d", "X", "Y", "Phi"))
    query_living_room3 = Compound("waypoint", "living_room3", Compound("pose_2d", "X", "Y", "Phi"))
    query_kitchen1 = Compound("waypoint", "kitchen1", Compound("pose_2d", "X", "Y", "Phi"))
    query_kitchen2 = Compound("waypoint", "kitchen2", Compound("pose_2d", "X", "Y", "Phi"))
    query_trash_bin = Compound("dropoff_point", "trash_bin", Compound("point_3d", "X", "Y", "Z"))

    if grasp_arm == "right": 
        arm = robot.rightArm
    else:
        arm = robot.leftArm

#    query_pose = robot.reasoner.base_pose(Compound("initial_open_challenge", robot.reasoner.pose_2d("X", "Y", "Phi")))
#    print query_pose

#    answers = robot.reasoner.query(query_pose)
#    print answers
#    initial_pose = (answers[0]["X"], answers[0]["Y"], answers[0]["Phi"])

    #query = Compound('region_of_interest', 'tables') #region_of_interest(rgo2013, open_challenge, tables, point_3d(0, 0, 0), point_3d(2, 2, 2))
    #robot.reasoner.query(query)

    #answers = robot.reasoner.query(Compound("template_matching_config", "Config"))
    #robot.perception.load_template_matching_config(answers[0]["Config"])

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    sm.userdata.object_locations = []
    sm.userdata.current_location = None

    with sm:

        smach.StateMachine.add( "START_CHALLENGE",
                                    states.StartChallengeRobust(robot, "initial"), 
                                    transitions={   "Done":"SAY_GOTO_LIVINGROOM", 
                                                    "Aborted":"SAY_GOTO_LIVINGROOM", 
                                                    "Failed":"SAY_GOTO_LIVINGROOM"}) 

        smach.StateMachine.add("SAY_GOTO_LIVINGROOM",
                                    states.Say(robot, "I will go to the living room, see if I can do something over there.", block=False),
                                    transitions={   "spoken":"GOTO_LIVING_ROOM1"})

        smach.StateMachine.add('GOTO_LIVING_ROOM1',
                                    states.NavigateGeneric(robot, goal_query = query_living_room1),
                                    transitions={   "arrived":"SCAN_TABLE_POSITION", 
                                                    "unreachable":"GOTO_LIVING_ROOM2", 
                                                    "preempted":"GOTO_LIVING_ROOM2", 
                                                    "goal_not_defined":"GOTO_LIVING_ROOM2"})

        smach.StateMachine.add('GOTO_LIVING_ROOM2',
                                    states.NavigateGeneric(robot, goal_query = query_living_room2),
                                    transitions={   "arrived":"SCAN_TABLE_POSITION", 
                                                    "unreachable":"GOTO_LIVING_ROOM3", 
                                                    "preempted":"GOTO_LIVING_ROOM3", 
                                                    "goal_not_defined":"GOTO_LIVING_ROOM3"})

        smach.StateMachine.add('GOTO_LIVING_ROOM3',
                                    states.NavigateGeneric(robot, goal_query = query_living_room3),
                                    transitions={   "arrived":"SCAN_TABLE_POSITION", 
                                                    "unreachable":"SCAN_TABLE_POSITION", 
                                                    "preempted":"SCAN_TABLE_POSITION", 
                                                    "goal_not_defined":"SCAN_TABLE_POSITION"})
            
        smach.StateMachine.add("SCAN_TABLE_POSITION", 
                                ScanTablePosition(robot, 20.0),
                                transitions={   'succeeded':'TAKE_ORDER'})

        smach.StateMachine.add( "TAKE_ORDER",
                                            Ask_drink(robot),
                                            transitions={   "done":"ASSERT_CURRENT_OPERATOR",
                                                            "failed":"ASSERT_CURRENT_OPERATOR"})

        smach.StateMachine.add( "ASSERT_CURRENT_OPERATOR",
                                assert_operator.AssertCurrentOperator(robot),
                                transitions={"asserted":"SCAN_TABLES",
                                             "no_operator":"SCAN_TABLES"})

        # After this state: objects might be in the world model
        smach.StateMachine.add("SCAN_TABLES", 
                        ScanTables(robot, 10.0),
                        transitions={   'succeeded':'DETERMINE_GOAL'})

        # # After this state: persons might be in the world model
        # smach.StateMachine.add("SCAN_FOR_PERSONS", 
        #                 ScanForPersons(robot),
        #                 transitions={   'done':'DETERMINE_GOAL', 'failed': 'DETERMINE_GOAL'})
        
        # After this state: persons might be in the world model
        smach.StateMachine.add("DETERMINE_GOAL", 
                        DetermineGoal(robot),
                        transitions={   'done':'MOVE_TO_TABLE'})

        #Scan for persons at the prior location
        # smach.StateMachine.add("SCAN_FOR_PERSONS_AT_PRIOR", 
        #                 ScanForPersons(robot),
        #                 transitions={ 'done':'MOVE_TO_GOAL_AFTER_PRIOR', 'failed':'ASK_GET_OBJECT'})

        smach.StateMachine.add("MOVE_TO_TABLE", 
                MoveToTable(robot),
                transitions={   'done':'RECOGNIZE_OBJECTS', 'failed_navigate' : 'MOVE_TO_TABLE', 'no_tables_left' : 'RETURN_LIVING_ROOM'})

        # STATE RECOGNIZE_OBJECTS: recognize objects on the table

        smach.StateMachine.add("RECOGNIZE_OBJECTS", 
                LookForServeObject(robot), # En andere dingen
                transitions={  'not_found':'MOVE_TO_TABLE', 'found': 'GRAB'})

        smach.StateMachine.add("GRAB", 
            states.GrabMachine(side, robot, Compound("base_grasp_point", "ObjectID", Compound("point_3d", "X", "Y", "Z"))), # En andere dingen
            transitions={   'succeeded':'MOVE_TO_GOAL', 'failed':'ASK_GRASP_OBJECT'})  

        smach.StateMachine.add("ASK_GRASP_OBJECT",
            AskGraspObject(robot, side),
            transitions={'done': 'MOVE_TO_GOAL'})

        # WITH OBJECT
        # STATE: arrive at a person: hand over object and go back to starting position
        smach.StateMachine.add("MOVE_TO_GOAL",                                                              # TODO SJOERD: in person_or_prior set query to person.
            MoveToGoal(robot), # En andere dingen
            transitions={   'succeeded_person':'SAY_HANDOVER', 
                            'succeeded_prior':'SAY_HANDOVER', 
                            'failed':'SAY_CANNOT_FIND_OPERATOR'})

        # smach.StateMachine.add("MOVE_TO_GOAL_AFTER_PRIOR", 
        #     MoveToGoal(robot), # En andere dingen
        #     transitions={   'succeeded_person':'SAY_HANDOVER', 
        #                     'succeeded_prior':'HANDOVER_PRIOR', 
        #                     'failed':'SAY_CANNOT_FIND_OPERATOR'})

        #Failed to deliver ask call for help
        smach.StateMachine.add("SAY_CANNOT_FIND_OPERATOR", 
            states.Say(robot, ["I am terribly sorry but I cannot reach you, can you please come to me?"]), 
            transitions={   'spoken':'SAY_HANDOVER'}) 

        # ToDo: Make state that waits for operator in front of him

        #Handover the object
        smach.StateMachine.add("SAY_HANDOVER", 
           states.Say(robot, ["Here is your order. Please take it from my gripper"]), 
           transitions={   'spoken':'OPEN_GRIPPER' })

        #Open gripper to release object
        smach.StateMachine.add("OPEN_GRIPPER", 
            states.SetGripper(robot, side, gripperstate=ArmState.OPEN),
            transitions={   'succeeded':'DECIDE_ACTION', 'failed': 'DECIDE_ACTION'})

        smach.StateMachine.add("DECIDE_ACTION",
            DecideAction(robot),
            transitions={'known_object': 'GRAB_SECOND_ITEM', 'unkown_object' : 'DETERMINE_GOAL', 'finished' : 'RETURN_LIVING_ROOM'})

        smach.StateMachine.add("GRAB_SECOND_ITEM", 
            states.GrabMachine(side, robot, Compound("base_grasp_point", "ObjectID", Compound("point_3d", "X", "Y", "Z"))), # En andere dingen
            transitions={   'succeeded':'MOVE_TO_GOAL', 'failed':'ASK_GRASP_OBJECT'})

        smach.StateMachine.add("SAY_GOTO_KITCHEN",
                                    states.Say(robot, "Let's throw this in the trashbin", block=False),
                                    transitions={   "spoken":"GOTO_KITCHEN"})

        smach.StateMachine.add('GOTO_KITCHEN',
                                    states.NavigateGeneric(robot, goal_query = query_kitchen1),
                                    transitions={   "arrived":"SCAN_TABLE_POSITION", 
                                                    "unreachable":"GOTO_KITCHEN2", 
                                                    "preempted":"GOTO_KITCHEN2", 
                                                    "goal_not_defined":"GOTO_KITCHEN2"})

        smach.StateMachine.add('GOTO_KITCHEN2',
                                    states.NavigateGeneric(robot, goal_query = query_kitchen2),
                                    transitions={   "arrived":"SCAN_TABLE_POSITION", 
                                                    "unreachable":"SAY_CANNOT_REACH_KITCHEN", 
                                                    "preempted":"SAY_CANNOT_REACH_KITCHEN", 
                                                    "goal_not_defined":"SAY_CANNOT_REACH_KITCHEN"})

        smach.StateMachine.add('RETURN_LIVING_ROOM',
                                    states.NavigateGeneric(robot, goal_query = query_living_room1),
                                    transitions={   "arrived":"SAY_THANKS", 
                                                    "unreachable":"RETURN_LIVING_ROOM2", 
                                                    "preempted":"RETURN_LIVING_ROOM2", 
                                                    "goal_not_defined":"RETURN_LIVING_ROOM2"})

        smach.StateMachine.add('RETURN_LIVING_ROOM2',
                                    states.NavigateGeneric(robot, goal_query = query_living_room2),
                                    transitions={   "arrived":"SAY_THANKS", 
                                                    "unreachable":"RETURN_LIVING_ROOM3", 
                                                    "preempted":"RETURN_LIVING_ROOM3", 
                                                    "goal_not_defined":"RETURN_LIVING_ROOM3"})

        smach.StateMachine.add('RETURN_LIVING_ROOM3',
                                    states.NavigateGeneric(robot, goal_query = query_living_room3),
                                    transitions={   "arrived":"SAY_THANKS", 
                                                    "unreachable":"SAY_THANKS", 
                                                    "preempted":"SAY_THANKS", 
                                                    "goal_not_defined":"SAY_THANKS"})

        smach.StateMachine.add('SCAN_BIN_POSITION',
                                    ToggleBinDetector(robot, roi_query=None, length_x=3.0, length_y=3.0, length_z=1.0, timeout=2.0),
                                    transitions={   "succeeded" : "DROPOFF_BIN",
                                                    "failed"    : "DROPOFF_BIN"})

        smach.StateMachine.add("DROPOFF_BIN",
                                    states.DropObject(arm, robot, query_trash_bin),
                                    transitions={   'succeeded':'RETURN_LIVING_ROOM',
                                                    'failed':'RETURN_LIVING_ROOM',
                                                    'target_lost':'SAY_CANNOT_DISPOSE'})

        smach.StateMachine.add('SAY_CANNOT_DISPOSE',
                                    states.Say(robot, "I am sorry, but i cannot throw the object in the trash bin",mood="sad",block=False),
                                    transitions={'spoken':'RETURN_LIVING_ROOM'})

            # TODO DRIVE TO AUDIENCE NEAR COUCH.

        smach.StateMachine.add('SAY_CANNOT_REACH_KITCHEN',
                                    states.Say(robot, "I cannot seem to go to throw this in the thrasbin, i am terribly sorry",mood="sad",block=False),
                                    transitions={'spoken':'RETURN_LIVING_ROOM'})

        smach.StateMachine.add('SAY_LASER_ERROR',
                                    states.Say(robot, "Something went terribly wrong, can I start again",mood="sad",block=False),
                                    transitions={'spoken':'EXIT'})

        smach.StateMachine.add('SAY_THANKS',
                                    states.Say(robot, "Thanks for your time, hope you enjoyed Robocup 2013."),
                                    transitions={'spoken':'EXIT'}) 

        smach.StateMachine.add('EXIT',
                                    states.NavigateGeneric(robot, goal_query = Compound("waypoint","exit_1",Compound("pose_2d","X","Y","Phi"))),
                                    transitions={   "arrived":"FINISH", 
                                                    "unreachable":"CLEAR_PATH_TO_EXIT", 
                                                    "preempted":"CLEAR_PATH_TO_EXIT", 
                                                    "goal_not_defined":"CLEAR_PATH_TO_EXIT"})

        smach.StateMachine.add('CLEAR_PATH_TO_EXIT',
                                    states.Say(robot, "I couldn't go to the exit. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_EXIT_SECOND_TRY'}) 

        smach.StateMachine.add('GO_TO_EXIT_SECOND_TRY',
                                    states.NavigateGeneric(robot, goal_query = Compound("waypoint","exit_2",Compound("pose_2d","X","Y","Phi"))),
                                    transitions={   "arrived":"FINISH", 
                                                    "unreachable":"FINISH", 
                                                    "preempted":"FINISH", 
                                                    "goal_not_defined":"FINISH"})

        smach.StateMachine.add( 'FINISH', 
                                    states.Finish(robot),
                                    transitions={'stop':'Done'})


    return sm


if __name__ == "__main__":
    rospy.init_node('final_challenge_executive')
    startup(setup_statemachine)

