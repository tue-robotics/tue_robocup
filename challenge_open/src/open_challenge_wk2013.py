#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_open')
import rospy
import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_smach_states.util.startup import startup
import robot_smach_states.util.reasoning_helpers as urh

# ToDo: replace GetCleanup
#from speech_interpreter.srv import GetCleanup
from speech_interpreter.srv import GetOpenChallenge
from speech_interpreter.srv import GetYesNo

from robot_skills.reasoner import Conjunction, Compound, Sequence

from robot_skills.arms import State as ArmState
import geometry_msgs

class AskForTask(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.preempted = False
        #self.get_goto_service = rospy.ServiceProxy('interpreter/get_open_challenge', GetOpenChallenge)

    def execute(self, userdata):
        self.robot.reasoner.query(Compound("retractall", Compound("goal", Compound("open_challenge", "X"))))

        # TODO: Here an service that has to be created has to be called
        #self.response = self.get_goto_service(4 , 60)  # This means that within 4 tries and within 60 seconds an answer is received.

        # Default values
        obj = "coke"
        location = "dinner_table"
        '''
        if self.response.answer == "bringacoketothekitchen":
            location = "dinner_table"
            obj = "coke"
        elif self.response.answer == "bringaspritetothekitchen":
            location = "dinner_table"
            obj = "sprite"
        '''

        #self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("open_challenge", location))))
        self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("bringTo", location))))
        self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", obj))))

            
        return "done"


class ScanTables(smach.State):
    def __init__(self, robot, timeout_duration):
        smach.State.__init__(self, outcomes=['succeeded'])
        #self.kwerie = 
        self.robot = robot
        self.timeout_duration = timeout_duration

    def execute(self, gl):

        rospy.loginfo("Trying to detect objects on tables")

        answers = self.robot.reasoner.query(Compound('region_of_interest', 
            'desk_1', Compound('point_3d', 'X', 'Y', 'Z'), Compound('point_3d', 'Length_x', 'Length_y', 'Length_z')))
        
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
            rospy.logwarn("Spindle timeout temporarily increased to 30 seconds")
            if self.robot.spindle.send_laser_goal(float(answer["Z"]), timeout=self.timeout_duration):
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

        counter = 0
        for answer in answers:
            self.robot.reasoner.assertz(Compound("goal_location", ("a" + str(counter)), Compound("point_3d", answer["X"], answer["Y"], answer["Z"])))
            counter += 1
        
        return "done"

class ScanForPersons(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])
        self.robot = robot
        self.preempted = False

    def execute(self, userdata):
        
        # use existing state to turn on laser_ppl_detection
        # pause for 0.5 [s]
        # use existing state to turn off laser_ppl_detection
            
        return "done"

class LookForServeObject(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["found", "not_found"])
        self.robot = robot
        self.preempted = False
        self.side = robot.leftArm

    def execute(self, userdata=None):
        look_at_query = Compound("base_grasp_point", Compound("point_3d", "X", "Y", "Z"))
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

        object_class = ""
        if answers:
            object_class = answers[0]["Object"]
            is_object_there = Conjunction(Compound("instance_of", "ObjectID", object_class),
                                        Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

            object_query_answers = self.robot.reasoner.query(is_object_there)
            if object_query_answers:
                self.robot.speech.speak("I have found what I was looking for, a " + str(object_class))
                self.robot.reasoner.retractall(Compound("base_grasp_point", "A"))
                self.robot.reasoner.assertz(Compound("base_grasp_point", Compound("point_3d", object_query_answers[0]["X"], object_query_answers[0]["Y"], object_query_answers[0]["Z"])))
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
        answers = self.robot.reasoner.query(Compound("goal_location", "Counter" , Compound("point_3d", "X", "Y", "Z")))

        print ""
        print answers
        print ""
        if(answers):
            answer = answers[0]
            self.robot.reasoner.query(Compound("retract", answer["Counter"], "A"))
            self.robot.reasoner.retractall(Compound("base_grasp_point", "A"))
            self.robot.reasoner.assertz(Compound("base_grasp_point", Compound("point_3d", answer["X"], answer["Y"], answer["Z"])))
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
                lookat_query=Compound("base_grasp_point", Compound("point_3d", "X", "Y", "Z"))), 
                transitions={'unreachable' : 'failed_navigate', 'preempted' : 'NAVIGATE_TO', 
                'arrived' : 'done', 'goal_not_defined' : 'failed_navigate'})


class MoveToGoal(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["succeeded_person" ,"succeeded_prior", "failed"])
        pass
    def execute(self, userdata=None):
        pass



def setup_statemachine(robot):
    side = robot.leftArm
    
    robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/locations.pl'))
    robot.reasoner.query(Compound("load_database", "tue_knowledge", 'prolog/objects.pl'))
    
    #Assert the current challenge.
    robot.reasoner.assertz(Compound("challenge", "open_challenge"))
    robot.reasoner.query(Compound("retractall", Compound("goal_location", "A")))

#    query_pose = robot.reasoner.base_pose(Compound("initial_open_challenge", robot.reasoner.pose_2d("X", "Y", "Phi")))
#    print query_pose

#    answers = robot.reasoner.query(query_pose)
#    print answers
#    initial_pose = (answers[0]["X"], answers[0]["Y"], answers[0]["Phi"])

    query = Compound('region_of_interest', 'tables') #region_of_interest(rgo2013, open_challenge, tables, point_3d(0, 0, 0), point_3d(2, 2, 2))
    robot.reasoner.query(query)


    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    sm.userdata.object_locations = []
    sm.userdata.current_location = None

    with sm:

        smach.StateMachine.add('INITIALIZE',
                        states.Initialize(robot),
                        transitions={'initialized':'INIT_POSE',
                                     'abort':'Aborted'})
        
        smach.StateMachine.add('INIT_POSE',
                        states.Set_initial_pose(robot, "initial"),
                        transitions={   'done':'SAY_START',
                                        'preempted':'Aborted',
                                        'error':'Aborted'})

        smach.StateMachine.add("SAY_START", 
                        states.Say(robot, ["Hi I am Amigo, what do you want me to do?"]),
                        transitions={   'spoken':'ASK_FOR_TASK'})

        # After this state: an object and location are guaranteed to be defined
        smach.StateMachine.add("ASK_FOR_TASK", 
                        AskForTask(robot),
                        transitions={   'done':'SCAN_TABLES'})

        # After this state: objects might be in the world model
        smach.StateMachine.add("SCAN_TABLES", 
                        ScanTables(robot, 2.0),
                        transitions={   'succeeded':'SCAN_FOR_PERSONS'})

        # After this state: persons might be in the world model
        smach.StateMachine.add("SCAN_FOR_PERSONS", 
                        ScanForPersons(robot),
                        transitions={   'done':'DETERMINE_GOAL', 'failed': 'DETERMINE_GOAL'})
        
        # After this state: persons might be in the world model
        smach.StateMachine.add("DETERMINE_GOAL", 
                        DetermineGoal(robot),
                        transitions={   'done':'MOVE_TO_TABLE'})

        #Scan for persons at the prior location
        smach.StateMachine.add("SCAN_FOR_PERSONS_AT_PRIOR", 
                        ScanForPersons(robot),
                        transitions={   'done':'MOVE_TO_GOAL', 'failed':'ASK_GET_OBJECT'})

        # STATE DETERMINE_GOAL: if world model contains objects
        #  if yes: move towards closest object
        #  else: move towards prior position

        smach.StateMachine.add("MOVE_TO_TABLE", 
                MoveToTable(robot),
                transitions={   'done':'RECOGNIZE_OBJECTS', 'failed_navigate' : 'MOVE_TO_TABLE', 'no_tables_left' : 'GO_HOME'})

        # STATE RECOGNIZE_OBJECTS: recognize objects on the table

        smach.StateMachine.add("RECOGNIZE_OBJECTS", 
                LookForServeObject(robot), # En andere dingen
                transitions={  'not_found':'MOVE_TO_TABLE', 'found': 'GRAB'})
        


        # STATE CHECK_OBJECTS_ON_TABLE: see if the object is the requested object
        #  if yes: grasp (with standard fallback scenarios)
        #    if person in world model: move to person
        #    else: move to prior position
        #  else:
        #    if other object in world model: go there
        #    else: move to prior position 

       #  smach.StateMachine.add("LOOK_FOR_SERVE_OBJECT", 
       #     LookForServeObject(robot), # En andere dingen
       #     transitions={   'found':'GRAB', 'not_found':'MOVE_TO_TABLE'})    

        smach.StateMachine.add("GRAB", 
            states.GrabMachine(side, robot, Compound("base_grasp_point", Compound("point_3d", "X", "Y", "Z"))), # En andere dingen
            transitions={   'succeeded':'MOVE_TO_GOAL', 'failed':'GO_HOME'})  

        # WITH OBJECT
        # STATE: arrive at a person: hand over object and go back to starting position
        smach.StateMachine.add("MOVE_TO_GOAL", 
            MoveToGoal(robot), # En andere dingen
            transitions={   'succeeded_person':'HANDOVER', 
                            'succeeded_prior':'SCAN_FOR_PERSONS_AT_PRIOR', 'failed':'ASK_GET_OBJECT'})

        #Handover the object
        smach.StateMachine.add("HANDOVER", 
            states.Say(robot, ["Here is your order. Please take it from my gripper"]), # En andere dingen
            transitions={   'spoken':'OPEN_GRIPPER' })


        #Open gripper to release object
        smach.StateMachine.add("OPEN_GRIPPER", 
            states.SetGripper(robot, side, gripperstate=ArmState.OPEN), # En andere dingen
            transitions={   'succeeded':'GO_HOME', 'failed': 'GO_HOME'})

        #Open gripper to release object
        smach.StateMachine.add("GO_HOME", 
            states.NavigateGeneric(robot, goal_pose_2d=None), # En andere dingen
            transitions={   'unreachable' : 'SAY_ERROR', 'preempted' : 'GO_HOME', 'arrived' : 'Done', 'goal_not_defined' : 'SAY_ERROR'})

        smach.StateMachine.add("SAY_ERROR", 
            states.Say(robot, ["Cannot reach my goal, end of challenge!"]), # En andere dingen
            transitions={   'spoken':'Done' })


        #Failed to deliver ask call for help
        smach.StateMachine.add("ASK_GET_OBJECT", 
            states.Say(robot, ["Here is your order. Please take it from my gripper"]), # En andere dingen
            transitions={   'spoken':'OPEN_GRIPPER'}) 

        smach.StateMachine.add("NEVER_REACHED", 
            states.Say(robot, ["Existing states never return to this state. Someone changed something, reconsider open challenge transitions"]), # En andere dingen
            transitions={   'spoken':'Done' })  



        
 

        # WITH OBJECT
        # STATE: no path to person, drive to prior position array (prior position must be available)

        # WITH OBJECT
        # STATE: arrive at prior position: scan for a person
        #   if person in world model: move to person
        #   else: I cannot find the person, can you please get the object

        # SECOND OBJECT SHOULD REUSE PREVIOUS STATES

        # STATE: object delivered. go back to starting position

        # STATE: at starting position, ask for second task ('bring me the <other-object>')

        # STATE: check world model for this object, must be present
        #   if present: go get it using grasping pipeline and previous states
        #   else: object not present, say 'I am sorry, that would take too much time'




    return sm


if __name__ == "__main__":
    rospy.init_node('open_challenge_executive')
    startup(setup_statemachine)

