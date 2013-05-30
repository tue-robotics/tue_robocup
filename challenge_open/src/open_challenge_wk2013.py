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

from robot_skills.reasoner import Conjunction, Compound

from robot_skills.arms import State as ArmState

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
        self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("bring", location))))
        self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", obj))))

            
        return "done"


class ScanTables(smach.State):
    def __init__(self, robot, timeout_duration):
        smach.State.__init__(self, outcomes=['succeeded'])
        #self.kwerie = 
        self.robot = robot

    def execute(self, gl):

        rospy.loginfo("Trying to detect objects on tables")

        ''' Remember current spindle position '''
        answers = self.robot.reasoner.query(self.grabpoint_query)

        answers = robot.reasoner.query(Compound())
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
        if self.robot.spindle.send_laser_goal(float(answer["Z"]), timeout=timeout_duration):
            self.robot.perception.toggle_perception_2d(target_point, answer["length_x"], answer["length_y"], answer["length_z"])
            rospy.logwarn("Here we should keep track of the uncertainty, how can we do that? Now we simply use a sleep")
            rospy.logwarn("Waiting for 2.0 seconds for laser update")
            rospy.sleep(rospy.Duration(2.0))
        else:
            rospy.logerror("Can't scan on spindle height, either the spindle timeout exceeded or ROI too low. Will have to move to prior location")

        ''' Reset head and stop all perception stuff '''
        self.robot.perception.toggle([])
        self.robot.spindle.send_goal(spindle_pos,waittime=timeout_duration)

        return 'succeeded'

'''class ScanTables(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.preempted = False

    def execute(self, userdata):
        
        # get table rois from reasoner
        # use existing state to turn on object_detector_2d
        # pause for 0.5 [s]
        # use existing state to turn off object_detector_2d
            
        return "done"'''

class DetermineGoal(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.preempted = False

    def execute(self, userdata):

        object_locations = self.robot.reasoner.query( Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")),
                                                    Compound( "not", Compound("property_expected", "ObjectID", "class_label", "Class")))
        robot.reasoner.query(object_locations)
        userdata.object_locations = []
        userdata.object_locations.append({})
        # get table rois from reasoner
        # use existing state to turn on object_detector_2d
        # pause for 0.5 [s]
        # use existing state to turn off object_detector_2d
            
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

        #Find object in wm, look for correct classlabel
        # if true: return found
        # else: return not_found
            
        return "found"

class MoveToTable(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed_navigate", "no_tables_left"])
        pass
    def execute(self, userdata=None):
        pass

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

#    query_pose = robot.reasoner.base_pose(Compound("initial_open_challenge", robot.reasoner.pose_2d("X", "Y", "Phi")))
#    print query_pose

#    answers = robot.reasoner.query(query_pose)
#    print answers
#    initial_pose = (answers[0]["X"], answers[0]["Y"], answers[0]["Phi"])

    query = Compound('region_of_interest', 'tables') #region_of_interest(rgo2013, open_challenge, tables, point_3d(0, 0, 0), point_3d(2, 2, 2))
    print "THE QUEERRRRYYY  :::: :: :: : :  "  + str(robot.reasoner.query(query))


    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE',
                        states.Initialize(robot),
                        transitions={'initialized':'INIT_POSE',
                                     'abort':'Aborted'})
        
        smach.StateMachine.add('INIT_POSE',
                        states.Set_initial_pose(robot, "initial_open_challenge"),
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
                        transitions={   'done':'Done', 'failed':'Done'})

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
                states.LookForObjectsAtROI(robot, None, None), # En andere dingen
                transitions={  'no_object_found':'MOVE_TO_TABLE', 'object_found': 'LOOK_FOR_SERVE_OBJECT', 'looking' : 'NEVER_REACHED', 'abort' : 'NEVER_REACHED'})
        


        # STATE CHECK_OBJECTS_ON_TABLE: see if the object is the requested object
        #  if yes: grasp (with standard fallback scenarios)
        #    if person in world model: move to person
        #    else: move to prior position
        #  else:
        #    if other object in world model: go there
        #    else: move to prior position 

        smach.StateMachine.add("LOOK_FOR_SERVE_OBJECT", 
            LookForServeObject(robot), # En andere dingen
            transitions={   'found':'GRAB', 'not_found':'MOVE_TO_TABLE'})    

        smach.StateMachine.add("GRAB", 
            states.GrabMachine(side, robot, None), # En andere dingen
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
    rospy.init_node('open_challenge_executive', log_level=rospy.DEBUG)
    startup(setup_statemachine)

