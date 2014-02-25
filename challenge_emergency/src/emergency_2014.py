#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_emergency')
import rospy
import smach
import geometry_msgs.msg

#import sys
import roslib.packages as p

import robot_skills.util.msg_constructors as msgs
import robot_smach_states as states
from robot_smach_states.util.startup import startup
import robot_smach_states.util.transformations as transformations

from speech_interpreter.srv import AskUser
from virtual_cam.srv import cheese
from challenge_emergency.srv import Start


#import states_new as states 
from psi import Compound, Sequence, Conjunction, Term


#########################################
# Created by: Teun Derksen #
#########################################
class LookForObject(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["looking" , "found", "not_found"])
        self.robot = robot

    def execute(self, userdata=None):
        serving_drink = "coke"    
        goal_answers = self.robot.reasoner.query(Conjunction(  Compound("=", "Waypoint", Compound("storage_room", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint"))))

        if not goal_answers:
            self.robot.speech.speak("I want to find the drink, but I don't know where to go... I'm sorry!")
            return "not_found"

        self.robot.speech.speak("I think I know the location of your " + serving_drink)
        # for now, take the first goal found
        goal_answer = goal_answers[0]

        self.robot.speech.speak("I'm on the move, looking for your drink!")

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        nav = NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        # we tried to make it to the new goal. Let's have a look to see whether we can find the object here
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        ## If nav_result is unreachable DO NOT stop looking, there are more options, return not_found when list of Waypoints is empty
        if nav_result == "unreachable":                    
            return "looking"
        elif nav_result == "preempted":
            return "looking"

        # we made it to the new goal. Let's have a look to see whether we can find the object here
        #self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        # look to ROI
        roi_answers = self.robot.reasoner.query(Compound("point_of_interest", waypoint_name, Compound("point_3d", "X", "Y", "Z")))
        if roi_answers:
            roi_answer = roi_answers[0]
            self.robot.head.send_goal(msgs.PointStamped(float(roi_answer["X"]), float(roi_answer["Y"]), float(roi_answer["Z"]), "/map"))

        self.robot.speech.speak("I will now start my perception module, and try to detect your drink")

        self.robot.perception.toggle(["template_matching"])
        rospy.sleep(5.0)
        self.robot.perception.toggle([])

        object_answers = self.robot.reasoner.query(Conjunction(  Compound("goal", Compound("serve", "Drink")),
                                           Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                           Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))))
        if not object_answers:
            rospy.loginfo("Template matching did not return an evidence.")
            object_answer_alternative = self.robot.reasoner.query(Conjunction(  Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))))
            if not object_answer_alternative:
                return "looking"

            grasp_drink = str(object_answer_alternative[0]["Drink"])
            self.robot.speech.speak("I apologize, I could not find your drink. However, I will bring you a " + str(grasp_drink))
            self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", grasp_drink))))
            return "found"

        if object_answers:
            self.robot.speech.speak("Here it is! I found the " + serving_drink)
            return "found"
        else:
            return "looking"



class Navigate_to_queryoutcome_emergency(states.Navigate_abstract):
    """Move to the output of a query, which is passed to this state as a Term from the reasoner-module.
    
    The state can take some parameters that specify which keys of the dictionary to use for which data.
    By default, the binding-key "X" refers to the x-part of the goal, etc. 
    
    Optionally, also a sorter can be given that sorts the bindings according to some measure.
    """
    def __init__(self, robot, query, X="X", Y="Y", Phi="Phi"):
        states.Navigate_abstract.__init__(self, robot)

        assert isinstance(query, Term)

        self.queryTerm = query
        self.X, self.Y, self.Phi = X, Y, Phi
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.queryTerm)

        if not answers:
            return None
            rospy.logerr("No answers found for query {query}".format(query=self.queryTerm))
        else:
            chosen_answer = answers[0]
            #From the summarized answer, 
            possible_locations = [( float(answer[self.X]),
                                    float(answer[self.Y]), 
                                    float(answer[self.Phi])) for answer in answers]

            x,y,phi = possible_locations[0]
            
            goal = possible_locations[0]
            rospy.loginfo("goal = {0}".format(goal))
            waypoint_name = chosen_answer["Waypoint"]
            
            self.robot.reasoner.query(Compound("assert", Compound("current_exploration_target", waypoint_name))) 

            rospy.logdebug("Found location for '{0}': {1}".format(self.queryTerm, (x,y,phi)))

            target_pose =  geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=self.robot.base.point(x,y), 
                                                                                     orientation=self.robot.base.orient(phi)))
            target_pose.header.frame_id = "/map"
            return target_pose



class LookForPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["looking" , "found", "not_found"])
        self.robot = robot

    def execute(self, userdata=None):

        # Move to the next waypoint in the room, make 'room' variable
        goal_answers = self.robot.reasoner.query(Conjunction(  
                                                    Compound("=", "Waypoint", Compound("kitchen_1", "W")),
                                                    Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                    Compound("not", Compound("visited", "Waypoint"))
                                                            ))

        if not goal_answers:
            self.robot.speech.speak("I have been looking everywhere. But could not find you.")
            return "not_found"

        # For now, take the first goal found
        goal_answer = goal_answers[0]
        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        nav = states.NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        if nav_result == "unreachable":                    
            return "looking"
        elif nav_result == "preempted":
            return "looking"

        return 'found'


        #### THIS IS IN A DIFFERENT STATE
        '''
        self.robot.head.set_pan_tilt(tilt=-0.2)
        self.robot.spindle.reset()

        self.robot.speech.speak("Let me see who I can find here...")
        
        self.response_start = self.robot.perception.toggle(["face_segmentation"])
        if self.response_start.error_code == 0:
            rospy.loginfo("Face segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Face segmentation failed to start")
            self.robot.speech.speak("I was not able to start face segmentation.")
            return 'looking'
        rospy.sleep(2)

        rospy.loginfo("Face segmentation will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face segmentation is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping face segmentation")

        person_query = Conjunction(  
                                    Compound( "property_expected", "ObjectID", "class_label", "face"),
                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        person_result = self.robot.reasoner.query(person_query)
 
        if not person_result:
            self.robot.head.set_pan_tilt(tilt=0.2)
            #self.robot.speech.speak("No one here. Checking for sitting persons!")

            self.response_start = self.robot.perception.toggle(["face_segmentation"])
            if self.response_start.error_code == 0:
                rospy.loginfo("Face segmentation has started correctly")
            elif self.response_start.error_code == 1:
                rospy.loginfo("Face segmentation failed to start")
                self.robot.speech.speak("I was not able to start face segmentation.")
                return 'looking'
            rospy.sleep(2)

            rospy.loginfo("Face segmentation will be stopped now")
            self.response_stop = self.robot.perception.toggle([])
        
            if self.response_stop.error_code == 0:
                rospy.loginfo("Face segmentation is stopped")
            elif self.response_stop.error_code == 1:
                rospy.loginfo("Failed stopping face segmentation")

            person_result = self.robot.reasoner.query(person_query)

            if not person_result:
                self.robot.speech.speak("No one here.")
                return "looking"
        if len(person_result) > 1:
            self.robot.speech.speak("I see some people!")
        else:
            self.robot.speech.speak("I found someone!")
        return "found"
        '''     

class DetectPerson(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done", "failed"])

        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo("reset_head and spindle")
        self.robot.spindle.reset()
        self.robot.head.reset_position()
        rospy.sleep(1.5)
        
        rospy.loginfo("Starting face segmentation")
        self.response_start = self.robot.perception.toggle(['face_segmentation'])
        #rospy.loginfo("error_code = {0}".format(self.response_start.error_code))
        #rospy.loginfo("error_msg = {0}".format(self.response_start.error_msg))
        if self.response_start.error_code == 0:
            rospy.loginfo("Face segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Face segmentation failed to start")
            self.robot.speech.speak("I was not able to start face segmentation.")
            return "failed"
        rospy.sleep(2)

        rospy.loginfo("Face segmentation will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face segmentation is stopped")
            return "done"
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping face segmentation")
            return "failed"

class CheckPersonFound(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["no_person_found", "person_unreachable", "person_found"])
        self.robot = robot


    def execute(self, userdata=None):

        self.robot.head.reset_position()
        self.robot.spindle.reset()

        person_query = Conjunction( 
                            Compound("property_expected","ObjectID", "class_label", "face"),
                            Compound("property_expected","ObjectID", "position", Sequence("X","Y","Z")),
                            Compound("not", Compound("registered", "ObjectID")))

        persons_answers = self.robot.reasoner.query(person_query)

        if not persons_answers:
            return "no_person_found"
        
        if len(persons_answers) > 1:
            self.robot.speech.speak("I see some people, this cannot happen in 2014!!")
        else:
            self.robot.speech.speak("I found someone!")

        #####################################################################################
        # TODO Loy: 
        # - from answers of person_query, get the x,y,z value with shortest distance.
        # - most important is to know what the ObjectID is. This ObjectID should be asserted as registered(already done below))
        # - assert x,y,z value to reasoner.
        # - formualte new query for NavigateGeneric with only this answer possible
        # - run NavigateGeneric with lookat_query=..new query..
        #####################################################################################


        # persons = (float(persons_answers["X"]), float(persons_answers["Y"]), float(persons_answers["Z"]))
        # rospy.loginfo("persons = {0}".format(persons))
        
        # person_names = persons_answers["ObjectID"]
        # rospy.loginfo("waypoint_name = {0}".format(person_names))
        

        # x,y,phi = min(goals, key=distance_to_base)

        # rospy.loginfo("x = {0}, y = {1}, phi = {2}".format(x, y, phi))


        #nav = states.NavigateGeneric(self.robot, lookat_query=person_query) 
        #nav = states.NavigateGeneric(self.robot, lookat_query=person_query, goal_sorter=distance_to_base) 

        nav = Navigate_to_queryoutcome_point_emergency(self.robot, person_query, X="X", Y="Y", Z="Z")

        nav_result = nav.execute()

        if nav_result == "unreachable" or nav_result == "preempted":
            self.robot.reasoner.query(Conjunction(Compound("current_person", "ObjectID"),
                                                      Compound("assert", Compound("registered", "ObjectID"))))
            return "person_unreachable"

        elif nav_result == "goal_not_defined":
            rospy.loginfo("Goal not defined received by NavigateGeneric. Should never happen, since this check has done before calling upon NavigateGeneric.")
            return "no_person_found"

        self.robot.reasoner.query(Conjunction(Compound("current_person", "ObjectID"),
                                              Compound("assert", Compound("registered", "ObjectID"))))                 

        return "person_found"



class AskYesNo(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["yes", "preempted", "no"])

        self.robot = robot
        self.preempted = False
        self.rate = rate
        self.ask_user_service_get_yes_no = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata=None):

        self.response = self.ask_user_service_get_yes_no("yesno", 2 , rospy.Duration(8))

        for x in range(0,len(self.response.keys)):
                if self.response.keys[x] == "answer":
                    response_answer = self.response.values[x]

        if response_answer == "true":
            return "yes"
        else:
            return "no"

# Class for registering person
class Register(smach.State):
    def __init__(self, robot=None, status=1):
        smach.State.__init__(self, outcomes=['finished'])
    
        self.robot = robot

        self.status = status

        self.get_picture = rospy.ServiceProxy('virtual_cam_saver/cheese', cheese)

    def execute(self, userdata=None):    

        return_result = self.robot.reasoner.query(Compound("register_person_no", "X"))
        person_no = float(return_result[0]["X"])  

        #First move head to look at person where face is detected
        person_query = Conjunction( 
                            Compound("current_person","ObjectID"),
                            Compound("property_expected","ObjectID", "position", Sequence("X","Y","Z")))

        answers = self.robot.reasoner.query(person_query)

        if not answers:            
            rospy.logerr("No answers found for query. SHOULD NOT HAPPEN!! Query: {query}".format(query=person_query))
            pos = self.robot.base.location.pose.position
            x = pos.x
            y = pos.y
        else:
            possible_locations = [( float(answer["X"]), 
                                    float(answer["Y"]), 
                                    float(answer["Z"])) for answer in answers]

            x,y,z = possible_locations[0]

        rospy.logdebug("[EG] status of person is {0} (1 = not oke, 0 is oke)".format(self.status))
        rospy.logdebug("[EG] person_no = {0}".format(person_no))
        rospy.logdebug("[EG] x value person = {0}".format(x))
        rospy.logdebug("[EG] y value person = {0}".format(y))

        # Register person
        rospy.logdebug("Register person in file ....")
        if person_no == 1:
            f = open(p.get_pkg_dir('challenge_emergency')+'/output/status.txt','w')  # 'w' means write
            
        else:
            f = open(p.get_pkg_dir('challenge_emergency')+'/output/status.txt','a')  # 'a' means append

        #f = open('status.txt','a')
        if self.status == 0:
            f.write('person_%d;0;' % person_no)  # 0 is not oke; 1 is ok and ;2 fire
            f.write('%.2f;%.2f \n' % (x,y))
            f.close()
            pathname = "/home/amigo/ros/fuerte/tue/trunk/tue_robocup/challenge_emergency/output/person_%d.png" % person_no

        elif self.status == 1:
            f.write('person_%d;1;' % person_no)  # 0 is not oke; 1 is ok and ;2 fire
            f.write('%.2f;%.2f \n' % (x, y))
            f.close()
            pathname = "/home/amigo/ros/fuerte/tue/trunk/tue_robocup/challenge_emergency/output/person_%d.png" % person_no

        elif self.status == 2:
            f.write('fire_1;2;')                # 0 is not oke; 1 is ok and ;2 fire
            f.write('%.2f;%.2f \n' % (x, y))  # TODO ERIK exact location stove
            f.close()
            pathname = "/home/amigo/ros/fuerte/tue/trunk/tue_robocup/challenge_emergency/output/fire.png"
        
        
        rospy.logdebug("pathname = {0}".format(pathname))

        ### TODO/IDEA: play sound of taking picture

        if self.get_picture(pathname):  # bool terug. (filename met hele pad.png)
            rospy.logdebug("picture taken!!")
        else:
            rospy.logdebug("no picture taken, i'm sorry")       

        person_no += 1
        self.robot.reasoner.query(Compound("retractall", Compound("register_person_no", "X")))
        self.robot.reasoner.query(Compound("assertz",Compound("register_person_no", person_no)))
        
        return 'finished'


class RunPdfCreator(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done', 'failed'])

        self.robot = robot
        self.startup_pdf_creator = rospy.ServiceProxy('/emergency_paper/startup', Start)

    def execute(self, userdata=None):

        try:
            self.response = self.startup_pdf_creator()  # starts pdf creator
        except Exception, e:
        #except KeyError, ke:
            print e
            rospy.loginfo("FAILED creating pdf on usb-stick")
            return "failed"
        rospy.loginfo("PDF is created on usb-stick")
        
        #rospy.loginfo("[EG] DELETE SLEEP AFTER TESTING")
        #rospy.sleep(60)
        
        return "done"


class LookAtPerson(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata=None):    

        #First move head to look at person where face is detected
        person_query = Conjunction( 
                            Compound("current_person","ObjectID"),
                            Compound("property_expected","ObjectID", "position", Sequence("X","Y","Z")))

        answers = self.robot.reasoner.query(person_query)

        if not answers:            
            rospy.logerr("No answers found for query. SHOULD NOT HAPPEN!! Query: {query}".format(query=person_query))
            pos = self.robot.base.location.pose.position
            x = pos.x
            y = pos.y
        else:
            possible_locations = [( float(answer["X"]), 
                                    float(answer["Y"]), 
                                    float(answer["Z"])) for answer in answers]

            x,y,z = possible_locations[0]

            if z > 1.5:
                self.robot.spindle.send_goal(0.4,timeout=5.0)  
                rospy.logdebug("Spindle should come up now!")

            lookat_point = msgs.PointStamped(x,y,z)
            rospy.loginfo("AMIGO should look at person now. (x = {0}, y = {1}, z = {2})".format(x,y,z))
            self.robot.head.send_goal(lookat_point, timeout=0, keep_tracking=True)
        return 'finished'

class CheckWaypointsAvailable(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['available','not_available'])

        self.robot = robot

    def execute(self, userdata=None):    

        navigate_apartment = Conjunction(  Compound("=", "Waypoint", Compound("apartment", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint")),
                                                 Compound("not", Compound("unreachable", "Waypoint")))

        goal_answers = self.robot.reasoner.query(navigate_apartment)             # I do not use not_visited and not_unreachable since these are no roi's

        if not goal_answers:
            return "not_available"     
        else:
            return "available"

def setup_statemachine(robot):

    # Retract old facts
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
    robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    robot.reasoner.query(Compound("retractall", Compound("current_object", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_visited", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_unreachable", "X")))
    robot.reasoner.query(Compound("retractall", Compound("registered", "X")))
    robot.reasoner.query(Compound("retractall", Compound("visited", "X")))
    robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))

    # Emergency parameters
    robot.reasoner.query(Compound("retractall", Compound("current_person", "X")))
    robot.reasoner.query(Compound("retractall", Compound("register_person_no", "X")))
    robot.reasoner.query(Compound("assert",Compound("register_person_no", "1")))

    # Load database
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

    # Assert the current challenge.
    robot.reasoner.query(Compound("assertz",Compound("challenge", "emergency")))

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    with sm:

        ######################################################
        ################## ENTER APPARTMENT ##################
        ######################################################
        smach.StateMachine.add( "START_CHALLENGE",
                                    states.StartChallengeRobust(robot, "initial"), 
                                    transitions={   "Done":"SAY_LOOK_FOR_PERSON", 
                                                    "Aborted":"SAY_LOOK_FOR_PERSON", 
                                                    "Failed":"SAY_LOOK_FOR_PERSON"})

        smach.StateMachine.add("SAY_LOOK_FOR_PERSON",
                                    states.Say(robot, "Looking for person.", block=False),
                                    transitions={   "spoken":"GO_TO_KITCHEN"})
        
        ######################################################
        ################### FIND THE PERSON ##################
        ######################################################
        query_kitchen_1 = Compound("waypoint", "kitchen_1", Compound("pose_2d", "X", "Y", "Phi"))
        query_bedroom_1 = Compound("waypoint", "bedroom_1", Compound("pose_2d", "X", "Y", "Phi"))
        query_livingroom_1 = Compound("waypoint", "livingroom_1", Compound("pose_2d", "X", "Y", "Phi"))

        # Look for the person in kitchen should check for every room
        smach.StateMachine.add('GO_TO_KITCHEN', 
                                    states.Navigate_to_queryoutcome(robot, query_kitchen_1, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'RESET_HEAD', 
                                                    'preempted':'GO_TO_KITCHEN_SECOND_TRY', 
                                                    'unreachable':'GO_TO_KITCHEN_SECOND_TRY', 
                                                    'goal_not_defined':'GO_TO_KITCHEN_SECOND_TRY'})

        query_kitchen_2 = Compound("waypoint", "kitchen_2", Compound("pose_2d", "X", "Y", "Phi"))
        smach.StateMachine.add('GO_TO_KITCHEN_SECOND_TRY', 
                                    states.Navigate_to_queryoutcome(robot, query_kitchen_2, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'RESET_HEAD', 
                                                    'preempted':'RESET_HEAD', 
                                                    'unreachable':'RESET_HEAD', 
                                                    'goal_not_defined':'RESET_HEAD'})

        smach.StateMachine.add("RESET_HEAD",
                                states.ResetHead(robot),
                                transitions={'done':'FIND_PERSON'})

        ######################################################
        #################   DETECT PERSON   ##################
        ######################################################
        smach.StateMachine.add("FIND_PERSON",
                                LookForPerson(robot),
                                transitions={   'found':'SAY_START_PEOPLE_DETECTION',
                                                'not_found':'SAY_GO_TO_EXIT',
                                                'looking':'FIND_PERSON'})
  
        smach.StateMachine.add("FAILED_DRIVING_TO_LOCATION",
                                states.Say(robot,"I was not able to reach the desired location to detect people. I will try another location.", block=False),  #LOCATION SHOULD BE FOUND, otherwise sentence is to long for non-blocking
                                transitions={'spoken':'FIND_PERSON'})

        smach.StateMachine.add("SAY_START_PEOPLE_DETECTION",
                                states.Say(robot,"Please look into my eyes. I will start my perception now.", block=False),
                                transitions={'spoken':'START_PEOPLE_DETECTION'})

        smach.StateMachine.add("START_PEOPLE_DETECTION",
                                DetectPerson(robot),
                                transitions={'done':'CHECK_WORLD_MODEL_FOR_PERSON',
                                             'failed':'FAILED_PERCEPTION'})

        # Could not reach ROI    
        smach.StateMachine.add("FAILED_PERCEPTION",
                                states.Say(robot,"I failed starting perception, maybe more luck at the next location.", block=False),  #LOCATION SHOULD BE FOUND, otherwise sentence is to long for non-blocking
                                transitions={'spoken':'FIND_PERSON'})


        smach.StateMachine.add("CHECK_WORLD_MODEL_FOR_PERSON",
                                CheckPersonFound(robot),
                                transitions={'no_person_found':'SAY_NO_PERSON_FOUND',
                                             'person_unreachable':'SAY_PERSON_UNREACHABLE',
                                             'person_found':'LOOK_AT_PERSON'})

        smach.StateMachine.add("SAY_NO_PERSON_FOUND",
                                states.Say(robot,"I do not see a person over here", block=False),  #LOCATION SHOULD BE FOUND, otherwise sentence is to long for non-blocking
                                transitions={'spoken':'NO_PERSON_FOUND'})

        smach.StateMachine.add("NO_PERSON_FOUND",
                                CheckWaypointsAvailable(robot),
                                transitions={'available':'SAY_NEXT_LOCATION',
                                             'not_available':'SAY_GO_TO_EXIT'})

        smach.StateMachine.add("SAY_NEXT_LOCATION",
                                states.Say(robot,"I will try the next location.", block=False),  #LOCATION SHOULD BE FOUND, otherwise sentence is to long for non-blocking
                                transitions={'spoken':'FIND_PERSON'})


        smach.StateMachine.add("SAY_PERSON_UNREACHABLE",
                                states.Say(robot,["I failed going to the desired person", "I am not able to reach the person I want to speak"], block=False),
                                transitions={'spoken':'GO_TO_LAST_EXPLORATION_POINT'})

        ## hier state toevoegen, terugrijden naar lookat point.

        query_last_exploration_location = Conjunction(Compound("current_exploration_target", "Location"),
                                                      Compound("waypoint", "Location", Compound("pose_2d", "X", "Y", "Phi")))

        smach.StateMachine.add('GO_TO_LAST_EXPLORATION_POINT', 
                                    states.Navigate_to_queryoutcome(robot, query_last_exploration_location, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE', 
                                                    'preempted':'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE', 
                                                    'unreachable':'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE', 
                                                    'goal_not_defined':'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE'})

        smach.StateMachine.add("SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE",
                                states.Say(robot,["I will ask my questions from here.", "I will ask you some questions from here."], block=False),
                                transitions={'spoken':'LOOK_AT_PERSON'})

        # Look at person
        smach.StateMachine.add('LOOK_AT_PERSON',
                                    LookAtPerson(robot),                          
                                    transitions={'finished':'ASK_IF_AMBULANCE'})   

        ######################################################
        #############   ASK AMBU AND REGISTER   ##############
        ######################################################
        smach.StateMachine.add('ASK_IF_AMBULANCE',
                                    states.Say(robot, 'Do you need an ambulance?'),
                                    transitions={'spoken':'ANSWER_AMBULANCE'})

        smach.StateMachine.add('ANSWER_AMBULANCE',
                                    AskYesNo(robot),
                                    transitions={   'yes':'SAY_REGISTER_NOT_OKAY',             # REGISTER AND RUN PDF
                                                    'preempted':'ASK_FETCH',        #
                                                    'no':'ASK_FETCH'})    
        smach.StateMachine.add('ASK_FETCH',
                                    states.Say(robot, 'What can I fetch you?'),
                                    transitions={'spoken':'LOOK_FOR_OBJECT'})

        # Person is not okay:
        smach.StateMachine.add('SAY_REGISTER_NOT_OKAY',
                                    states.Say(robot, 'I will register your position and take a picture so the ambulance can find you.', block=True),
                                    transitions={'spoken':'REGISTER_PERSON_NOT_OKAY'})     

        smach.StateMachine.add('REGISTER_PERSON_NOT_OKAY',
                                    Register(robot, status=0),  #input 0 (person is not oke)
                                    transitions={'finished':'SAVE_PDF_ON_STICK'})

        smach.StateMachine.add('SAVE_PDF_ON_STICK',
                                    RunPdfCreator(robot),
                                    transitions={'done':'SAY_REGISTER_NOT_OKAY_CALM',
                                                 'failed':'SAY_REGISTER_NOT_OKAY_CALM'})

        smach.StateMachine.add('SAY_REGISTER_NOT_OKAY_CALM',
                                    states.Say(robot, 'Please stay calm and help will arrive very soon.', block=False), 
                                    transitions={'spoken':'LOOK_FOR_OBJECT'})
        ######################################################
        ############### ASK FETCH AND DELIVER ################
        ######################################################
        smach.StateMachine.add( 'LOOK_FOR_OBJECT',
                                    LookForObject(robot),
                                    transitions={   "looking":"LOOK_FOR_OBJECT",
                                                    "found":'SAY_DRINK_NOT_FOUND',
                                                    "not_found":'SAY_DRINK_NOT_FOUND'})
        smach.StateMachine.add( 'SAY_DRINK_NOT_FOUND',
                                    states.Say(robot, ["I could not find the drink you wanted.", 
                                                "I looked really hard, but I couldn't find your drink."]),
                                    transitions={   'spoken':'SAY_GO_TO_EXIT' }) 

        ######################################################
        ############# GO TO ENTRY OF APPARTMENT ##############
        ######################################################
        
        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('SAY_GO_TO_EXIT',
                                    states.Say(robot, "I have searched the whole apartment for people. Therefore I will go to the exit. If people are still here and can hear me, go to the exit!!", block=False),
                                    transitions={'spoken':'GO_TO_EXIT'}) 

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT', 
                                    states.Navigate_named(robot, "exit_1"),
                                    transitions={   'arrived':'SUCCEED_GO_TO_EXIT', 
                                                    'preempted':'CLEAR_PATH_TO_EXIT', 
                                                    'unreachable':'CLEAR_PATH_TO_EXIT', 
                                                    'goal_not_defined':'CLEAR_PATH_TO_EXIT'})

        # Amigo will say that it arrives at the registration table
        smach.StateMachine.add('CLEAR_PATH_TO_EXIT',
                                    states.Say(robot, "I couldn't go to the exit. Please clear the path, I will give it another try."),
                                    transitions={'spoken':'GO_TO_EXIT_SECOND_TRY'}) 

        # Then amigo will drive to the registration table. Defined in knowledge base. Now it is the table in the test map.
        smach.StateMachine.add('GO_TO_EXIT_SECOND_TRY', 
                                    states.Navigate_named(robot, "exit_2"),
                                    transitions={   'arrived':'SUCCEED_GO_TO_EXIT', 
                                                    'preempted':'FAILED_GO_TO_EXIT', 
                                                    'unreachable':'FAILED_GO_TO_EXIT', 
                                                    'goal_not_defined':'FAILED_GO_TO_EXIT'})

        smach.StateMachine.add('FAILED_GO_TO_EXIT',
                                    states.Say(robot, 'I was not able to go to the exit. I will stop here and save all the information I gathered in a PDF file on a USB stick.', block=False),
                                    transitions={'spoken':'SAVE_PDF_ON_STICK_FINAL'})

        smach.StateMachine.add('SUCCEED_GO_TO_EXIT',
                                    states.Say(robot, 'I will now save all the information I gathered in a PDF file on a USB stick.', block=False),
                                    transitions={'spoken':'SAVE_PDF_ON_STICK_FINAL'})

        smach.StateMachine.add('SAVE_PDF_ON_STICK_FINAL',
                                    RunPdfCreator(robot),
                                    transitions={'done':'AT_END',
                                                 'failed':'AT_END'})
    

        ######################################################
        ################# WAIT FOR AMBULANCE #################
        ######################################################

        
        # Finally amigo will stop and says 'goodbye' to show that he's done.
        smach.StateMachine.add('AT_END',
                                states.Say(robot, "Goodbye"),
                                transitions={'spoken':'Done'})

    return sm

if __name__ == "__main__":
    rospy.init_node('emergency_exec')  #, log_level=rospy.DEBUG)
    rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    rospy.loginfo("!!!!!!!!!!!!!!!!!EMERGENCY CHALLENGE!!!!!!!!!!!!!!!!!")
    rospy.loginfo("!!! MAKE SURE DEPENDENCIES LAUNCH FILE IS RUNNING !!!")
    rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    startup(setup_statemachine)
