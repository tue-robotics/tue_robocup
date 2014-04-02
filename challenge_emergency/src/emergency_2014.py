#!/usr/bin/python
import roslib; roslib.load_manifest('challenge_emergency')
import rospy
import smach

import roslib.packages as p
import std_msgs.msg
import robot_skills.util.msg_constructors as msgs
import robot_smach_states as states
from robot_smach_states.util.startup import startup

from speech_interpreter.srv import AskUser
from virtual_cam.srv import cheese
from challenge_emergency.srv import Start
from person_emergency_detector.srv import Switch
from person_emergency_detector.srv import GetUnknownOctomapBlobPositions
from person_emergency_detector.msg import Position

#import states_new as states 
from psi import Compound, Sequence, Conjunction

# Hardcoded emergency room {'office','bedroom' or 'kitchen'}
room = 'office'

''' TO DO:
- Make a list of likely and unlikely positions for the emergency to occur
- Fill in hardcoded room before the start of the challenge also in the pdf creator
- Verification of a face when you drove to it!
- Decide if we want to detect the accident or just move to a person
'''
#########################################
#       Created by: Teun Derksen        #
#########################################
class SleepTime(smach.State):
    def __init__(self, robot=None,sleeptime=1):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot
        self.sleeptime = sleeptime

    def execute(self, userdata=None):    
        rospy.sleep(self.sleeptime)
        return "finished"

class UnknownOctomapBlobDetector(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["success", "failed", "looking"])

        self.counter = 0
        self.robot = robot
        self.unknown_blob_detection = rospy.ServiceProxy('/unknown_octomap_blob_detector/get_unknown_octomap_blob_positions',GetUnknownOctomapBlobPositions) 

    def execute(self, userdata=None):


        self.robot.spindle.reset()
        self.robot.head.reset_position()

        self.robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X"))) 

        navigate_room = Conjunction(  Compound("=", "Waypoint", Compound(room, "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint")),
                                                 Compound("not", Compound("unreachable", "Waypoint")))

        goal_answers = self.robot.reasoner.query(navigate_room)             # I do not use not_visited and not_unreachable since these are no roi's

        self.robot.reasoner.query(Compound("assert", Compound("current_exploration_target", "Waypoint"))) 

        if not goal_answers:
            return "failed"

        goal_answer = goal_answers[0]
        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))

        waypoint_name = goal_answer["Waypoint"]
            

        nav = states.NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()
        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        # look to ROI
        if room == "kitchen" and self.counter == 1:
            roi_answers = self.robot.reasoner.query(Compound("point_of_interest", Compound("kitchen2", "W"), Compound("point_3d", "X", "Y", "Z")))
            if roi_answers:
                #print roi_answers
                for x in range(0,len(roi_answers)):
                    roi_answer = roi_answers[x]
                    #print roi_answer
                    self.robot.head.send_goal(msgs.PointStamped(float(roi_answer["X"]), float(roi_answer["Y"]), float(roi_answer["Z"]), "/map"))
                    rospy.sleep(1.5)

        else:
            roi_answers = self.robot.reasoner.query(Compound("point_of_interest", Compound(room, "W"), Compound("point_3d", "X", "Y", "Z")))
            if roi_answers:
                #print roi_answers
                for x in range(0,len(roi_answers)):
                    roi_answer = roi_answers[x]
                    #print roi_answer
                    self.robot.head.send_goal(msgs.PointStamped(float(roi_answer["X"]), float(roi_answer["Y"]), float(roi_answer["Z"]), "/map"))
                    rospy.sleep(1.5)

        #rospy.wait_for_service("/unknown_octomap_blob_detector/get_unknown_octomap_blob_positions",timeout=2.0)
        try:
            if room == 'bedroom':
                self.response = self.unknown_blob_detection(0.0, -1.4, 0, 5.2, 3.5, 2)# todo     
            elif room == 'office':
                self.response = self.unknown_blob_detection(0.0, 3.6, 0, 4.75, 8.5, 2)    
            else:
                if self.counter == 0:
                    self.response = self.unknown_blob_detection(5, -1.4, 0, 8.8, 3.7, 2)
                    self.counter = self.counter + 1
                else:
                    self.response = self.unknown_blob_detection(5.0, 4, 0, 8.8, 8.5, 2)

            print self.response
            if len(self.response.positions) == 0:
                return 'looking'


            possible_locations = [( float(answer.x), 
                                    float(answer.y), 
                                    float(answer.z)) for answer in self.response.positions]

            print possible_locations
            goal = list(possible_locations[0])
            print goal
            #nav = states.NavigateGeneric(self.robot, lookat_point_3d=goal)
            #nav_result = nav.execute()

            # HACK TO LOOK UP WHEN FACING STANDING PEOPLE
            if goal[2] > 0.45:
                goal[2] = 1.2

            self.robot.reasoner.assertz(Compound("emergency_person", Compound("point_3d", Sequence(goal[0], goal[1], goal[2]))))
            #answers = self.robot.reasoner.query(Compound("emergency_person", Compound("point_3d",Sequence("X","Y","Z"))))
             
            lookat_point = msgs.PointStamped(goal[0],goal[1],goal[2]) # todo
            self.robot.head.send_goal(lookat_point, timeout=0, keep_tracking=True)

        except Exception, e:
            print e
            return "failed"
        return "success"


class LookingForPersonOld(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["found", "looking", "not_found"])

        self.robot = robot

    def execute(self, userdata=None):
        
        self.robot.spindle.reset()
        self.robot.head.reset_position()

        self.robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X"))) 

        navigate_room = Conjunction(  Compound("=", "Waypoint", Compound(room, "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint")),
                                                 Compound("not", Compound("unreachable", "Waypoint")))

        goal_answers = self.robot.reasoner.query(navigate_room)             # I do not use not_visited and not_unreachable since these are no roi's

        self.robot.reasoner.query(Compound("assert", Compound("current_exploration_target", "Waypoint"))) 

        if not goal_answers:
            return "not_found"

        goal_answer = goal_answers[0]
        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))

        waypoint_name = goal_answer["Waypoint"]
            

        nav = states.NavigateGeneric(self.robot, goal_pose_2d=goal)
        nav_result = nav.execute()

        self.robot.reasoner.query(Compound("assert", Compound("visited", waypoint_name)))

        if nav_result == "unreachable" or nav_result == "preempted":
            return "looking"

        self.robot.head.set_pan_tilt(tilt=-0.2)
        self.robot.spindle.reset()

        # we made it to the new goal. Let's have a look to see whether we can find the person here
        #self.robot.speech.speak("Let me see who I can find here...")
        rospy.sleep(1.5)
        
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
            #self.robot.speech.speak("Checking the floor")
            rospy.sleep(1.5)

            self.response_start = self.robot.perception.toggle(["face_segmentation"])
            if self.response_start.error_code == 0:
                rospy.loginfo("Face segmentation has started correctly")
            elif self.response_start.error_code == 1:
                rospy.loginfo("Face segmentation failed to start")
                self.robot.speech.speak("I was not able to start face segmentation.")
                return 'looking'

            rospy.sleep(1.5)

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
            self.robot.speech.speak("I see some people!",block=False)
        else:
            self.robot.speech.speak("I found someone!",block=False)
        #self.robot.reasoner.assertz(Compound("emergency_person", person_result[0]["ObjectID"]))
        self.robot.reasoner.assertz(Compound("emergency_person", Compound("point_3d", Sequence(person_result[0]["X"], person_result[0]["Y"], person_result[0]["Z"]))))


        return "found"     
        

class LookingForPerson(smach.StateMachine):
    def __init__(self, robot, detect_persons=True):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])

        self.robot = robot

        self.possible_person_query = Conjunction(   Compound("instance_of", "ObjectID", "person"), 
                                                    Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")),
                                                    Compound("not", Compound("ignored_person", "ObjectID")))

        self.current_possible_person_query = Conjunction(   Compound("current_possible_person", "ObjectID"), 
                                                            Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        self.person_query = Conjunction(  Compound("instance_of", "ObjectID", "validated_person"), 
                                          Compound("property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

        robot.reasoner.assertz(Compound("ignored_person", "nobody")) #Required to make the predicate known.
        robot.reasoner.retractall(Compound("current_possible_person", "ObjectID"))

        with self:
            if detect_persons:
                #Turn on ppl_detection and switch off after we detected a person
                smach.StateMachine.add( "WAIT_FOR_POSSIBLE_DETECTION",
                                        states.Wait_queried_perception(robot, ["ppl_detection"], self.possible_person_query, timeout=3),
                                        transitions={   "query_true":"SET_CURRENT_POSSIBLE_PERSON",
                                                        "timed_out":"Failed",
                                                        "preempted":"Aborted"})

            smach.StateMachine.add( 'SET_CURRENT_POSSIBLE_PERSON', 
                                    states.Select_object(robot, self.possible_person_query, "current_possible_person"),
                                    transitions={   'selected':'LOOK_AT_POSSIBLE_PERSON',
                                                    'no_answers':'Failed'})

            #Look at the possible_person detection, to verify through Luis's human_tracking them in a next state
            smach.StateMachine.add('LOOK_AT_POSSIBLE_PERSON',
                                    states.LookAtPoint(robot, self.current_possible_person_query),
                                    transitions={   'looking':'WAIT_FOR_HUMAN_DETECTION',
                                                    'no_point_found':'Failed',
                                                    'abort':'Aborted'})

            #Turn on human_tracking with the kinect and wait for a match of it in the world model
            smach.StateMachine.add( "WAIT_FOR_HUMAN_DETECTION",
                                    states.Wait_queried_perception(robot, ["human_tracking"], self.person_query, timeout=10),
                                    transitions={   "query_true":"RETRACT_POSSIBLE",
                                                    "timed_out":"RETRACT_POSSIBLE",
                                                    "preempted":"Aborted"})

            smach.StateMachine.add('RETRACT_POSSIBLE', 
                                    states.Select_object(robot, self.current_possible_person_query, "ignored_person"),
                                    transitions={   'selected':'Done',
                                                    'no_answers':'Failed'})

class LookAtPerson(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata=None):    

        person_query = Conjunction(  
                                    Compound( "property_expected", "ObjectID", "class_label", "face"),
                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        emergency_blob_query = Compound("emergency_person", Compound("point_3d",Sequence("X","Y","Z")))




        answers = self.robot.reasoner.query(emergency_blob_query)
        print answers

        if not answers:            
            rospy.logerr("No answers found for query. SHOULD NOT HAPPEN!! Query: {query}".format(query=emergency_blob_query))
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



class LookForObject(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["looking" , "found", "not_found"])
        self.robot = robot

    def execute(self, userdata=None):

        return_result = self.robot.reasoner.query(Compound("goal", Compound("serve", "Drink")))

        if not return_result:
            self.robot.speech.speak("I forgot which object you wanted")
            return "not_found"
        serving_drink = str(return_result[0]["Drink"])  


        goal_answers = self.robot.reasoner.query(Conjunction(  Compound("=", "Waypoint", Compound("storage_room", "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint"))))

        if not goal_answers:
            self.robot.speech.speak("I cannot find the object")
            return "not_found"

        #self.robot.speech.speak("I know the location of your " + serving_drink)
        goal_answer = goal_answers[0]

        goal = (float(goal_answer["X"]), float(goal_answer["Y"]), float(goal_answer["Phi"]))
        waypoint_name = goal_answer["Waypoint"]

        nav = states.NavigateGeneric(self.robot, goal_pose_2d=goal)
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
            print roi_answer
            self.robot.head.send_goal(msgs.PointStamped(float(roi_answer["X"]), float(roi_answer["Y"]), float(roi_answer["Z"]), "/map"))


        # query to detect object, finishes when something found or timeout!
        query_detect_object = Conjunction(Compound("goal", Compound("serve", "Drink")),
                                          Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                          Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")))

        self.response_start = self.robot.perception.toggle(["object_segmentation"])
 
        if self.response_start.error_code == 0:
            rospy.loginfo("Object recogition has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Object recogition failed to start")
            self.robot.speech.speak("I was not able to start object recognition.")
            return "not_found"

        wait_machine = states.Wait_query_true(self.robot, query_detect_object, 7)
        wait_result = wait_machine.execute()

        rospy.loginfo("Object recogition will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Object recogition is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping Object recogition")

        # interpret results wait machine
        if wait_result == "timed_out":
            return "looking"
        elif wait_result == "preempted":
            return "looking"
        elif wait_result == "query_true":
            self.robot.speech.speak("Hey, I found your " + serving_drink,block=False)
            return "found"


        '''
        self.robot.perception.toggle(["object_segmentation"])
        rospy.sleep(5.0)
        self.robot.perception.toggle([])

        object_answers = self.robot.reasoner.query(Conjunction(  Compound("goal", Compound("serve", "Drink")),
                                           Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                           Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))))
        if not object_answers:
            rospy.loginfo("object_segmentation  did not return an evidence.")
            object_answer_alternative = self.robot.reasoner.query(Conjunction(  Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo"))))
            if not object_answer_alternative:
                return "looking"

            grasp_drink = str(object_answer_alternative[0]["Drink"])
            self.robot.speech.speak("Could not find your object. I will just bring you a " + str(grasp_drink))
            self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", grasp_drink))))
            return "found"

        if object_answers:
            self.robot.speech.speak("I found the " + serving_drink)
            return "found"
        else:
            return "looking"
        '''

class AskObject(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done" , "failed"])
        self.robot = robot
        self.ask_user_service_get_object = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata=None):
        self.response = self.ask_user_service_get_object("emergency_objects", 3 , rospy.Duration(60))  # This means that within 3 tries and within 60 seconds an answer is received. 
            
        for x in range(0,len(self.response.keys)):
            if self.response.keys[x] == "answer":
                response_answer = self.response.values[x]

        if response_answer == "no_answer" or  response_answer == "wrong_answer":
            self.robot.speech.speak("I'll bring you a water",block=False)
            response_answer = "water"

        self.robot.reasoner.query(Compound("assert", Compound("goal", Compound("serve", response_answer))))
        return "done"

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
        print response_answer
        if response_answer == "true":
            return "yes"
        else:
            return "no"

class Register(smach.State):
    def __init__(self, robot=None, status=1):
        smach.State.__init__(self, outcomes=['finished'])
    
        self.robot = robot
        self.status = status
        self.get_picture = rospy.ServiceProxy('virtual_cam/cheese', cheese)

    def execute(self, userdata=None):    

        '''
        person_query = Conjunction( 
                            Compound("instance_of", "ObjectID", "person"), 
                            Compound("property_expected","ObjectID", "position", Sequence("X","Y","Z")))
        '''
        person_query = Conjunction(  
                                    Compound( "property_expected", "ObjectID", "class_label", "face"),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        emergency_blob_query = Compound("emergency_person", Compound("point_3d",Sequence("X","Y","Z")))


        answers = self.robot.reasoner.query(emergency_blob_query)

        if not answers:            
            rospy.logerr("No answers found for query. SHOULD NOT HAPPEN!!")# Query: {query}".format(query=person_query))
            pos = self.robot.base.location.pose.position
            x = pos.x
            y = pos.y
        else:
            possible_locations = [( float(answer["X"]), 
                                    float(answer["Y"]), 
                                    float(answer["Z"])) for answer in answers]

            x,y,z = possible_locations[0]

        rospy.logdebug("Status of person is {0} (1 = not oke, 0 is oke)".format(self.status))
        rospy.logdebug("x value person = {0}".format(x))
        rospy.logdebug("y value person = {0}".format(y))
        rospy.logdebug("room = {0}".format(room))

        # Register person
        rospy.logdebug("Register person in file ....")
        f = open(p.get_pkg_dir('challenge_emergency')+'/output/status.txt','w')  # 'w' means write
        rospy.logdebug("Opened file")   
        if self.status == 0:
            f.write('person;0;')  # 0 is not oke; 1 is ok and ;2 fire
            
        elif self.status == 1:
            f.write('person;1;')  # 0 is not oke; 1 is ok and ;2 fire

        rospy.logdebug("Wrote status")
        f.write('%.2f;%.2f \n' % (x, y))
        f.write(room + '\n')
        '''   
        f.write('%.2f;%.2f' % (x, y))
        f.write(room + '\n')
        '''
        f.close() 
        rospy.logdebug("Closed file")   
        pathname = "/home/amigo/ros/groovy/tue/trunk/tue_robocup/challenge_emergency/output/person.png"
        rospy.logdebug("pathname = {0}".format(pathname))

        if self.get_picture(pathname):
            rospy.loginfo("Picture taken!!")
        else:
            rospy.loginfo("No picture taken")       
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
               
        return "done"

class SwitchEmergencyDetector(smach.State):
    def __init__(self, robot, switch):
        smach.State.__init__(self, outcomes=['done','failed'])

        self.switch = switch
        self.robot = robot
        self.startup_people_emergency_detection = rospy.ServiceProxy('/person_emergency_detector/switch', Switch) 

    def execute(self, userdata=None):

        try:
            self.response = self.startup_people_emergency_detection(self.switch, 0.0) 
        except Exception, e:
            print e
            return "failed"
        return "done"


class ObservingEmergency(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done','waiting','expired'])
        self.robot = robot
        rospy.Subscriber("/person_emergency_detector/result", std_msgs.msg.String, self.callback)
        self.detected_poses = { 'wave' : 0, 'fall' : 0 }
        self.counter = 0

    def callback(self, data):
        seen = data.data
        if seen != '':
            self.detected_poses[seen] = self.detected_poses[seen] + 1
            rospy.loginfo(": I saw %s" % seen)

    def execute(self, userdata=None):

        if self.counter > 100:
            return 'expired'

        if self.detected_poses['wave'] > 30 or self.detected_poses['fall'] > 30:
            return 'done'

        self.counter = self.counter + 1 
        return 'waiting'

class WaitForAmbulance(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['detected','waiting','timed_out','failed'])
        self.robot = robot

        self.counter = 0
    
    def execute(self, userdata=None):

        if self.counter > 3:
            return 'timed_out'

        self.robot.spindle.reset()
        self.robot.head.set_pan_tilt(tilt=-0.2)
        
        if self.counter == 0:
            rospy.sleep(1.5)        
        
        self.robot.speech.speak("Waiting for help!.")


        query_detect_person = Conjunction(Compound("property_expected", "ObjectID", "class_label", "face"),
                                          Compound("property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                          Compound("property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

        self.response_start = self.robot.perception.toggle(['face_segmentation'])

        if self.response_start.error_code == 0:
            rospy.loginfo("Face segmentation has started correctly")
        elif self.response_start.error_code == 1:
            rospy.loginfo("Face segmentation failed to start")
            self.robot.speech.speak("I was not able to detect faces.")
            return "failed"

        wait_machine = states.Wait_query_true(self.robot, query_detect_person, 10)
        wait_result = wait_machine.execute()

        rospy.loginfo("Face segmentation will be stopped now")
        self.response_stop = self.robot.perception.toggle([])
        
        if self.response_stop.error_code == 0:
            rospy.loginfo("Face segmentation is stopped")
        elif self.response_stop.error_code == 1:
            rospy.loginfo("Failed stopping face segmentation")

        if wait_result == "timed_out":
            self.counter = self.counter + 1
            return "waiting"

        elif wait_result == "query_true":
            answers = self.robot.reasoner.query(query_detect_person)
            possible_locations = [( float(answer["X"]), 
                                    float(answer["Y"]), 
                                    float(answer["Z"])) for answer in answers]
            x,y,z = possible_locations[0]

            if z > 1.5:
                self.robot.spindle.high()
                rospy.logdebug("Spindle should come up now!")

            lookat_point = msgs.PointStamped(x,y,z)
            rospy.loginfo("AMIGO should look at person now. (x = {0}, y = {1}, z = {2})".format(x,y,z))
            self.robot.head.send_goal(lookat_point,timeout=0)

            return 'detected'

def setup_statemachine(robot):

    # Retract old facts
    robot.reasoner.query(Compound("retractall", Compound("goal", "X")))
    robot.reasoner.query(Compound("retractall", Compound("challenge", "X")))
    robot.reasoner.query(Compound("retractall", Compound("explored", "X")))
    robot.reasoner.query(Compound("retractall", Compound("state", "X", "Y")))
    robot.reasoner.query(Compound("retractall", Compound("current_exploration_target", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_visited", "X")))
    robot.reasoner.query(Compound("retractall", Compound("not_unreachable", "X")))
    robot.reasoner.query(Compound("retractall", Compound("emergency_person", "X")))
    robot.reasoner.query(Compound("retractall", Compound("visited", "X")))
    robot.reasoner.query(Compound("retractall", Compound("unreachable", "X")))

    # Load database
    robot.reasoner.query(Compound("load_database","tue_knowledge",'prolog/locations.pl'))

    # Assert the current challenge.
    robot.reasoner.query(Compound("assertz",Compound("challenge", "emergency")))

    sm = smach.StateMachine(outcomes=['Done','Aborted'])

    person_query = Conjunction(  
                                    Compound( "property_expected", "ObjectID", "class_label", "face"),
                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

    general_person_query = Conjunction(  
                                    Compound( "property_expected", "ObjectID", "class_label", "face"),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")))

    emergency_person_query = Conjunction(  
                                    Compound( "property_expected", "ObjectID", "class_label", "face"),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X","Y","Z")),
                                    Compound( "emergency_person", "ObjectID"))

    grabpoint_query = Conjunction(  
                                    Compound("goal", Compound("serve", "Drink")),
                                    Compound( "property_expected", "ObjectID", "class_label", "Drink"),
                                    Compound( "property_expected", "ObjectID", "position", Compound("in_front_of", "amigo")),
                                    Compound( "property_expected", "ObjectID", "position", Sequence("X", "Y", "Z")))

    fallback_ambulance_query = Compound("waypoint", room, Compound("pose_2d", "X", "Y", "Phi"))

    query_last_exploration_location = Conjunction(Compound("current_exploration_target", room),
                                                  Compound("waypoint", room, Compound("pose_2d", "X", "Y", "Phi")))


    emergency_blob_query = Compound("emergency_person", Compound("point_3d",Sequence("X","Y","Z")))

    arm = robot.leftArm

    with sm:

        ######################################################
        ################## ENTER APPARTMENT ##################
        ######################################################
        smach.StateMachine.add( "START_CHALLENGE",
                                states.StartChallengeRobust(robot, "initial"), 
                                transitions={   "Done":"SAY_LOOK_FOR_PERSON", 
                                                "Aborted":"SAY_LOOK_FOR_PERSON", 
                                                "Failed":"SAY_LOOK_FOR_PERSON"})

        smach.StateMachine.add( "SAY_LOOK_FOR_PERSON",
                                states.Say(robot,"Going to the " + room, block=False),
                                transitions={    "spoken":"FIND_PERSON_BACKUP"})
                                #transitions={    "spoken":"FIND_PERSON"})

        ######################################################
        ########## GO TO ROOM AND LOOK FOR PERSON ############
        ######################################################
        smach.StateMachine.add( "FIND_PERSON",
                                LookingForPersonOld(robot),
                                transitions={   'found':'NAVIGATE_TO_PERSON',
                                                'looking':'FIND_PERSON',
                                                'not_found':'ASK_IF_AMBULANCE'})

        smach.StateMachine.add( "FIND_PERSON_BACKUP",
                                UnknownOctomapBlobDetector(robot),
                                transitions={   'success':'NAVIGATE_TO_PERSON',
                                                'failed':'FIND_PERSON',
                                                'looking':'FIND_PERSON_BACKUP'})
        
        smach.StateMachine.add( "NAVIGATE_TO_PERSON",
                                #states.NavigateGeneric(robot, lookat_query=general_person_query, xy_dist_to_goal_tuple=(1.0,0)),
                                states.NavigateGeneric(robot, lookat_query=emergency_blob_query, xy_dist_to_goal_tuple=(1.0,0)),
                                transitions={   "arrived":"LOOK_AT_PERSON",
                                                "unreachable":'SAY_PERSON_UNREACHABLE',
                                                "preempted":'SAY_PERSON_UNREACHABLE',
                                                "goal_not_defined":'SAY_PERSON_UNREACHABLE'})

        smach.StateMachine.add( 'LOOK_AT_PERSON',
                                LookAtPerson(robot),                          
                                transitions={'finished':'APPROACH_PERSON'})  #SKIP THE EMERGENCY DETECTOR


        smach.StateMachine.add( "SAY_PERSON_UNREACHABLE",
                                states.Say(robot,"I failed going to the person", block=False),
                                transitions={'spoken':'GO_TO_LAST_EXPLORATION_POINT'})

        smach.StateMachine.add( "GO_TO_LAST_EXPLORATION_POINT",
                                states.NavigateGeneric(robot, goal_query=query_last_exploration_location),
                                transitions={   "arrived":"SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE",
                                                    "unreachable":'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE',
                                                    "preempted":'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE',
                                                    "goal_not_defined":'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE'})
        

        smach.StateMachine.add( "SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE",
                                states.Say(robot,"I will ask my questions from here.", block=False),
                                transitions={'spoken':'RELOOK_AT_PERSON'})

        smach.StateMachine.add( 'RELOOK_AT_PERSON',
                                LookAtPerson(robot),                          
                                transitions={'finished':'APPROACH_PERSON'})  # SKIP THE EMERGENCY DETECTOR


        ''' NEW, NOT TESTED
        robot.reasoner.query(Compound("assertz", Compound("goal", Compound("emergency", room))))
        #query_accident_room = Conjunction(   Compound("goal", Compound("emergency", "Room")),
        #                                     Compound("waypoint", "Room", Compound("pose_2d", "X", "Y", "Phi")))

        query_accident_room = Conjunction(  Compound("=", "Waypoint", Compound(room, "W")),
                                                 Compound("waypoint", "Waypoint", Compound("pose_2d", "X", "Y", "Phi")),
                                                 Compound("not", Compound("visited", "Waypoint")),
                                                 Compound("not", Compound("unreachable", "Waypoint")))

        smach.StateMachine.add('GO_TO_ACCIDENT_ROOM', 
                                    states.Navigate_to_queryoutcome(robot, query_accident_room, X="X", Y="Y", Phi="Phi"),
                                    transitions={   'arrived':'RESET_HEAD', 
                                                    'preempted':'RESET_HEAD', 
                                                    'unreachable':'RESET_HEAD', 
                                                    'goal_not_defined':'RESET_HEAD'})

        smach.StateMachine.add("RESET_HEAD",
                                states.ResetHead(robot),
                                transitions={'done':'FIND_PERSON'})

        ######################################################
        #############   DETECT PERSON and LOOK  ##############
        ###################################################### 
        
        smach.StateMachine.add("FIND_PERSON",
                                LookingForPerson(robot),
                                transitions={   'Done':'LOOK_AT_PERSON',
                                                'Failed':'SAY_GO_TO_EXIT',
                                                'Aborted':'SAY_GO_TO_EXIT'})

        smach.StateMachine.add('LOOK_AT_PERSON',
                                LookAtPerson(robot),                          
                                transitions={'finished':'OBSERVE_EMERGENCY'})  
        '''
        ######################################################
        ################  OBSERVE EMERGENCY  #################
        ######################################################
        smach.StateMachine.add( "SAY_TURN_ON_EMERGENCY_DETECTOR",
                                states.Say(robot,["Waiting for emergency"], block=True),
                                transitions={'spoken':'TURN_ON_EMERGENCY_DETECTOR'})


        smach.StateMachine.add( "TURN_ON_EMERGENCY_DETECTOR",
                                SwitchEmergencyDetector(robot,switch=True),  
                                transitions={'done':'OBSERVE_EMERGENCY',
                                             'failed':'OBSERVE_EMERGENCY'})

        smach.StateMachine.add( "OBSERVE_EMERGENCY",
                                ObservingEmergency(robot), 
                                transitions={'done':'TURN_OFF_EMERGENCY_DETECTOR',
                                             'waiting':'WAITING_STATE',
                                             'expired':'TURN_OFF_EMERGENCY_DETECTOR'})

        smach.StateMachine.add( "WAITING_STATE",
                                SleepTime(robot, sleeptime=0.75), 
                                transitions={'finished':'OBSERVE_EMERGENCY'})

        smach.StateMachine.add( "TURN_OFF_EMERGENCY_DETECTOR",
                                SwitchEmergencyDetector(robot,switch=False), 
                                transitions={'done':'SAY_TURN_OFF_EMERGENCY_DETECTOR',
                                             'failed':'SAY_TURN_OFF_EMERGENCY_DETECTOR'})

        smach.StateMachine.add( "SAY_TURN_OFF_EMERGENCY_DETECTOR",
                                states.Say(robot,["Emergency detected"], block=False),
                                transitions={'spoken':'APPROACH_PERSON'})

        ######################################################
        ################   APPROACH PERSON   #################
        ######################################################
        smach.StateMachine.add( "APPROACH_PERSON",
                                    #states.NavigateGeneric(robot, lookat_query=person_query, xy_dist_to_goal_tuple=(0.8,0)),
                                    states.NavigateGeneric(robot, lookat_query=emergency_blob_query, xy_dist_to_goal_tuple=(0.8,0)),
                                    transitions={   "arrived":"RELOOK_AT_PERSON2",
                                                    "unreachable":'FAILED_DRIVING_TO_LOCATION',
                                                    "preempted":'FAILED_DRIVING_TO_LOCATION',
                                                    "goal_not_defined":'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE2'})

        smach.StateMachine.add( "FAILED_DRIVING_TO_LOCATION",
                                    states.Say(robot,"I was not able to reach the desired location of the person.", block=False),
                                    transitions={'spoken':'GO_TO_LAST_EXPLORATION_POINT2'})
        

        smach.StateMachine.add( "GO_TO_LAST_EXPLORATION_POINT2",
                                states.NavigateGeneric(robot, goal_query=query_last_exploration_location),
                                transitions={   "arrived":"SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE2",
                                                    "unreachable":'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE2',
                                                    "preempted":'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE2',
                                                    "goal_not_defined":'SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE2'})

        smach.StateMachine.add( "SAY_ASK_QUESTIONS_TO_PERSON_UNREACHABLE2",
                                    states.Say(robot,"I will ask my questions from here.", block=False),
                                    transitions={'spoken':'RELOOK_AT_PERSON2'})

        smach.StateMachine.add( 'RELOOK_AT_PERSON2',
                                    LookAtPerson(robot),                          
                                    transitions={'finished':'ASK_IF_AMBULANCE'})  

        ######################################################
        #############   ASK AMBU AND REGISTER   ##############
        ######################################################
        smach.StateMachine.add( 'ASK_IF_AMBULANCE',
                                    states.Say(robot, 'Do you need an ambulance?'),
                                    transitions={'spoken':'ANSWER_AMBULANCE'})

        smach.StateMachine.add( 'ANSWER_AMBULANCE',
                                    AskYesNo(robot),
                                    transitions={   'yes':'SAY_REGISTER_NOT_OKAY',            
                                                    'preempted':'SAY_REGISTER_NOT_OKAY',       
                                                    'no':'SAY_REGISTER_OKAY'})    

        smach.StateMachine.add( 'SAY_REGISTER_NOT_OKAY',
                                    states.Say(robot, 'I will register your position so the ambulance can find you.', block=True),
                                    transitions={'spoken':'REGISTER_PERSON_NOT_OKAY'})

        smach.StateMachine.add( 'SAY_REGISTER_OKAY',
                                    states.Say(robot, 'I will register your position.', block=True),
                                    transitions={'spoken':'REGISTER_PERSON_OKAY'})        

        smach.StateMachine.add( 'REGISTER_PERSON_NOT_OKAY',
                                    Register(robot, status=0),  #input 0 (person is not oke)
                                    transitions={'finished':'SAVE_PDF_ON_STICK'})

        smach.StateMachine.add( 'REGISTER_PERSON_OKAY',
                                    Register(robot, status=1),  #input 0 (person is oke)
                                    transitions={'finished':'SAVE_PDF_ON_STICK'})

        smach.StateMachine.add( 'SAVE_PDF_ON_STICK',
                                    RunPdfCreator(robot),
                                    transitions={'done':'ASK_FETCH',
                                                 'failed':'ASK_FETCH'})

        ######################################################
        ####################  ASK FETCH  #####################
        ######################################################
        smach.StateMachine.add('ASK_FETCH',
                                    AskObject(robot),
                                    transitions={"done":"LOOK_FOR_OBJECT",
                                                 "failed":"ASK_FETCH"})

        ######################################################
        ############## LOOK, GRAP AND DELIVER ################
        ######################################################
        smach.StateMachine.add( 'LOOK_FOR_OBJECT',
                                    LookForObject(robot),
                                    transitions={   "looking":"LOOK_FOR_OBJECT",
                                                    "found":'PICKUP_OBJECT',
                                                    "not_found":'SAY_OBJECT_NOT_FOUND'})

        arm = robot.leftArm

        smach.StateMachine.add( 'PICKUP_OBJECT',
                                    states.GrabMachine(arm, robot, grabpoint_query),
                                    transitions={   "succeeded":"RETURN_TO_PERSON",
                                                    "failed":'SAY_OBJECT_NOT_GRASPED' }) 
        
        smach.StateMachine.add( "RETURN_TO_PERSON",
                                    #states.NavigateGeneric(robot, lookat_query=general_person_query, xy_dist_to_goal_tuple=(0.8,0)),
                                    states.NavigateGeneric(robot, lookat_query=emergency_blob_query, xy_dist_to_goal_tuple=(0.8,0)),
                                    transitions={   "arrived":"HANDOVER_TO_HUMAN",
                                                    "unreachable":'HANDOVER_TO_HUMAN',
                                                    "preempted":'HANDOVER_TO_HUMAN',
                                                    "goal_not_defined":'HANDOVER_TO_HUMAN'})

        smach.StateMachine.add( 'SAY_OBJECT_NOT_GRASPED',
                                    states.Say(robot, ["I could not pick up the object you wanted", 
                                                       "I failed to grab the object you wanted."]),
                                    transitions={   'spoken':'HUMAN_HANDOVER' }) 

        smach.StateMachine.add( 'HUMAN_HANDOVER',
                                    states.Human_handover(arm,robot),
                                    transitions={   'succeeded':'RETURN_TO_PERSON',
                                                    'failed':'RETURN_TO_PERSON' })

        smach.StateMachine.add( 'SAY_OBJECT_NOT_FOUND',
                                    states.Say(robot, ["I could not find the object you wanted.", 
                                                       "I looked really hard, but I couldn't find your object."]),
                                    transitions={   'spoken':'SAY_GO_TO_EXIT' }) 

        smach.StateMachine.add("HANDOVER_TO_HUMAN",
                                    states.HandoverToHuman(arm, robot),
                                    transitions={   'succeeded':'SAY_GO_TO_EXIT',
                                                    'failed':'SAY_GO_TO_EXIT'})

        ######################################################
        ############# GO TO ENTRY OF APPARTMENT ##############
        ######################################################
        smach.StateMachine.add('SAY_GO_TO_EXIT',
                                    states.Say(robot, "I'm going to wait at the entry", block=False),
                                    transitions={'spoken':'GO_TO_EXIT'}) 

        # Amigo goes to the exit (waypoint stated in knowledge base)
        smach.StateMachine.add('GO_TO_EXIT', 
                                    states.NavigateGeneric(robot, goal_name="emergency_exit"),
                                    transitions={   'arrived':'WAIT_FOR_AMBULANCE', 
                                                    'preempted':'GO_TO_EXIT2', 
                                                    'unreachable':'GO_TO_EXIT2', 
                                                    'goal_not_defined':'GO_TO_EXIT2'})

        smach.StateMachine.add('GO_TO_EXIT2', 
                                    states.NavigateGeneric(robot, goal_name="emergency_exit2"),
                                    transitions={   'arrived':'WAIT_FOR_AMBULANCE', 
                                                    'preempted':'WAIT_FOR_AMBULANCE', 
                                                    'unreachable':'WAIT_FOR_AMBULANCE', 
                                                    'goal_not_defined':'WAIT_FOR_AMBULANCE'})

        ######################################################
        ################# WAIT FOR AMBULANCE ################# 
        ######################################################  
        smach.StateMachine.add('WAIT_FOR_AMBULANCE', 
                                    WaitForAmbulance(robot),
                                    transitions={   'detected':'SAY_FOLLOW_ME', 
                                                    'waiting':'WAIT_FOR_AMBULANCE', 
                                                    'failed':'SAY_FAILED_DETECTION',
                                                    'timed_out':'SAY_FOLLOW_ME'})

        smach.StateMachine.add('SAY_FOLLOW_ME',
                                    states.Say(robot, 'Please follow me to the patient.',block=False),
                                    transitions={'spoken':'GUIDE_TO_PERSON'})

        smach.StateMachine.add('SAY_FAILED_DETECTION',
                                    states.Say(robot, 'I could not detect the ambulance, please follow me to the patient whereever you are.',block=False),
                                    transitions={'spoken':'GUIDE_TO_PERSON'})

        ######################################################
        ################### DRIVE TO PERSON ##################
        ######################################################
        smach.StateMachine.add('GUIDE_TO_PERSON', 
                                    states.NavigateGeneric(robot, lookat_query=emergency_blob_query, xy_dist_to_goal_tuple=(0.8,0)),
                                    #states.NavigateGeneric(robot, lookat_query=emergency_person_query, xy_dist_to_goal_tuple=(0.8,0)),
                                    transitions={   "arrived":"SAY_ARRIVED_AT_PATIENT",
                                                    "unreachable":'SAY_ARRIVED_AT_PATIENT',
                                                    "preempted":'SAY_ARRIVED_AT_PATIENT',
                                                    "goal_not_defined":'FAILBACK_GUIDE_TO_PERSON'})

        smach.StateMachine.add('FAILBACK_GUIDE_TO_PERSON',
                                    states.NavigateGeneric(robot, goal_query=fallback_ambulance_query),
                                    transitions={   "arrived":"SAY_ARRIVED_AT_PATIENT",
                                                    "unreachable":"SAY_ARRIVED_AT_PATIENT",
                                                    "preempted":"SAY_ARRIVED_AT_PATIENT", 
                                                    "goal_not_defined":"SAY_ARRIVED_AT_PATIENT"})

        smach.StateMachine.add('SAY_ARRIVED_AT_PATIENT',
                                    states.Say(robot, 'The patient is here, thanks for the assistance.',block=False),
                                    transitions={'spoken':'SAVE_PDF_ON_STICK_FINAL'})

        ######################################################
        #####################  GO TO EXIT  ###################
        ###################################################### 
        # SKIPPED NOT NEEDED FOR CHALLENGE
        smach.StateMachine.add('EXIT_APPARTMENT', 
                                    states.NavigateGeneric(robot, goal_name="exit_1"),
                                    transitions={   'arrived':'SUCCEED_GO_TO_EXIT', 
                                                    'preempted':'FAILED_GO_TO_EXIT', 
                                                    'unreachable':'FAILED_GO_TO_EXIT', 
                                                    'goal_not_defined':'FAILED_GO_TO_EXIT'})

        smach.StateMachine.add('FAILED_GO_TO_EXIT',
                                    states.Say(robot, 'I was not able to go to the exit.', block=False),
                                    transitions={'spoken':'SAVE_PDF_ON_STICK_FINAL'})

        smach.StateMachine.add('SUCCEED_GO_TO_EXIT',
                                    states.Say(robot, 'I will now save all the information I gathered.', block=False),
                                    transitions={'spoken':'SAVE_PDF_ON_STICK_FINAL'})

        smach.StateMachine.add('SAVE_PDF_ON_STICK_FINAL',
                                    RunPdfCreator(robot),
                                    transitions={'done':'AT_END',
                                                 'failed':'AT_END'})

        ######################################################
        #####################  AT THE END  ###################
        ######################################################
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
