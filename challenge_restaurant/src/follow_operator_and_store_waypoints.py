import smach, rospy, sys
from robot_smach_states.util.startup import startup
import robot_smach_states as states

import threading
import time

import math

from cb_planner_msgs_srvs.msg import *

from robot_skills.util import transformations, msg_constructors

from robocup_knowledge import load_knowledge
knowledge = load_knowledge("challenge_restaurant")

class FollowOperatorAndStoreWaypoints(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done",'aborted'])

        self._robot = robot
        self._speech_recognition_thread = None
        self._speech_recognition_result = None
        
        self._operator_id = None
        
        self._waypoint_dict = {}
        
    def _save_kitchen(self):
        self._robot.ed.add_entity(id="kitchen", posestamped=self._robot.base.get_location(), type="waypoint")

    def _speech_recognition_thread_function(self):
        print "restarting speech recognition thread"
        
        self._speech_recognition_result = None
        
        # Filter if we heard number already
        choices = knowledge.guiding_choices
        choices["location"] = [ c for c in choices["location"] if c not in self._waypoint_dict ]
        
        self._speech_recognition_result = self._robot.ears.recognize(knowledge.guiding_spec, choices, time_out = rospy.Duration(100)) # Wait 100 secs
        
        self._speech_recognition_thread = None
        
    def _register_operator(self):
        operator = None
        while not operator:
            self._robot.speech.speak("Please stand in front of me!")
            rospy.sleep(2)
            operator = self._robot.ed.get_closest_entity(self, center_point=msg_constructors.PointStamped(x=1.7, y=0, z=0, frame_id="/%s/base_link"%self._robot.robot_name))
        print "We have a new operator: %s"%operator.id
        self._robot.speech.speak("Okay, show me around!")
        self._operator_id = operator.id
        
    def _check_speech_result(self, result, has_operator):
        # Check speech result
        if self._speech_recognition_result:
            if has_operator and result.result != "Please follow me":
                self._heard_location(result.choices)
            elif not has_operator and result.result == "Please follow me":
                self._register_operator()
            
    def _heard_location(self, choices):
        # Stop the base
        self._robot.base.local_planner.cancelCurrentPlan()
        
        self._robot.speech.speak("I heard %s %s, is this allright?"%(choices["location"], choices["side"]))
        yes_no_result = self._robot.ears.recognize("<banana>", {"banana": ["yes", "no"]})
        if yes_no_result and yes_no_result.choices["banana"] == "yes":
            base_pose = self._robot.base.get_location()
            
            side = choices["side"]
            
            if side == "left":
                base_pose.pose.orientation = transformations.euler_z_to_quaternion(transformations.euler_z_from_quaternion(base_pose.orientation) + math.pi / 2)
            elif side == "right":
                base_pose.pose.orientation = transformations.euler_z_to_quaternion(transformations.euler_z_from_quaternion(base_pose.orientation) - math.pi / 2)

            # Get position of the base
            self._waypoint_dict[choices["location"]] = base_pose
            
            print "Current waypoint dictionary:"
            print self._waypoint_dict
            
            self._robot.speech.speak("Very well, moving on!")
        else:
            self._robot.speech.speak("I am sorry, moving on!")
                
    def _check_all_knowledge(self):
        for c in knowledge.guiding_choices['location']:
            if c not in self._waypoint_dict:
                return False
                
        # Assert data to the world model
        for waypoint_id, waypoint in self._waypoint_dict.iteritems():
            print "Asserting waypoint %s to world model"%waypoint_id    
            # Adding entity does not work yet, should crash here
            self._robot.ed.add_entity(id=waypoint_id, posestamped=waypoint, type="waypoint")
            
        return True
        
    def _restart_speech_recognition_thread(self):
        if not self._speech_recognition_thread:
            self._speech_recognition_thread = threading.Thread(target=self._speech_recognition_thread_function, args=())
            self._speech_recognition_thread.start()
            
    def _get_operator(self, operator_id):        
        if self._operator_id:
            operator = self._robot.ed.get_entity(id=operator_id)
        else:
            operator = None
                
        return operator
        
    def _update_navigation(self, operator):
        self._robot.base.move(knowledge.navigation_position_constraint_operator, operator.id)
        
    def execute(self, userdata):
        while not rospy.is_shutdown():
            
            # Check if operator present still present
            operator = self._get_operator(self._operator_id)

            # Check the speech result
            self._check_speech_result(self._speech_recognition_result, operator is not None)
            
            if operator:
                self._update_navigation(operator)

            # Check if we have all knowledge already
            if self._check_all_knowledge():
                return "done"

            # (Re)Start the speech recognition thread if not running 
            self._restart_speech_recognition_thread()

            rospy.sleep(1)

        return "aborted"

# testing purposes
def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['done', 'aborted'])

    with sm:
        smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'FOLLOW_OPERATOR_AND_STORE_WAYPOINTS',
                                                'abort':'aborted'})
        smach.StateMachine.add('FOLLOW_OPERATOR_AND_STORE_WAYPOINTS', FollowOperatorAndStoreWaypoints(robot), transitions={ 'done' :'done', 'aborted' : 'aborted'})
    
    return sm


# TEST
if __name__ == '__main__':
    rospy.init_node('follow_operator')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[FOLLOW_OPERATOR] Please provide robot name as argument."
        exit(1)

    startup(setup_statemachine, robot_name=robot_name)
