import smach, rospy, sys
from robot_smach_states.util.startup import startup
import robot_smach_states as states

import threading
import time

import math

from cb_planner_msgs_srvs.msg import *

from robot_skills.util import transformations

from robocup_knowledge import load_knowledge
knowledge = load_knowledge("challenge_restaurant")

class FollowOperatorAndStoreWaypoints(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done",'aborted','lost_operator'])

        self._robot = robot
        self._speech_recognition_thread = None
        self._speech_recognition_result = None
        
        self._waypoint_dict = {}
        
    def _save_kitchen(self):
        print "Saving kitchen"

    def _speech_recognition_thread_function(self):
        print "restarting speech recognition thread"
        
        self._speech_recognition_result = None
        
        # Filter if we heard number already
        choices = knowledge.guiding_choices
        choices["location"] = [ c for c in choices["location"] if c not in self._waypoint_dict ]
        
        self._speech_recognition_result = self._robot.ears.recognize(knowledge.guiding_spec, choices, time_out = rospy.Duration(100)) # Wait 100 secs
        
        self._speech_recognition_thread = None
        
    def _check_speech_result(self):
        # Check speech result
        if self._speech_recognition_result:
            
            # Stop the base
            self._robot.base.local_planner.cancelCurrentPlan()
            
            self._robot.speech.speak("I heard %s %s, is this allright?"%(self._speech_recognition_result.choices["location"], self._speech_recognition_result.choices["side"]))
            yes_no_result = self._robot.ears.recognize("<banana>", {"banana": ["yes", "no"]})
            if yes_no_result and yes_no_result.choices["banana"] == "yes":
                base_pose = self._robot.base.get_location().pose
                
                side = self._speech_recognition_result.choices["side"]
                
                if side == "left":
                    base_pose.orientation = transformations.euler_z_to_quaternion(transformations.euler_z_from_quaternion(base_pose.orientation) + math.pi / 2)
                elif side == "right":
                    base_pose.orientation = transformations.euler_z_to_quaternion(transformations.euler_z_from_quaternion(base_pose.orientation) - math.pi / 2)

                # Get position of the base
                self._waypoint_dict[self._speech_recognition_result.choices["location"]] = base_pose
                
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
            self._robot.ed.add_entity(id=waypoint_id, pose=waypoint, type=waypoint)
            
        return True
        
    def _restart_speech_recognition_thread(self):
        if not self._speech_recognition_thread:
            self._speech_recognition_thread = threading.Thread(target=self._speech_recognition_thread_function, args=())
            self._speech_recognition_thread.start()
            
    def _get_operator(self):
        operator = None
        
        while not operator:
            operator = self._robot.ed.get_entity(id= knowledge.operator_id)
            
            if not operator:
                self._robot.speech.speak("I lost my operator, please step in front of me!")
                
        return operator
        
    def _update_navigation(self, operator):
        self._robot.base.move(knowledge.navigation_position_constraint_operator, operator.id)
        
         
    def execute(self, userdata):
        while not rospy.is_shutdown():
            
            # TODO: uncomment Blocking if not operator present
            #operator = self._get_operator()

            # Check the speech result
            self._check_speech_result()
            
            # TODO: uncomment Update navigation
            #self._update_navigation(operator)

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
