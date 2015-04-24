import smach, rospy, sys
from robot_smach_states.util.startup import startup
import robot_smach_states as states

import threading
import time

from cb_planner_msgs_srvs.msg import *

from robocup_knowledge import load_knowledge
knowledge = load_knowledge("challenge_restaurant")

WAYPOINT_DICT = {}

class FollowOperatorAndStoreWaypoints(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done",'aborted'])

        self._robot = robot
        self._speech_recognition_thread = None
        self._speech_recognition_result = None

    def _speech_recognition_thread_function(self):
        print "restarting speech recognition thread"
        
        self._speech_recognition_result = None
        
        # Filter if we heard number already
        choices = knowledge.guiding_choices
        choices["number"] = [ c for c in choices["number"] if c not in WAYPOINT_DICT ]
        
        self._speech_recognition_result = self._robot.ears.recognize(knowledge.guiding_spec, choices, time_out = rospy.Duration(100)) # Wait 100 secs
        
        self._speech_recognition_thread = None
        
    def _check_speech_result(self):
        # Check speech result
        if self._speech_recognition_result:
            
            # Stop the base
            self._robot.base.local_planner.cancelCurrentPlan()
            
            self._robot.speech.speak("I heard %s %s, is this allright?"%(self._speech_recognition_result.choices["number"], self._speech_recognition_result.choices["side"]))
            yes_no_result = self._robot.ears.recognize("<banana>", {"banana": ["yes", "no"]})
            if yes_no_result and yes_no_result.choices["banana"] == "yes":
                # Get position of the base
                WAYPOINT_DICT[self._speech_recognition_result.choices["number"]] = self._robot.base.get_location()
                
                print "Current waypoint dictionary:"
                print WAYPOINT_DICT
                
                self._robot.speech.speak("Very well, moving on!")
            else:
                self._robot.speech.speak("I am sorry, moving on!")
                
    def _check_all_knowledge(self):
        return "one" in WAYPOINT_DICT and "two" in WAYPOINT_DICT and "three" in WAYPOINT_DICT
        
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
        
    def __update_navigation(self):
        self._robot.base.move(knowledge.navigation_position_constraint, knowledge.operator_id)
        
         
    def execute(self, userdata):
        while not rospy.is_shutdown():
            
            # Blocking if not operator present
            operator = self._get_operator()

            # Check the speech result
            self._check_speech_result()
            
            # Update navigation
            self._update_navigation()

            # Check if we have all knowledge already
            if self._check_all_knowledge():
                return "done"

            # (Re)Start the speech recognition thread if not running 
            self._restart_speech_recognition_thread()

            rospy.sleep(1)

        return "aborted"

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
