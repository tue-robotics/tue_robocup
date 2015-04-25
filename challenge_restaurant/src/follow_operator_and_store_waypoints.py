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
        robot.base.local_planner.cancelCurrentPlan()
        self._speech_recognition_thread = None
        self._speech_recognition_result = None
        
        self._operator_id = None
        
        self._waypoint_dict = {}

        self._has_all_knowledge = False
        
    def _save_kitchen(self):
        self._robot.ed.update_entity(id="kitchen", posestamped=self._robot.base.get_location(), type="waypoint")

    def _speech_recognition_thread_function(self):
        print "restarting speech recognition thread"
        
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
            print self._speech_recognition_result
            if has_operator and result.result != "Please follow me" and 'location' in result.choices:
                self._heard_location(result.choices)
            elif not has_operator and result.result == "Please follow me":
                self._register_operator()

        self._speech_recognition_result = None
            
    def _heard_location(self, choices):
        # Stop the base
        self._robot.base.local_planner.cancelCurrentPlan()
        
        base_pose = self._robot.base.get_location()
        
        if not "side" in choices:
            return

        side = choices["side"]
        location = choices["location"]

        self._robot.speech.speak("%s %s, it is, moving on!"%(side, location))
        
        if side == "left":
            base_pose.pose.orientation = transformations.euler_z_to_quaternion(transformations.euler_z_from_quaternion(base_pose.pose.orientation) + math.pi / 2)
        elif side == "right":
            base_pose.pose.orientation = transformations.euler_z_to_quaternion(transformations.euler_z_from_quaternion(base_pose.pose.orientation) - math.pi / 2)

        # Get position of the base
        self._waypoint_dict[location] = base_pose
        
        print "Current waypoint dictionary:"
        print self._waypoint_dict
        
        self._robot.speech.speak("Very well, moving on!")
                
    def _check_all_knowledge(self):
        if self._has_all_knowledge:
            return True

        for c in knowledge.guiding_choices['location']:
            if c not in self._waypoint_dict:
                return False
                
        # Assert data to the world model
        for waypoint_id, waypoint in self._waypoint_dict.iteritems():
            print "Asserting waypoint %s to world model"%waypoint_id    
            # Adding entity does not work yet, should crash here
            self._robot.ed.update_entity(id=waypoint_id, posestamped=waypoint, type="waypoint")

        self._has_all_knowledge = True

        self._robot.speech.speak("I stored all places, please bring me back to the kitchen! ")
            
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

        if self._operator_id and not operator:
            self._robot.speech.speak("I lost my operator!")
            self._robot.base.local_planner.cancelCurrentPlan()
            self._operator_id = None
                
        return operator
        
    def _update_navigation(self, operator):
        if operator:
            self._robot.base.move(knowledge.navigation_position_constraint_operator, operator.id)

    def _back_in_kitchen(self):
        # Get the robot pose and compare if we are close enough to the kitchen waypoint
        kitchen = self._robot.ed.get_entity(id="kitchen")
        if kitchen:
            current = self._robot.base.get_location() 
            if math.hypot(current.pose.position.x - kitchen.pose.position.x, current.pose.position.y - kitchen.pose.position.y) < knowledge.kitchen_radius:
                return True
        else:
            print "NO KITCHEN IN ED???"
        return False

        
    def execute(self, userdata):
        self._save_kitchen()
        while not rospy.is_shutdown():
            
            # Check if operator present still present
            operator = self._get_operator(self._operator_id)

            # Check the speech result
            self._check_speech_result(self._speech_recognition_result, operator is not None)
            
            # Update the navigation
            self._update_navigation(operator)

            # Check if we have all knowledge already
            if not self._check_all_knowledge():
                # (Re)Start the speech recognition thread if not running 
                self._restart_speech_recognition_thread()
            else:
                if self._back_in_kitchen():
                    break

            rospy.sleep(1)

        
        self._robot.base.local_planner.cancelCurrentPlan()

        return "done"

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
