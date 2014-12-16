#! /usr/bin/env python
import roslib;
import rospy
import smach

''' For siren demo challenge '''
import thread

from robot_smach_states.reasoning import Wait_query_true

from psi import Compound
import robot_skills.util.msg_constructors as msgs
import std_msgs.msg

class Initialize(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['initialized',
                                             'abort'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.lights.set_color(0,0,1)  #be sure lights are blue
        
        self.robot.head.reset()
        self.robot.leftArm.reset()
        self.robot.leftArm.send_gripper_goal('close',0.0)
        self.robot.rightArm.reset()
        self.robot.rightArm.send_gripper_goal('close',0.0)
        #self.robot.reasoner.reset()
        self.robot.spindle.reset()
        self.robot.base.reset_costmap()

        ## Check if TF link between /map and /base_link is set, if not error at initialize in stead of during first navigate execution
        rospy.loginfo("TF link between /map and /base_link is checked. If it takes longer than a second, probably an error. Do a restart!!!")
        self.robot.base.get_location()

        return 'initialized'

class SetInitialPose(smach.State):
    ## To call upon this state:
    # example 1: Set_initial_pose(robot, "front_of_door"),
    # OR
    # Set_initial_pose(robot, [1,0,0])
    def __init__(self, robot, init_position):       
        smach.State.__init__(self, outcomes=["done", "preempted", "error"])

        self.robot = robot
        self.preempted = False

        self.initial_position = init_position
    
    def location_2d(self, location):
        e_loc = self.robot.ed.get_entity(id=location)

        if not e_loc:
            rospy.logerr("SetInitialPose: ED entity '" + location + "' does not exist.")
            return []

        print e_loc

        try:
            rz = e_loc.data["pose"]["rz"]
        except KeyError:
            rz = 0

        return e_loc.pose.position.x, e_loc.pose.position.y, rz

    def execute(self, userdata):
        if isinstance(self.initial_position, str):
            x,y,phi = self.location_2d(self.initial_position)
        elif len(self.initial_position) == 3: #Tuple or list        
            x = self.initial_position[0]
            y = self.initial_position[1]
            phi = self.initial_position[2]
        else:
            rospy.logerr("Initial pose {0} could not be set".format(self.initial_position))
            return "error"

        rospy.loginfo('Set initial pose to {0}, {1}, {2}'.format(x, y, phi))

        self.robot.base.set_initial_pose(x, y, phi)
        
        # Reset costmap: costmap is obviously entirely off if the localization was wrong before giving the initial pose
        self.robot.base.reset_costmap()
        # Wait 0.5 s just to be sure
        rospy.sleep(rospy.Duration(0.5))

        return "done"

############################## Wait for trigger ##############################
class WaitForTrigger(smach.State):
    '''
    This state will block execution until a suitable trigger command is received on the channel /trigger
    It will receive std_msgs.String and will compare it to the strings in the array that is given.

    Example to wait for one of the strings 'allow' or 'deny' (could be sent from a gui):

        WaitForTrigger(robot, ['allow', 'deny']),
                       transitions={    'allow':     'DO_SOMETHING',
                                        'deny':      'DO_SOMETHING',
                                        'preempted': 'failed'})
    '''

    def __init__(self, robot, triggers):
        smach.State.__init__(self, 
                             outcomes=triggers+['preempted'])
        self.robot = robot
        self.triggers = triggers

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
            return self.trigger_received
        else:
            return 'preempted'

    def callback(self, data):
        # Simply print out values in our custom message.
        if data.data in self.triggers:
            rospy.loginfo('trigger received: %s', data.data)
            self.trigger_received = data.data
        else:
            rospy.logwarn('wrong trigger received: %s', data.data)

############################## State Wait ##############################
class Wait_time(smach.State):
    def __init__(self, robot=None, waittime=10):
        smach.State.__init__(self, outcomes=['waited','preempted'])
        self.robot = robot
        self.waittime = waittime
    
    def execute(self, *args, **kwargs):
        total_sleep = 0
        sleep_interval = 0.1
        #import ipdb; ipdb.set_trace()
        while total_sleep <= self.waittime:
            rospy.sleep(sleep_interval)
            total_sleep += sleep_interval
            if self.preempt_requested():
                rospy.loginfo('Wait_time preempted at {0} of {1}'.format(total_sleep, self.waittime))
                self.service_preempt()
                return 'preempted'
        return 'waited'

class Wait_Condition(smach.State):
    '''Wait until a condition is satisfied, possible on a robot. 
    When the condtion is satisfied, the value that matched the condition is stored in the userdata.
    The callback must return that value or something that evaluates to False otherwise.
    The arguments to the callback are userdata, robot'''
    def __init__(self, robot, condition_callback, timeout):
        ''' '''
        smach.State.__init__(self,
                             outcomes=['satisfied', 'timed_out', 'preempted'],
                             output_keys=['trigger_value'])
        self.condition_callback = condition_callback
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata):
        starttime = rospy.Time.now()

        while (rospy.Time.now() - starttime) < rospy.Duration(self.timeout)\
         and not rospy.is_shutdown():
            cb_output = self.condition_callback(userdata, self.robot)
            if cb_output:
                userdata.trigger_value = cb_output
                return "satisfied"
            if self.preempt_requested():
                self.service_preempt()
                return "preempted"
            rospy.sleep(0.1)
        return 'timed_out'

class Finish(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self,
                                   outcomes=['stop'])
        self.robot = robot
    
    def execute(self, gl):
        
        print "Finished executing task."
        return 'stop'
    
# class FinishOld(smach.State):
#     def __init__(self, robot=None):
#         smach.State.__init__(self,
#                                    outcomes=['stop'],input_keys=['challenge','start_time'])
#         self.robot = robot
    
#     def execute(self, gl):
        
#         duration = calculate_duration(gl.start_time)
#         print "Finished executing task", gl.challenge, "in", duration, "seconds."
#         return 'stop'

class PlaySound(smach.State):
    def __init__(self, filename, blocking=False):
        smach.State.__init__(self, outcomes=['played','error'])
        self.file = filename
        self.blocking = blocking
        
    def execute(self, ud):
        import os
        extension = os.path.splitext(self.file)[1]
        player = {".mp3":"mpg123", ".wav":"aplay"}[extension]
        command = (player, self.file)
        try:
            rospy.loginfo("Running '{0}'".format(*command))
            if self.blocking:
                os.system("{0} {1}".format(*command))
            else:
                thread.start_new_thread(os.system,("".join(command)))
            return "played"
        except Exception, e:
            rospy.logerr(e)
            return "error"

class SetTimeMarker(smach.State):
    def __init__(self, robot, name):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.name = name

    def execute(self, userdata=None):
        self.robot.reasoner.set_time_marker(self.name)
        return "done"

class CheckTime(smach.State):
    def __init__(self, robot, name, max_duration):
        smach.State.__init__(self, outcomes=["ok", "timeout"])
        self.robot = robot
        self.max_duration = max_duration
        self.name = name

    def execute(self, userdata=None):
        if self.robot.reasoner.get_time_since(self.name) > self.max_duration:
            return "timeout"
        else:
            return "ok"

class Wait_queried_perception(Wait_query_true):
    """Toggle the given perception module(s) and wait for its perceptions to match the given query
    E.g. toggle the people detection and wait until a person is perceived. Then toggle the module off again."""

    def __init__(self, robot, modules, query, timeout=5):
        """init the superstate with custom pre/post callbacks dependent on these args"""
        if isinstance(modules, str):
            modules = [modules] #Make it a list

        pre = lambda *args, **kwargs: robot.perception.toggle(modules)
        post = lambda *args, **kwargs: robot.perception.toggle([])
        Wait_query_true.__init__(self, robot, query, timeout=timeout, pre_callback=pre, post_callback=post)

class LookAtHand(smach.State):
    def __init__(self, robot, side, keep_tracking=False, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout
        self.side = side
        self.keep_tracking = keep_tracking

    def execute(self, userdata=None):
        side_string = {self.robot.leftArm:"left", self.robot.rightArm:"right"}[self.side]
        self.robot.head.look_at_hand(side_string, keep_tracking=self.keep_tracking) #TODO: Unify side as string or/and object
        return "done"

############################## Atomic Reset States ##############################

class ResetHead(smach.State):
    def __init__(self, robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata=None):

        self.robot.head.reset_position(timeout=self.timeout)
        return "done"

class ResetLeftArm(smach.State):
    def __init__(self, robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata=None):

        self.robot.leftArm.reset_arm()
        self.robot.leftArm.send_gripper_goal_close(timeout=self.timeout)
        return "done"

class ResetRightArm(smach.State):
    def __init__(self, robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata=None):

        self.robot.rightArm.reset_arm()
        self.robot.rightArm.send_gripper_goal_close(timeout=self.timeout)
        return "done"

class ResetArms(smach.State):
    def __init__(self, robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata=None):

        self.robot.leftArm.reset()
        self.robot.leftArm.send_gripper_goal_close(timeout=self.timeout)
        self.robot.rightArm.reset()
        self.robot.rightArm.send_gripper_goal_close(timeout=self.timeout)
        return "done"

class ResetArm(smach.State):
    def __init__(self, robot, side, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout
        self.side = side

    def execute(self, userdata=None):
        self.side.reset_arm()
        self.side.send_gripper_goal_close(timeout=self.timeout)
        return "done"

class ResetSpindle(smach.State):
    def __init__(self, robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata=None):

        self.robot.spindle.reset()
        return "done"

class SetSpindle(smach.State):
    def __init__(self, robot, timeout=0.0, height=0.35):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout
        self.height = height

    def execute(self, userdata=None):
        self.robot.spindle.send_goal(self.height)
        return "done"
############################## Combination Reset States ##############################

class ResetArmsSpindle(smach.State):
    def __init__(self, robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata=None):

        self.robot.leftArm.reset_arm()
        self.robot.leftArm.send_gripper_goal_close(timeout=self.timeout)
        self.robot.rightArm.reset_arm()
        self.robot.rightArm.send_gripper_goal_close(timeout=self.timeout)
        self.robot.spindle.reset()
        return "done"

class ResetArmsHead(smach.State):
    def __init__(self, robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata=None):

        self.robot.leftArm.reset_arm()
        self.robot.leftArm.send_gripper_goal_close(timeout=self.timeout)
        self.robot.rightArm.reset_arm()
        self.robot.rightArm.send_gripper_goal_close(timeout=self.timeout)
        self.robot.head.reset_position(timeout=self.timeout)
        return "done"

class ResetHeadSpindle(smach.State):
    def __init__(self, robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata=None):

        self.robot.spindle.reset()
        self.robot.head.reset_position(timeout=self.timeout)
        return "done"

class ResetArmsSpindleHead(smach.State):
    # Checks how many tasks have been done and if another task is needed
    # Does this check with the database in the reasoner
    def __init__(self,robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])

        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata):

        self.robot.leftArm.reset_arm()
        self.robot.leftArm.send_gripper_goal_close(timeout=self.timeout)
        self.robot.rightArm.reset_arm()
        self.robot.rightArm.send_gripper_goal_close(timeout=self.timeout)
        self.robot.head.reset_position(timeout=self.timeout)
        self.robot.spindle.reset()

        return "done"

class ResetSpindle_HeadUp(smach.State):
    def __init__(self, robot, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata=None):

        self.robot.spindle.reset()
        self.robot.head.look_up()
        return "done"

############################## RESET REASONER State ##############################

class ResetReasoner(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
    def execute(self, userdata=None):
        self.robot.reasoner.reset()
        return "done"

