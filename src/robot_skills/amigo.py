#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
import head
#import worldmodel
import base
import spindle
import speech
import arms
import perception
import ears
import ebutton
import lights

import tf

from util import transformations
#from components import message_helper
import geometry_msgs
import std_msgs.msg

from math import degrees, radians

# This is very ugly, but needed for mapgo, as it depends on robot_smach_states
# It is not possible to add robot_smach_states to the manifest of robot_skills, as
# robot_smach_states already depends on robot_skills (infinite recursion!)
# TODO: find better solution
roslib.load_manifest('robot_smach_states')

class Amigo(object):
    """
    Interface to all parts of Amigo. When initializing Amigo, you can choose a list of components
    which wont be needed
    
    # I want a blind and mute Amigo!
    >>> Amigo(['perception', 'speech'])
    
    # I want a full fledged, awesome Amigo
    >>> Amigo()
    """
    def __init__(self, dontInclude = [], wait_services=False):
        
        self.tf_listener = tf.TransformListener()

        if 'head' not in dontInclude:
            self.head = head.Head()
        # if 'worldmodel' not in dontInclude:
        #     self.worldmodel = worldmodel.WorldModel(self.tf_listener)
        if 'base' not in dontInclude:
            self.base = base.Base(self.tf_listener, wait_service=wait_services,use_2d=True)
        if 'spindle' not in dontInclude:
            self.spindle = spindle.Spindle(wait_service=wait_services)
        if 'speech' not in dontInclude:
            self.speech = speech.Speech(wait_service=wait_services)
        if 'arms' not in dontInclude:
            self.arms = arms.Arms(self.tf_listener) #TODO: use self.tf_listener here
        if 'leftArm' not in dontInclude:
            self.leftArm = arms.Arm(arms.Side.LEFT, self.tf_listener)
        if 'rightArm' not in dontInclude:
            self.rightArm = arms.Arm(arms.Side.RIGHT, self.tf_listener)
        if 'perception' not in dontInclude:
            try:
                self.perception = perception.Perception(wait_service=wait_services)
            except:
                rospy.logwarn("Perception could not be initialized. Is the providing node running?")
        if 'ears' not in dontInclude:
            self.ears = ears.Ears()
        if 'ebutton' not in dontInclude:
            self.ebutton = ebutton.EButton()
        if 'lights' not in dontInclude:
            self.lights = lights.Lights()
        if 'reasoner' not in dontInclude:
            try:
                import reasoner
                self.reasoner = reasoner.Reasoner()
            except ImportError:
                rospy.logwarn("Reasoner could not be imported into Amigo")
        
        self.leftSide = arms.Side.LEFT
        self.rightSide = arms.Side.RIGHT
        self.pub_target = rospy.Publisher("/target_location", geometry_msgs.msg.Pose2D)
 
 #These are deprecated       
    # def look_at(self, worldmodel_target, _timeout=10.0):
    #     """
    #     Look at a specific target from the worldmodel
    #     Expects a worldmodel_target. Optional time_out
        
    #     Example:
    #     >>> import roslib; roslib.load_manifest('robot_skills')
    #     >>> import object_msgs
    #     >>> o = object_msgs.msg.ExecutionTarget(ID=1, class_label='MarkEnSimon')
    #     >>> amigo.look_at(o) 
    #     """
    #     target_position = self.worldmodel.determine_target_position(worldmodel_target)
    #     if target_position:
    #         rospy.loginfo("Looking at worldmodel_target {0}".format(worldmodel_target))
    #         return self.head.send_goal(target_position, timeout=_timeout)
    #     else:
    #         return False
        
    # def look_at_id(self, id, _timeout=10):
    #     """
    #     Look at a specific ID from the worldmodel.
    #     Expects an ID and optional _time_out
        
    #     Example:
    #     # Over 9000?
    #     >>> amigo.look_at_id(9001)
    #     """
    #     target_position = self.worldmodel.determine_target_position_by_ID(id)
    #     if target_position:
    #         rospy.loginfo("Looking at worldmodel_target ID {0}".format(id))
    #         return self.head.send_goal(target_position, timeout=_timeout)
    #     else:
    #         return False
        
    # def closest_target(self, ID=None, class_label=None, name=None):
    #     matches = self.worldmodel.search_target(ID, class_label, name)
        
    #     if matches:
    #         base_loc = self.base.location[0]
    #         #select the one whose .pose.position is closest to self.base.location[0]
    #         closest = min(matches, key=lambda obj: transformations.compute_distance(obj.pose.position, base_loc))
    #         return closest
    #     else:
    #         return None
        
    # def target_closer_than(self, distance, ID=None, class_label=None, name=None):
    #     target = self.closest_target(ID, class_label, name)
    #     if target == None:
    #         return False
    #     d = transformations.compute_distance(target.pose.position, self.base.location[0])
    #     rospy.logdebug("[target_closer_than] Distance {0} and max {1} is {2}".format(d, distance, d < distance))
    #     return d < distance
    
    def publish_target(self, x, y):
        self.pub_target.publish(geometry_msgs.msg.Pose2D(x, y, 0))

    # def tf_transform(self, coordinates, inputframe, outputframe):
    #     ps = geometry_msgs.msg.PointStamped(point= coordinates) 
    #     ps.header.frame_id = inputframe
    #     ps.header.stamp = rospy.Time()
    #     output_coordinates = self.tf_listener.transformPoint(outputframe, ps)
    #     return output_coordinates.point
        
    def tf_transform_pose(self, ps,frame):
        output_pose = geometry_msgs.msg.PointStamped
        self.tf_listener.waitForTransform(frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        output_pose = self.tf_listener.transformPose(frame, ps) 
        return output_pose

    def close(self):
        try:
            self.head.close()
        except: pass
        # try:
        #     self.worldmodel.close()
        # except: pass

        try:
            self.base.close()
        except: pass

        try:
            self.spindle.close()
        except: pass

        try:
            self.speech.close()
        except: pass

        try:
            self.arms.close()
        except: pass

        try:
            self.leftArm.close()
        except: pass

        try:
            self.rightArm.close()
        except: pass

        try:
            self.perception.close()
        except: pass

        try:
            self.ears.close()
        except: pass

        try:
            self.ebutton.close()
        except: pass

        try:
            self.lights.close()
        except: pass

        try:
            self.reasoner.close()
        except: pass

    def __enter__(self):
        pass

    def __exit__(self, exception_type, exception_val, trace):
        if any((exception_type, exception_val, trace)):
            rospy.logerr("Amigo exited with {0},{1},{2}".format(exception_type, exception_val, trace))
        self.close()

if __name__ == "__main__":
    import atexit
    import util.msg_constructors as msgs
    from reasoner import Compound, Conjunction

    rospy.init_node("amigo_executioner", anonymous=True)
    amigo = Amigo(wait_services=False)
    robot = amigo #All state machines use robot. ..., this makes copy/pasting easier.

    atexit.register(amigo.close) #When exiting the interpreter, call amigo.close(), which cancels all action goals etc.

    head_reset = lambda: amigo.head.reset_position()
    head_down  = lambda: amigo.head.look_down()
    right_close = lambda: amigo.rightArm.send_gripper_goal_close()
    left_close = lambda: amigo.leftArm.send_gripper_goal_close()
    right_open = lambda: amigo.rightArm.send_gripper_goal_open()
    left_open = lambda: amigo.leftArm.send_gripper_goal_open()
    speak = lambda sentence: amigo.speech.speak(sentence)
    praat = lambda sentence: amigo.speech.speak(sentence, language='nl')
    look_at_point = lambda x, y, z: amigo.head.send_goal(amigo.head.point(x, y, z), frame_id="/base_link")
    
    mapgo_old = amigo.base.go
    def mapgo(x,y,phi, dist=1.5):
        from robot_smach_states import navigation       
        nav_state = navigation.NavigateGeneric(robot, goal_pose_2d=(x,y,phi), look_at_path_distance=dist)
        return nav_state.execute()
    
    r = amigo.reasoner
    open_door   = lambda: r.assertz(r.state("door1", "open"))
    close_door  = lambda: r.assertz(r.state("door1", "close"))
    def insert_object(x,y,z):
        from test_tools.WorldFaker import WorldFaker
        wf = WorldFaker()
        wf.insert({"position":(x,y,z)})

    def basego(x,y,phi):
        return amigo.base.go(x,y,phi,frame="/base_link")

    #Useful for making location-files
    def get_pose_2d():
       loc,rot = amigo.base.location
       rot_array = [rot.w, rot.x, rot.y, rot.z]
       rot3 = tf.transformations.euler_from_quaternion(rot_array)
       print '''    - frame_id: "/map"
      name: "INSERT"
      class_label: "location"
      x: {0}
      y: {1}
      phi: {2}
      
    '''.format(loc.x, loc.y, rot3[0])
       return (loc.x, loc.y, rot3[0]) 

    def hear(text):
        pub = rospy.Publisher('/speech/output', std_msgs.msg.String)
        rospy.loginfo("Telling Amigo '{0}'".format(text))
        pub.publish(std_msgs.msg.String(text))

    print """\033[1;33m You can now command amigo from the python REPL. 
    Type e.g. help(amigo) for help on objects and functions, 
    or type 'amigo.' <TAB> to see what methods are available. 
    Press ctrl-D or type 'exit()' to exit the awesome amigo-console.
    WARNING: When the console exits, it cancels ALL arm and base goals. \n
    There are handy shortcuts, such as: mapgo/basego(x,y,phi), speak(sentence), 
    head_down, head_reset, left/right_close/open, look_at_point(x,y,z), \n
    get_pose_2d() etc. Also, try 'dir()'.\n 
    You can use both robot.foo or amigo.foo for easy copypasting.\033[1;m \n"""
