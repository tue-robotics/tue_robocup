#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy
import head
#import worldmodel
import base
import base2
import spindle
import speech
import arms
import perception
import ears
import ebutton
import lights

import tf
import tf_server

from util import transformations
#from components import message_helper
import geometry_msgs
import std_msgs.msg

from math import degrees, radians

import amigo_inverse_reachability.srv
from psi import Compound, Sequence, Conjunction

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
        
        self._get_base_goal_poses = rospy.ServiceProxy('/inverse_reachability/inverse_reachability', amigo_inverse_reachability.srv.GetBaseGoalPoses)

        self.tf_listener = tf_server.TFClient()

        if 'head' not in dontInclude:
            self.head = head.Head()
        # if 'worldmodel' not in dontInclude:
        #     self.worldmodel = worldmodel.WorldModel(self.tf_listener)
        if 'base2' not in dontInclude:
            self.base2 = base2.Base(self.tf_listener, wait_service=wait_services) # Added by Rein (new nav interface)
        if 'base' not in dontInclude:
            self.base = base.Base(self.tf_listener, wait_service=wait_services, use_2d=None) #None indicates: sort it out yourselve
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
    
    def publish_target(self, x, y):
        self.pub_target.publish(geometry_msgs.msg.Pose2D(x, y, 0))
        
    def tf_transform_pose(self, ps,frame):
        output_pose = geometry_msgs.msg.PointStamped
        self.tf_listener.waitForTransform(frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        output_pose = self.tf_listener.transformPose(frame, ps) 
        return output_pose

    def store_position_knowledge(self, label, dx=0.78, z=0.75, drop_dz=0.13, filename='/tmp/locations.pl'):

        # Query reasoner for environment name
        ans_env = self.reasoner.query(Compound("environment", "Env"))

        if not ans_env:
            rospy.logerr("Could not get environment name from reasoner.")
            return

        env_name = ans_env[0]["Env"]
        
        # Determine base position as (x, y, phi)
        pose = self.base.location
        (pos, quat) = pose.pose.position, pose.pose.orientation
        phi = self.base.phi(quat)

        base_pose = Compound("waypoint", env_name, "Challenge", label, Compound("pose_2d", round(pos.x, 3), round(pos.y, 3), round(phi, 3)))
        print base_pose

        # Determine lookat point (point of interest)

        time = rospy.Time.now()

        ps = geometry_msgs.msg.PointStamped()
        ps.header.stamp = time
        ps.header.frame_id = "/amigo/base_link"
        ps.point.x = dx # dx meter in front of robot
        ps.point.y = 0.0
        ps.point.z = z

        self.tf_listener.waitForTransform("/map", ps.header.frame_id, time, rospy.Duration(2.0))
        ps_MAP = self.tf_listener.transformPoint("/map", ps)

        poi = Compound("point_of_interest", env_name, "Challenge", label, Compound("point_3d", round(ps_MAP.point.x, 3), round(ps_MAP.point.y, 3), round(ps_MAP.point.z, 3)))
        print poi

        dropoff = Compound("dropoff_point", env_name, "Challenge", label, Compound("point_3d", round(ps_MAP.point.x, 3), round(ps_MAP.point.y, 3), round(ps_MAP.point.z + drop_dz, 3)))
        print dropoff        

        with open(filename, "a") as myfile:
            myfile.write(str(base_pose) + ".\n")
            myfile.write(str(poi) + ".\n")
            myfile.write(str(dropoff) + ".\n")

        # assert the facts to the reasoner
        #self.reasoner.query(Compound("assert", base_pose))
        #self.reasoner.query(Compound("assert", poi))

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

    # This function was originally located in base.py, but since the reasoner is needed to get room dimentions, it is placed here.
    def get_base_goal_poses(self, target_point_stamped, x_offset, y_offset, cost_threshold_norm=0.2):

        request = amigo_inverse_reachability.srv.GetBaseGoalPosesRequest()
        
        request.robot_pose = self.base.location.pose

        request.target_pose.position = target_point_stamped.point
        request.target_pose.orientation.x = 0
        request.target_pose.orientation.y = 0
        request.target_pose.orientation.z = 0
        request.target_pose.orientation.w = 1

        request.cost_threshold_norm = cost_threshold_norm
        
        request.x_offset = x_offset
        request.y_offset = y_offset
        rospy.logdebug("Inverse reachability request = {0}".format(request).replace("\n", " ").replace("\t", " "))
        response = self._get_base_goal_poses(request)
        rospy.logdebug("Inverse reachability response = {0}".format(response).replace("\n", " ").replace("\t", " "))
        
        ## Only get poses that are in the same room as the grasp point.
        
        rooms_dimensions = self.reasoner.query(Compound("room_dimensions", "Room", Compound("size", "Xmin", "Ymin", "Zmin", "Xmax", "Ymax", "Zmax")))
        
        if rooms_dimensions:
            for x in range(0,len(rooms_dimensions)):
                room_dimensions = rooms_dimensions[x]
                if (target_point_stamped.point.x > float(room_dimensions["Xmin"]) and target_point_stamped.point.x < float(room_dimensions["Xmax"]) and target_point_stamped.point.y > float(room_dimensions["Ymin"]) and target_point_stamped.point.y < float(room_dimensions["Ymax"])):
                    rospy.loginfo("Point for inverse reachability in room: {0}".format(str(room_dimensions["Room"])))
                    rospy.sleep(2)
                    break
                else:
                    room_dimensions = ""

        if rooms_dimensions and room_dimensions:

            x_min = float(room_dimensions["Xmin"])
            x_max = float(room_dimensions["Xmax"])
            y_min = float(room_dimensions["Ymin"])
            y_max = float(room_dimensions["Ymax"])
            #print "x_min = ", x_min, ", x_max = ", x_max, ", y_min = ", y_min, ", y_max = ", y_max, "\n"
            base_goal_poses = []
            for base_goal_pose in response.base_goal_poses:

                x_pose = base_goal_pose.position.x
                y_pose = base_goal_pose.position.y
                # print "x_pose = ", x_pose, ", y_pose = ", y_pose, "\n"
                if (x_pose > x_min and x_pose < x_max and y_pose > y_min and y_pose < y_max):
                    # print "Pose added\n"
                    base_goal_poses.append(geometry_msgs.msg.PoseStamped())
                    base_goal_poses[-1].header.frame_id = "/map"
                    base_goal_poses[-1].header.stamp = rospy.Time()
                    base_goal_poses[-1].pose = base_goal_pose            
                # else:
                    # print "point deleted\n"
            # print "AFTER base_goal_poses length = ", len(base_goal_poses), "\n"

        else:
            base_goal_poses = []

            for base_goal_pose in response.base_goal_poses:
                base_goal_poses.append(geometry_msgs.msg.PoseStamped())
                base_goal_poses[-1].header.frame_id = "/map"
                base_goal_poses[-1].header.stamp = rospy.Time()
                base_goal_poses[-1].pose = base_goal_pose

        return base_goal_poses

if __name__ == "__main__":
    import atexit
    import util.msg_constructors as msgs
    from reasoner import Compound, Conjunction, Sequence, Variable

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
    speak = lambda sentence: amigo.speech.speak(sentence, block=False)
    praat = lambda sentence: amigo.speech.speak(sentence, language='nl', block=False)
    look_at_point = lambda x, y, z: amigo.head.send_goal(msgs.PointStamped(x, y, z, frame_id="/amigo/base_link"))
        
    r = amigo.reasoner
    q = amigo.reasoner.query

    mapgo = amigo.base.go

    def airgo(x,y,z, xoffset=0.5, yoffset=0.1):
        target = amigo.base.point(x,y,z, stamped=True)
        ik_poses = amigo.base.get_base_goal_poses(target, xoffset, yoffset)
        amigo.base.send_goal(ik_poses[0])
        amigo.head.send_goal(msgs.PointStamped(x,y,z))

    def namego(name):
        """Go to the waypoint with the given name. 
        When there re multiple locations with the samen name, shame on you for having messed up locations, and Amigo will go to the first query result"""
        query = Compound("waypoint", name, Compound("pose_2d", "X", "Y", "Phi"))
        answers = amigo.reasoner.query(query)
        try:
            selected = answers[0]
        except IndexError:
            rospy.logerr("No named location {0} (name = {1})".format(query, name))
            return None
        x,y,phi = float(selected["X"]), float(selected["Y"]), float(selected["Phi"])
        return amigo.base.go(x,y,phi)
    
    def basego(x,y,phi):
        return amigo.base.go(x,y,phi,frame="/amigo/base_link")

    open_door   = lambda: r.assertz(r.state("door1", "open"))
    close_door  = lambda: r.assertz(r.state("door1", "close"))
    def insert_object(x,y,z):
        from test_tools.WorldFaker import WorldFaker
        wf = WorldFaker()
        wf.insert({"position":(x,y,z)})

    #Useful for making location-files
    def get_pose_2d():
       posestamped = amigo.base.location
       loc,rot = posestamped.pose.point, posestamped.pose.orientation
       rot_array = [rot.w, rot.x, rot.y, rot.z]
       rot3 = tf.transformations.euler_from_quaternion(rot_array)
       print 'x={0}, y={1}, phi={2}'.format(loc.x, loc.y, rot3[0])
       return (loc.x, loc.y, rot3[0]) 

    def hear(text):
        pub = rospy.Publisher('/pocketsphinx/output', std_msgs.msg.String)
        rospy.logdebug("Telling Amigo '{0}'".format(text))
        pub.publish(std_msgs.msg.String(text))

    def save_sentence(sentence):
        """Let amigo say a sentence and them move the generated speech file to a separate file that will not be overwritten"""
        speak(sentence)
        import os
        path = sentence.replace(" ", "_")
        path += ".wav"
        os.system("mv /tmp/speech.wav /tmp/{0}".format(path))

    def test_audio():
        OKGREEN = '\033[92m'
        ENDC = '\033[0m'

        amigo.speech.speak("Please say: continue when I turn green", block=True) #definitely block here, otherwise we hear ourselves
        
        result = amigo.ears.ask_user("continue")
        rospy.loginfo("test_audio result: {0}".format(result))
        if result and result != "false":
            rospy.loginfo(OKGREEN+"Speach pipeline working"+ENDC)
            amigo.speech.speak("Yep, my ears are fine", block=False)
        else:
            rospy.logerr("Speech pipeline not working")
            amigo.speech.speak("Nope, my ears are clogged", block=False)


    print """\033[1;33m You can now command amigo from the python REPL. 
    Type e.g. help(amigo) for help on objects and functions, 
    or type 'amigo.' <TAB> to see what methods are available. 
    Also, try 'dir()'
    You can use both robot.foo or amigo.foo for easy copypasting.
    Press ctrl-D or type 'exit()' to exit the awesome amigo-console.
    WARNING: When the console exits, it cancels ALL arm and base goals.
    There are handy shortcuts, such as: 
        - mapgo/basego(x,y,phi), 
        - airgo(x,y,z) for AmigoInverseReachability, 
        - namego(<waypoint name>)
        - speak/praat/(sentence), save_sentence(sentence) saves the .wav file to /tmp/<sentence_you_typed>.wav
        - head_down, head_reset, left/right_close/open, look_at_point(x,y,z), 
        - get_pose_2d()
        - test_audio()
    Finally, methods can be called without parentheses, like 'speak "Look ma, no parentheses!"'\033[1;m"""
