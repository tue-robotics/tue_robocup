#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import robot

import tf
import tf_server

from util import transformations
import geometry_msgs
import std_msgs.msg

from math import degrees, radians
from psi import Compound, Sequence, Conjunction

# class AmigoArms(arms.Arms):
#     def __init__(self, tf_listener):
#         super(AmigoArms, self).__init__(tf_listener)

class Amigo(robot.Robot):
    """
    Interface to all parts of Amigo. When initializing Amigo, you can choose a list of components
    which wont be needed
    
    # I want a blind and mute Amigo!
    >>> Amigo(['perception', 'speech'])
    
    # I want a full fledged, awesome Amigo
    >>> Amigo()
    """
    def __init__(self, dontInclude = [], wait_services=False):
        super(Amigo, self).__init__(robot_name="amigo", wait_services=wait_services)
    

if __name__ == "__main__":
    print "     _              __"
    print "    / `\\  (~._    ./  )"
    print "    \\__/ __`-_\\__/ ./"
    print "   _ \\ \\/  \\  \\ |_   __"
    print " (   )  \\__/ -^    \\ /  \\"
    print "  \\_/ \"  \\  | o  o  |.. /  __"
    print "       \\\\. --' ====  /  || /  \\"
    print "         \\   .  .  |---__.\\__/"
    print "         /  :     /   |   |"
    print "         /   :   /     \\_/"
    print "      --/ ::    ("
    print "     (  |     (  (____"
    print "   .--  .. ----**.____)"
    print "   \\___/          "
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
