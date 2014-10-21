import atexit
import util.msg_constructors as msgs
from reasoner import Compound, Conjunction, Sequence, Variable

rospy.init_node("robot_executioner", anonymous=True)
robot = Robot(wait_services=False)

atexit.register(robot.close) #When exiting the interpreter, call robot.close(), which cancels all action goals etc.

head_reset = lambda: robot.head.reset_position()
head_down  = lambda: robot.head.look_down()
right_close = lambda: robot.rightArm.send_gripper_goal_close()
left_close = lambda: robot.leftArm.send_gripper_goal_close()
right_open = lambda: robot.rightArm.send_gripper_goal_open()
left_open = lambda: robot.leftArm.send_gripper_goal_open()
speak = lambda sentence: robot.speech.speak(sentence, block=False)
praat = lambda sentence: robot.speech.speak(sentence, language='nl', block=False)
look_at_point = lambda x, y, z: robot.head.send_goal(msgs.PointStamped(x, y, z, frame_id="base_link")) # ToDo: correct frame id
    
r = robot.reasoner
q = robot.reasoner.query

mapgo = robot.base.go

def airgo(x,y,z, xoffset=0.5, yoffset=0.1):
    target = robot.base.point(x,y,z, stamped=True)
    ik_poses = robot.base.get_base_goal_poses(target, xoffset, yoffset)
    robot.base.send_goal(ik_poses[0])
    robot.head.send_goal(msgs.PointStamped(x,y,z))

def namego(name):
    """Go to the waypoint with the given name. 
    When there re multiple locations with the samen name, shame on you for having messed up locations, and the robot will go to the first query result"""
    query = Compound("waypoint", name, Compound("pose_2d", "X", "Y", "Phi"))
    answers = robot.reasoner.query(query)
    try:
        selected = answers[0]
    except IndexError:
        rospy.logerr("No named location {0} (name = {1})".format(query, name))
        return None
    x,y,phi = float(selected["X"]), float(selected["Y"]), float(selected["Phi"])
    return robot.base.go(x,y,phi)

def basego(x,y,phi):
    return robot.base.go(x,y,phi,frame="base_link") # ToDo: correct frame id

open_door   = lambda: r.assertz(r.state("door1", "open"))
close_door  = lambda: r.assertz(r.state("door1", "close"))
def insert_object(x,y,z):
    from test_tools.WorldFaker import WorldFaker
    wf = WorldFaker()
    wf.insert({"position":(x,y,z)})

#Useful for making location-files
def get_pose_2d():
   posestamped = robot.base.location
   loc,rot = posestamped.pose.point, posestamped.pose.orientation
   rot_array = [rot.w, rot.x, rot.y, rot.z]
   rot3 = tf.transformations.euler_from_quaternion(rot_array)
   print 'x={0}, y={1}, phi={2}'.format(loc.x, loc.y, rot3[0])
   return (loc.x, loc.y, rot3[0]) 

def hear(text):
    pub = rospy.Publisher('/pocketsphinx/output', std_msgs.msg.String)
    rospy.logdebug("Telling robot '{0}'".format(text))
    pub.publish(std_msgs.msg.String(text))

def save_sentence(sentence):
    """Let robot say a sentence and them move the generated speech file to a separate file that will not be overwritten"""
    speak(sentence)
    import os
    path = sentence.replace(" ", "_")
    path += ".wav"
    os.system("mv /tmp/speech.wav /tmp/{0}".format(path))

def test_audio():
    OKGREEN = '\033[92m'
    ENDC = '\033[0m'

    robot.speech.speak("Please say: continue when I turn green", block=True) #definitely block here, otherwise we hear ourselves
    
    result = robot.ears.ask_user("continue")
    rospy.loginfo("test_audio result: {0}".format(result))
    if result and result != "false":
        rospy.loginfo(OKGREEN+"Speach pipeline working"+ENDC)
        robot.speech.speak("Yep, my ears are fine", block=False)
    else:
        rospy.logerr("Speech pipeline not working")
        robot.speech.speak("Nope, my ears are clogged", block=False)


print """\033[1;33m You can now command the robot from the python REPL. 
Type e.g. help (robot) for help on objects and functions, 
or type 'robot.' <TAB> to see what methods are available. 
Also, try 'dir()'
You can use both robot.foo or the robot.foo for easy copypasting.
Press ctrl-D or type 'exit()' to exit the awesome robot-console.
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