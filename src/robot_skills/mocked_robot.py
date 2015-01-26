#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_skills')
import rospy

import geometry_msgs
import std_msgs.msg
import tf

import mock

class Arm(object):
    def __init__(self, robot_name, side, tf_listener):
        self.side = side

        self.default_configurations = mock.MagicMock()
        self.default_trajectories = mock.MagicMock()
        self.load_param = mock.MagicMock()
        self.close = mock.MagicMock()
        self.send_goal = mock.MagicMock()
        self.to = mock.MagicMock()
        self.send_joint_goal = mock.MagicMock()
        self.configuration = mock.MagicMock()
        self.default_configurations = mock.MagicMock()
        self.send_joint_trajectory = mock.MagicMock()
        self.configuration = mock.MagicMock()
        self.self = mock.MagicMock()
        self.reset = mock.MagicMock()
        self.send_gripper_goal = mock.MagicMock()
        self._send_joint_trajectory = mock.MagicMock()
        self._publish_marker = mock.MagicMock()
        self._arm_measurement_callback = mock.MagicMock()

class Base(object):
    def __init__(self, *args, **kwargs):
        self.move = mock.MagicMock
        self.force_drive = mock.MagicMock
        self.get_location = mock.MagicMock
        self.set_initial_pose = mock.MagicMock
        self.go = mock.MagicMock
        self.reset_costmap = mock.MagicMock
        self.cancel_goal = mock.MagicMock

class Ears(object):
    def __init__(self, *args, **kwargs):
        self.recognize = mock.MagicMock()

class EButton(object):
    def __init__(self, *args, **kwargs):
        self.close = mock.MagicMock()
        self._listen = mock.MagicMock()
        self.read_ebutton = mock.MagicMock()

class Head(object):
    def __init__(self, *args, **kwargs):
        self.close = mock.MagicMock()
        self.set_pan_tilt = mock.MagicMock()
        self.send_goal = mock.MagicMock()
        self.ults = mock.MagicMock()
        self.ult = mock.MagicMock()
        self.cancel_goal = mock.MagicMock()
        self.reset = mock.MagicMock()
        self.look_at_hand = mock.MagicMock()
        self.wait = mock.MagicMock()
        self.getGoal = mock.MagicMock()
        self.atGoal = mock.MagicMock()
        self.lookAtStandingPerson = mock.MagicMock()
        self.setPanTiltGoal = mock.MagicMock()
        self.setLookAtGoal = mock.MagicMock()
        self.cancelGoal = mock.MagicMock()
        self._setHeadReferenceGoal = mock.MagicMock()
        self.__feedbackCallback = mock.MagicMock()
        self.__doneCallback = mock.MagicMock()

class Lights(object):
    def __init__(self, *args, **kwargs):
        self.close = mock.MagicMock()
        self.set_color = mock.MagicMock()
        self.on = mock.MagicMock()
        self.off = mock.MagicMock()
        self.start_sinus = mock.MagicMock()

class Perception(object):
    def __init__(self, *args, **kwargs):
        self.close = mock.MagicMock()
        self.toggle = mock.MagicMock()
        self.toggle_always_on = mock.MagicMock()
        self.toggle_always_off = mock.MagicMock()
        self.toggle_everything_off = mock.MagicMock()
        self.toggle_recognition = mock.MagicMock()
        self.toggle_perception_2d = mock.MagicMock()
        self.set_perception_roi = mock.MagicMock()
        self.set_table_roi = mock.MagicMock()
        self.toggle_bin_detection = mock.MagicMock()
        self.load_template_matching_config = mock.MagicMock()
        self.learn_person = mock.MagicMock()
        self.cancel_learn_persons = mock.MagicMock()
        self.__cb_learn_face = mock.MagicMock()
        self.get_learn_face_counter = mock.MagicMock()
        self.rec_start = mock.MagicMock()
        self.people_detection_torso_laser = mock.MagicMock()

class PerceptionED(object):
    def __init__(self, *args, **kwargs):
        self.close = mock.MagicMock()
        self.toggle = mock.MagicMock()
        self.toggle_always_on = mock.MagicMock()
        self.toggle_always_off = mock.MagicMock()
        self.toggle_everything_off = mock.MagicMock()
        self.toggle_recognition = mock.MagicMock()
        self.toggle_perception_2d = mock.MagicMock()
        self.set_perception_roi = mock.MagicMock()
        self.set_table_roi = mock.MagicMock()
        self.toggle_bin_detection = mock.MagicMock()
        self.load_template_matching_config = mock.MagicMock()
        self.learn_person = mock.MagicMock()
        self.cancel_learn_persons = mock.MagicMock()
        self.get_learn_face_counter = mock.MagicMock()
        self.rec_start = mock.MagicMock()
        self.people_detection_torso_laser = mock.MagicMock()

class Reasoner(object):
    def __init__(self, *args, **kwargs):
        self.close = mock.MagicMock()
        self.query = mock.MagicMock()
        self.assertz = mock.MagicMock()
        self.exists_predicate = mock.MagicMock()
        self.__getattr__ = mock.MagicMock()
        self.attach_object_to_gripper = mock.MagicMock()
        self.detach_all_from_gripper = mock.MagicMock()
        self.set_time_marker = mock.MagicMock()
        self.get_time_since = mock.MagicMock()
        self.reset = mock.MagicMock()

class Speech(object):
    def __init__(self, *args, **kwargs):
        self.close = mock.MagicMock()
        self.speak = mock.MagicMock()
        self.__speak = mock.MagicMock()
        self.speak_info = mock.MagicMock()
        self.get_info = mock.MagicMock()
        self.get_action = mock.MagicMock()
        self.buildList = mock.MagicMock()

class Torso(object):
    def __init__(self, *args, **kwargs):
        self.close = mock.MagicMock()
        self.send_goal = mock.MagicMock()
        self._send_goal = mock.MagicMock()
        self.high = mock.MagicMock()
        self.medium = mock.MagicMock()
        self.low = mock.MagicMock()
        self.reset = mock.MagicMock()
        self.wait = mock.MagicMock()
        self.cancel_goal = mock.MagicMock()
        self._receive_torso_measurement = mock.MagicMock()
        self.get_position    = mock.MagicMock()

class ED(object):
    def __init__(self, *args, **kwargs):
        self.get_entities = mock.MagicMock()
        self.get_closest_entity = mock.MagicMock()
        self.get_entity = mock.MagicMock()
        self.reset = mock.MagicMock()


# class MockbotArms(arms.Arms):
#     def __init__(self, tf_listener):
#         super(MockbotArms, self).__init__(tf_listener)

class Mockbot(object):
    """
    Interface to all parts of Mockbot. When initializing Mockbot, you can choose a list of components
    which wont be needed
    
    # I want a blind and mute Mockbot!
    >>> Mockbot(['perception', 'speech'])
    
    # I want a full fledged, awesome Mockbot
    >>> Mockbot()
    """
    def __init__(self, *args, **kwargs):
        self.robot_name = "mockbot"
        self.tf_listener = mock.MagicMock()

        # Body parts
        self.base = Base()
        self.torso = Torso()
        self.leftArm = Arm(self.robot_name, self.tf_listener, "left")
        self.rightArm = Arm(self.robot_name, self.tf_listener, "right")
        self.head = Head()

        # Human Robot Interaction
        self.speech = Speech()
        self.ears = Ears()
        self.ebutton = EButton()
        self.lights = Lights()

        # Perception: can we get rid of this???
        self.perception = Perception()

        # Reasoning/world modeling
        self.ed = ED()
        self.reasoner = Reasoner()

        # Miscellaneous
        self.pub_target = rospy.Publisher("/target_location", geometry_msgs.msg.Pose2D)
        self.base_link_frame = "/"+self.robot_name+"base_link"

        #Grasp offsets
        #TODO: Don't hardcode, load from parameter server to make robot independent.
        self.grasp_offset = geometry_msgs.msg.Point(0.5, 0.2, 0.0)

        self.publish_target = mock.MagicMock()
        self.tf_transform_pose = mock.MagicMock()
        self.close = mock.MagicMock()
        self.get_base_goal_poses = mock.MagicMock()

    def __enter__(self):
        pass

    def __exit__(self, exception_type, exception_val, trace):
        if any((exception_type, exception_val, trace)):
            rospy.logerr("Robot exited with {0},{1},{2}".format(exception_type, exception_val, trace))
        self.close()

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

    rospy.init_node("mockbot_executioner", anonymous=True)
    mockbot = Mockbot(wait_services=False)
    robot = mockbot #All state machines use robot. ..., this makes copy/pasting easier.

    atexit.register(mockbot.close) #When exiting the interpreter, call mockbot.close(), which cancels all action goals etc.

    head_reset = lambda: mockbot.head.reset_position()
    head_down  = lambda: mockbot.head.look_down()
    right_close = lambda: mockbot.rightArm.send_gripper_goal_close()
    left_close = lambda: mockbot.leftArm.send_gripper_goal_close()
    right_open = lambda: mockbot.rightArm.send_gripper_goal_open()
    left_open = lambda: mockbot.leftArm.send_gripper_goal_open()
    speak = lambda sentence: mockbot.speech.speak(sentence, block=False)
    praat = lambda sentence: mockbot.speech.speak(sentence, language='nl', block=False)
    look_at_point = lambda x, y, z: mockbot.head.send_goal(msgs.PointStamped(x, y, z, frame_id="/mockbot/base_link"))
        
    r = mockbot.reasoner
    q = mockbot.reasoner.query

    mapgo = mockbot.base.go
    
    def basego(x,y,phi):
        return mockbot.base.go(x,y,phi,frame="/mockbot/base_link")

    open_door   = lambda: r.assertz(r.state("door1", "open"))
    close_door  = lambda: r.assertz(r.state("door1", "close"))
    def insert_object(x,y,z):
        from test_tools.WorldFaker import WorldFaker
        wf = WorldFaker()
        wf.insert({"position":(x,y,z)})

    #Useful for making location-files
    def get_pose_2d():
       posestamped = mockbot.base.location
       loc,rot = posestamped.pose.point, posestamped.pose.orientation
       rot_array = [rot.w, rot.x, rot.y, rot.z]
       rot3 = tf.transformations.euler_from_quaternion(rot_array)
       print 'x={0}, y={1}, phi={2}'.format(loc.x, loc.y, rot3[0])
       return (loc.x, loc.y, rot3[0]) 

    def hear(text):
        pub = rospy.Publisher('/pocketsphinx/output', std_msgs.msg.String)
        rospy.logdebug("Telling Mockbot '{0}'".format(text))
        pub.publish(std_msgs.msg.String(text))

    def save_sentence(sentence):
        """Let mockbot say a sentence and them move the generated speech file to a separate file that will not be overwritten"""
        speak(sentence)
        import os
        path = sentence.replace(" ", "_")
        path += ".wav"
        os.system("mv /tmp/speech.wav /tmp/{0}".format(path))

    def test_audio():
        OKGREEN = '\033[92m'
        ENDC = '\033[0m'

        mockbot.speech.speak("Please say: continue when I turn green", block=True) #definitely block here, otherwise we hear ourselves
        
        result = mockbot.ears.ask_user("continue")
        rospy.loginfo("test_audio result: {0}".format(result))
        if result and result != "false":
            rospy.loginfo(OKGREEN+"Speach pipeline working"+ENDC)
            mockbot.speech.speak("Yep, my ears are fine", block=False)
        else:
            rospy.logerr("Speech pipeline not working")
            mockbot.speech.speak("Nope, my ears are clogged", block=False)


    print """\033[1;33m You can now command mockbot from the python REPL. 
    Type e.g. help(mockbot) for help on objects and functions, 
    or type 'mockbot.' <TAB> to see what methods are available. 
    Also, try 'dir()'
    You can use both robot.foo or mockbot.foo for easy copypasting.
    Press ctrl-D or type 'exit()' to exit the awesome mockbot-console.
    WARNING: When the console exits, it cancels ALL arm and base goals.
    There are handy shortcuts, such as: 
        - mapgo/basego(x,y,phi), 
        - speak/praat/(sentence), save_sentence(sentence) saves the .wav file to /tmp/<sentence_you_typed>.wav
        - head_down, head_reset, left/right_close/open, look_at_point(x,y,z), 
        - get_pose_2d()
        - test_audio()
    Finally, methods can be called without parentheses, like 'speak "Look ma, no parentheses!"'\033[1;m"""
