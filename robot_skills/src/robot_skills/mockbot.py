#! /usr/bin/env python

# System
from collections import defaultdict
import mock
import random
import os

# ROS
import geometry_msgs
import PyKDL as kdl
import rospy
import std_msgs.msg
import tf

# TU/e Robotics
import arms
from ed_msgs.msg import EntityInfo
from ed_sensor_integration_msgs.srv import UpdateResponse
from robot_skills import robot
from robot_skills.util.kdl_conversions import VectorStamped, FrameStamped
from robot_skills.classification_result import ClassificationResult
from robot_skills.util.entity import from_entity_info
from hmi import HMIResult
from hmi.common import random_sentence, parse_sentence


def random_kdl_vector():
    return kdl.Vector(random.random(), random.random(), random.random())


def random_kdl_frame():
    return kdl.Frame(kdl.Rotation.RPY(random.random(), random.random(), random.random()),
                     random_kdl_vector())


def mock_query(description, grammar, target, timeout):
    sentence = random_sentence(grammar, target)
    semantics = parse_sentence(sentence, grammar, target)
    return HMIResult(sentence=sentence, semantics=semantics)


def old_query(spec, choices, timeout=10):
    raise Exception('robot.ears.recognize IS REMOVED. Use `robot.hmi.query`')


class MockedRobotPart(object):
    def __init__(self, robot_name, tf_listener, *args, **kwargs):
        self.robot_name = robot_name
        self.tf_listener = tf_listener

        self.load_param = mock.MagicMock()
        self.wait_for_connections = mock.MagicMock()
        self.create_simple_action_client = mock.MagicMock()
        self.create_service_client = mock.MagicMock()
        self.create_subscriber = mock.MagicMock()
        self._add_connection = mock.MagicMock()
        self.operational = mock.MagicMock()
        self.subscribe_hardware_status = mock.MagicMock()
        self.unsubscribe_hardware_status = mock.MagicMock()
        self.process_hardware_status = mock.MagicMock()
        self.reset = mock.MagicMock()


class Arm(MockedRobotPart):
    def __init__(self, robot_name, tf_listener, get_joint_states, side):
        super(Arm, self).__init__(robot_name, tf_listener)

        self.side = side
        self.get_joint_states = get_joint_states

        self.occupied_by = None
        self._operational = True

        self._base_offset = random_kdl_vector()

        self.default_configurations = mock.MagicMock()
        self.default_trajectories = mock.MagicMock()
        self.has_joint_goal = mock.MagicMock()
        self.has_joint_trajectory = mock.MagicMock()
        self.cancel_goals = mock.MagicMock()
        self.close = mock.MagicMock()
        self.send_goal = mock.MagicMock()
        self.send_joint_goal = mock.MagicMock()
        self.send_joint_trajectory = mock.MagicMock()
        self.reset = mock.MagicMock()
        self.send_gripper_goal = mock.MagicMock()
        self.handover_to_human = mock.MagicMock()
        self.handover_to_robot = mock.MagicMock()
        self._send_joint_trajectory = mock.MagicMock()
        self._publish_marker = mock.MagicMock()
        self.wait_for_motion_done = mock.MagicMock()

    def collect_gripper_types(self, gripper_type):
        return gripper_type


class Base(MockedRobotPart):
    def __init__(self, robot_name, tf_listener, *args, **kwargs):
        super(Base, self).__init__(robot_name, tf_listener)
        self.move = mock.MagicMock()
        self.force_drive = mock.MagicMock()
        self.get_location = lambda: FrameStamped(random_kdl_frame(), "/map")
        self.set_initial_pose = mock.MagicMock()
        self.go = mock.MagicMock()
        self.reset_costmap = mock.MagicMock()
        self.cancel_goal = mock.MagicMock()
        self.analyzer = mock.MagicMock()
        self.global_planner = mock.MagicMock()
        self.local_planner = mock.MagicMock()
        self.local_planner.getStatus = mock.MagicMock(return_value="arrived")  # always arrive for now
        self.global_planner.getPlan = mock.MagicMock(return_value=["dummy_plan"])  # always arrive for now


class Hmi(MockedRobotPart):
    def __init__(self, robot_name, tf_listener, *args, **kwargs):
        super(Hmi, self).__init__(robot_name, tf_listener)

        self.query = mock_query
        self.show_image = mock.MagicMock()
        self.show_image_from_msg = mock.MagicMock()
        self.old_query = old_query
        self.reset = mock.MagicMock()
        self.restart_dragonfly = mock.MagicMock()


class EButton(MockedRobotPart):
    def __init__(self, robot_name, tf_listener, *args, **kwargs):
        super(EButton, self).__init__(robot_name, tf_listener)
        self.close = mock.MagicMock()
        self._listen = mock.MagicMock()
        self.read_ebutton = mock.MagicMock()


class Head(MockedRobotPart):
    def __init__(self, robot_name, tf_listener, *args, **kwargs):
        super(Head, self).__init__(robot_name, tf_listener)
        self.reset = mock.MagicMock()
        self.close = mock.MagicMock()
        self.set_pan_tilt = mock.MagicMock()
        self.send_goal = mock.MagicMock()
        self.cancel_goal = mock.MagicMock()
        self.reset = mock.MagicMock()
        self.look_at_hand = mock.MagicMock()
        self.wait = mock.MagicMock()
        self.getGoal = mock.MagicMock()
        self.atGoal = mock.MagicMock()
        self.look_at_standing_person = mock.MagicMock()
        self.look_at_point = mock.MagicMock()
        self.look_at_ground_in_front_of_robot = mock.MagicMock()  # TODO: Must return a EntityInfo
        self.setPanTiltGoal = mock.MagicMock()
        self.setLookAtGoal = mock.MagicMock()
        self.cancelGoal = mock.MagicMock()
        self.wait_for_motion_done = mock.MagicMock()
        self._setHeadReferenceGoal = mock.MagicMock()
        self.__feedbackCallback = mock.MagicMock()
        self.__doneCallback = mock.MagicMock()


class Perception(MockedRobotPart):
    def __init__(self, robot_name, tf_listener, *args, **kwargs):
        super(Perception, self).__init__(robot_name, tf_listener)
        self.reset = mock.MagicMock()
        self.close = mock.MagicMock()
        self.learn_person = mock.MagicMock()
        self.detect_faces = mock.MagicMock()
        self.get_best_face_recognition = mock.MagicMock()
        self.get_rgb_depth_caminfo = mock.MagicMock()
        self.project_roi = lambda *args, **kwargs: VectorStamped(random.random(), random.random(), random.random(), "/map")


class Lights(MockedRobotPart):
    def __init__(self, robot_name, tf_listener, *args, **kwargs):
        super(Lights, self).__init__(robot_name, tf_listener)
        self.close = mock.MagicMock()
        self.set_color = mock.MagicMock()
        self.set_color_colorRGBA = mock.MagicMock()
        self.on = mock.MagicMock()
        self.off = mock.MagicMock()


class Speech(MockedRobotPart):
    def __init__(self, robot_name, tf_listener, *args, **kwargs):
        super(Speech, self).__init__(robot_name, tf_listener)
        self.close = mock.MagicMock()
        self.speak = mock.MagicMock()


class Torso(MockedRobotPart):
    def __init__(self, robot_name, tf_listener, get_joint_states, *args, **kwargs):
        super(Torso, self).__init__(robot_name, tf_listener)

        self.get_joint_states = get_joint_states

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
        self.get_position = mock.MagicMock()
        self.wait_for_motion_done = mock.MagicMock()


class ED(MockedRobotPart):
    @staticmethod
    def generate_random_entity(id=None, type=None):
            entity_info = EntityInfo()

            if not id:
                entity_info.id = str(hash(entity_info))
            else:
                entity_info.id = id
            entity_info.type = random.choice(["random_from_magicmock", "human", "coke", "fanta"])
            # entity.data = mock.MagicMock()
            entity_info.data = ""

            entity = from_entity_info(entity_info)
            entity._pose = random_kdl_frame()
            if type:
                entity.type = type
            return entity

    def __init__(self, robot_name, tf_listener, *args, **kwargs):
        super(ED, self).__init__(robot_name, tf_listener)
        self._dynamic_entities = defaultdict(self.generate_random_entity,
                                     {e.id: e for e in [self.generate_random_entity() for _ in range(5)]})

        self._dynamic_entities['john'] = self.generate_random_entity(id='john', type='person')
        self._static_entities = defaultdict(self.generate_random_entity,
                                     {e.id: e for e in [self.generate_random_entity() for _ in range(5)]})
        self._static_entities['test_waypoint_1'] = self.generate_random_entity(id='test_waypoint_1', type='waypoint')
        self._static_entities['cabinet'] = self.generate_random_entity(id='cabinet')

        self.get_closest_entity = lambda *args, **kwargs: random.choice(self._entities.values())
        self.get_entity = lambda id=None, parse=True: self._entities[id]
        self.reset = lambda *args, **kwargs: self._dynamic_entities.clear()
        self.navigation = mock.MagicMock()
        self.navigation.get_position_constraint = mock.MagicMock()
        self.update_entity = mock.MagicMock()
        self.get_closest_possible_person_entity = lambda *args, **kwargs: self.generate_random_entity()
        self.get_closest_laser_entity = lambda *args, **kwargs: self.generate_random_entity()
        self.get_entity_info = mock.MagicMock()
        self.wait_for_connections = mock.MagicMock()

        self._person_names = []

    def get_entities(self, type="", center_point=VectorStamped(), radius=0, id="", parse=True):

        center_point_in_map = center_point.projectToFrame("/map", self.tf_listener)

        entities = self._entities.values()
        if type:
            entities = [e for e in entities if e.is_a(type)]
        if radius:
            entities = [e for e in entities if e.distance_to_2d(center_point_in_map.vector) <= radius]
        if id:
            entities = [e for e in entities if e.id == id]

        return entities

    @property
    def _entities(self):
        return defaultdict(ED.generate_random_entity, self._dynamic_entities.items() + self._static_entities.items())

    def segment_kinect(self, *args, **kwargs):
        self._dynamic_entities = {e.id: e for e in [ED.generate_random_entity() for _ in range(5)]}
        return self._entities

    def update_kinect(self, *args, **kwargs):
        new_entities = {e.id:e for e in [ED.generate_random_entity() for _ in range(2)]}
        self._dynamic_entities.update(new_entities)

        res = UpdateResponse()
        res.new_ids = [e.id for e in new_entities.values()]
        res.updated_ids = [random.choice(self._dynamic_entities).id for _ in range(2)]
        res.deleted_ids = []
        return res

    def classify(self, ids, types=None, unknown_threshold=0.0):
        entities = [self._entities[_id] for _id in ids if _id in self._entities]
        return [ClassificationResult(e.id, e.type, random.uniform(0, 1), None) for e in entities]

    def detect_people(self, rgb, depth, cam_info):
        return True, [self._dynamic_entities['operator'].id]


class MockedTfListener(mock.MagicMock):
    def __init__(self):
        super(MockedTfListener, self).__init__()

    @staticmethod
    def waitForTransform(*args, **kwargs):
        return True

    @staticmethod
    def transformPoint(frame_id, point_stamped):
        point_stamped.header.frame_id = frame_id
        return point_stamped


class Mockbot(robot.Robot):
    """
    Interface to all parts of Mockbot. When initializing Mockbot, you can choose a list of components
    which wont be needed

    # I want a blind and mute Mockbot!
    >>> Mockbot(['perception', 'speech'])  # doctest: +SKIP

    # I want a full fledged, awesome Mockbot
    >>> Mockbot()  # doctest: +SKIP
    """
    def __init__(self, *args, **kwargs):
        robot_name = "mockbot"

        super(Mockbot, self).__init__(robot_name=robot_name, wait_services=False, tf_listener=MockedTfListener)

        self.publish_target = mock.MagicMock()
        self.tf_transform_pose = mock.MagicMock()
        self.close = mock.MagicMock()
        self.get_joint_states = mock.MagicMock()

        self.add_body_part('hmi', Hmi(self.robot_name, self.tf_listener))

        # Body parts
        self.add_body_part('base', Base(self.robot_name, self.tf_listener))
        self.add_body_part('torso', Torso(self.robot_name, self.tf_listener, self.get_joint_states))
        self.add_arm_part(
            'leftArm',
            Arm(self.robot_name, self.tf_listener, self.get_joint_states, "left")
        )
        self.add_arm_part(
            'rightArm',
            Arm(self.robot_name, self.tf_listener, self.get_joint_states, "right")
        )
        self.add_body_part('head', Head(self.robot_name, self.tf_listener))

        # Human Robot Interaction
        self.add_body_part('speech', Speech(self.robot_name, self.tf_listener))
        self.add_body_part('ebutton', EButton(self.robot_name, self.tf_listener))
        self.add_body_part('lights', Lights(self.robot_name, self.tf_listener))

        # Reasoning/world modeling
        self.add_body_part('ed', ED(self.robot_name, self.tf_listener))
        self.add_body_part('perception', Perception(self.robot_name, self.tf_listener))

        # Miscellaneous
        self.pub_target = mock.MagicMock()

        # Grasp offsets
        # TODO: Don't hardcode, load from parameter server to make robot independent.
        self.grasp_offset = geometry_msgs.msg.Point(0.5, 0.2, 0.0)

    def __enter__(self):
        pass

    def __exit__(self, exception_type, exception_val, trace):
        if any((exception_type, exception_val, trace)):
            rospy.logerr("Robot exited with {0},{1},{2}".format(exception_type, exception_val, trace))
        self.close()


if __name__ == "__main__":
    print("     _              __")
    print("    / `\\  (~._    ./  )")
    print("    \\__/ __`-_\\__/ ./")
    print("   _ \\ \\/  \\  \\ |_   __")
    print(" (   )  \\__/ -^    \\ /  \\")
    print("  \\_/ \"  \\  | o  o  |.. /  __")
    print("       \\\\. --' ====  /  || /  \\")
    print("         \\   .  .  |---__.\\__/")
    print("         /  :     /   |   |")
    print("         /   :   /     \\_/")
    print("      --/ ::    (")
    print("     (  |     (  (____")
    print("   .--  .. ----**.____)")
    print("   \\___/          ")
    import atexit

    rospy.init_node("mockbot_executioner", anonymous=True)
    mockbot = Mockbot(wait_services=False)
    robot = mockbot  # All state machines use robot. ..., this makes copy/pasting easier.

    atexit.register(mockbot.close)  # When exiting the interpreter, call mockbot.close(), which cancels all action goals etc.

    head_reset = lambda: mockbot.head.reset()
    head_down  = lambda: mockbot.head.reset()
    right_close = lambda: mockbot._arms['rightArm'].send_gripper_goal_close()
    left_close = lambda: mockbot._arms['leftArm'].send_gripper_goal_close()
    right_open = lambda: mockbot._arms['rightArm'].send_gripper_goal_open()
    left_open = lambda: mockbot._arms['leftArm'].send_gripper_goal_open()
    speak = lambda sentence: mockbot.speech.speak(sentence, block=False)
    praat = lambda sentence: mockbot.speech.speak(sentence, language='nl', block=False)
    look_at_point = lambda x, y, z: mockbot.head.look_at_point(VectorStamped(x, y, z, frame_id="/mockbot/base_link"))

    mapgo = mockbot.base.go

    def basego(x,y,phi):
        return mockbot.base.go(x,y,phi,frame="/mockbot/base_link")

    def insert_object(x,y,z):
        from test_tools.WorldFaker import WorldFaker
        wf = WorldFaker()
        wf.insert({"position":(x,y,z)})

    # Useful for making location-files
    def get_pose_2d():
       posestamped = mockbot.base.location
       loc,rot = posestamped.pose.point, posestamped.pose.orientation
       rot_array = [rot.w, rot.x, rot.y, rot.z]
       rot3 = tf.transformations.euler_from_quaternion(rot_array)
       print('x={0}, y={1}, phi={2}'.format(loc.x, loc.y, rot3[0]))
       return loc.x, loc.y, rot3[0]

    def hear(text):
        pub = rospy.Publisher('/pocketsphinx/output', std_msgs.msg.String, queue_size=10)
        rospy.logdebug("Telling Mockbot '{0}'".format(text))
        pub.publish(std_msgs.msg.String(text))

    def save_sentence(sentence):
        """
        Let mockbot say a sentence and them move the generated speech file to a separate file that will not be overwritten
        """
        speak(sentence)
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


    print("""\033[1;33m You can now command mockbot from the python REPL.
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
    Finally, methods can be called without parentheses, like 'speak "Look ma, no parentheses!"'\033[1;m""")
