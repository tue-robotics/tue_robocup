#! /usr/bin/env python

from __future__ import absolute_import, print_function

import os
import random
# System
from collections import defaultdict

import PyKDL as kdl
# ROS
import geometry_msgs
import mock
import rospy
import six
import std_msgs.msg
import tf_conversions
# TU/e Robotics
from ed_msgs.msg import EntityInfo
from ed_sensor_integration_msgs.srv import UpdateResponse

from hmi import HMIResult
from hmi.common import parse_sentence, random_sentence
from robot_skills import robot
from robot_skills.classification_result import ClassificationResult
from robot_skills.util.entity import from_entity_info
from robot_skills.util.kdl_conversions import FrameStamped, VectorStamped


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


class AlteredMagicMock(mock.MagicMock):
    def assert_called_with(self, *args, **kwargs):
        """
        Assert that the mock was called with the specified arguments.

        Raises an AssertionError if the args and keyword args passed in are
        different to the last call to the mock.
        """
        if self.call_args is None:
            expected = self._format_mock_call_signature(args, kwargs)
            raise AssertionError('Expected call: %s\nNot called' % (expected,))

        def _error_message(cause):
            msg = self._format_mock_failure_message(args, kwargs)
            if six.PY2 and cause is not None:
                # Tack on some diagnostics for Python without __cause__
                msg = '%s\n%s' % (msg, str(cause))
            return msg

        expected = self._call_matcher((args, kwargs))
        actual = self._call_matcher(self.call_args)
        if not self.equal_or_more_arguments(expected, actual):
            cause = expected if isinstance(expected, Exception) else None
            six.raise_from(AssertionError(_error_message(cause)), cause)

    def assert_any_call(self, *args, **kwargs):
        """
        Assert the mock has been called with the specified arguments.

        The assert passes if the mock has *ever* been called, unlike
        `assert_called_with` and `assert_called_once_with` that only pass if
        the call is the most recent one.
        """
        expected = self._call_matcher((args, kwargs))
        actual = [self._call_matcher(c) for c in self.call_args_list]
        if not any([self.equal_or_more_arguments(expected, c) for c in actual]):
            cause = expected if isinstance(expected, Exception) else None
            expected_string = self._format_mock_call_signature(args, kwargs)
            six.raise_from(AssertionError("'%s' call not found" % expected_string), cause)

    @staticmethod
    def equal_or_more_arguments(expected, actual):
        """
        Check whether the actual call contains at least the arguments and keyword arguments of the expected call.
        But still return true if the actual call contains more (keyword) arguments than expected
        """
        # check args
        for i in range(len(expected[0])):
            if i >= len(actual[0]):
                return False
            if expected[0][i] != actual[0][i]:
                return False
        # check kwargs
        for k in expected[1]:
            if k not in actual[1]:
                return False
            if expected[1][k] != actual[1][k]:
                return False
        return True


class MockedRobotPart(object):
    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        self.robot_name = robot_name
        self.tf_buffer = tf_buffer

        self.load_param = AlteredMagicMock()
        self.wait_for_connections = AlteredMagicMock()
        self.create_simple_action_client = AlteredMagicMock()
        self.create_service_client = AlteredMagicMock()
        self.create_subscriber = AlteredMagicMock()
        self._add_connection = AlteredMagicMock()
        self.operational = AlteredMagicMock()
        self.subscribe_hardware_status = AlteredMagicMock()
        self.unsubscribe_hardware_status = AlteredMagicMock()
        self.process_hardware_status = AlteredMagicMock()
        self.reset = AlteredMagicMock()


class Arm(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, get_joint_states, name):
        super(Arm, self).__init__(robot_name, tf_buffer)

        self.arm_name = name
        self.get_joint_states = get_joint_states

        self._operational = True

        self._base_offset = random_kdl_vector()

        self.default_configurations = AlteredMagicMock()
        self.default_trajectories = AlteredMagicMock()
        self.has_joint_goal = AlteredMagicMock()
        self.has_joint_trajectory = AlteredMagicMock()
        self.cancel_goals = AlteredMagicMock()
        self.close = AlteredMagicMock()
        self.send_goal = AlteredMagicMock()
        self.send_joint_goal = AlteredMagicMock()
        self.send_joint_trajectory = AlteredMagicMock()
        self.reset = AlteredMagicMock()
        self._send_joint_trajectory = AlteredMagicMock()
        self._publish_marker = AlteredMagicMock()
        self.wait_for_motion_done = AlteredMagicMock()

        # add parts
        self.gripper = Gripper(robot_name, tf_buffer)
        self.handover_detector = HandoverDetector(robot_name, tf_buffer)

    def collect_gripper_types(self, gripper_type):
        return gripper_type

    @property
    def base_offset(self):
        return self._base_offset


class Gripper(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        super(Gripper, self).__init__(robot_name, tf_buffer)
        self.occupied_by = None
        self.send_goal = AlteredMagicMock()


class HandoverDetector(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        super(HandoverDetector, self).__init__(robot_name, tf_buffer)
        self.handover_to_human = AlteredMagicMock()
        self.handover_to_robot = AlteredMagicMock()


class Base(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        super(Base, self).__init__(robot_name, tf_buffer)
        self.move = AlteredMagicMock()
        self.turn_towards = AlteredMagicMock()
        self.force_drive = AlteredMagicMock()
        self.get_location = lambda: FrameStamped(random_kdl_frame(), "map")
        self.set_initial_pose = AlteredMagicMock()
        self.wait_for_motion_done = AlteredMagicMock()
        self.go = AlteredMagicMock()
        self.reset_costmap = AlteredMagicMock()
        self.cancel_goal = AlteredMagicMock()
        self.analyzer = AlteredMagicMock()
        self.global_planner = AlteredMagicMock()
        self.local_planner = AlteredMagicMock()
        self.local_planner.getStatus = mock.MagicMock(return_value="arrived")  # always arrive for now
        self.global_planner.getPlan = mock.MagicMock(return_value=["dummy_plan"])  # always arrive for now
        self.global_planner.path_length = 0.0


class Hmi(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        super(Hmi, self).__init__(robot_name, tf_buffer)

        self.query = mock_query
        self.show_image = AlteredMagicMock()
        self.show_image_from_msg = AlteredMagicMock()
        self.old_query = old_query
        self.reset = AlteredMagicMock()
        self.restart_dragonfly = AlteredMagicMock()


class EButton(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        super(EButton, self).__init__(robot_name, tf_buffer)
        self.close = AlteredMagicMock()
        self._listen = AlteredMagicMock()
        self.read_ebutton = AlteredMagicMock()


class Head(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        super(Head, self).__init__(robot_name, tf_buffer)
        self.reset = AlteredMagicMock()
        self.close = AlteredMagicMock()
        self.set_pan_tilt = AlteredMagicMock()
        self.send_goal = AlteredMagicMock()
        self.cancel_goal = AlteredMagicMock()
        self.reset = AlteredMagicMock()
        self.wait = AlteredMagicMock()
        self.getGoal = AlteredMagicMock()
        self.atGoal = AlteredMagicMock()
        self.look_at_standing_person = AlteredMagicMock()
        self.look_at_point = AlteredMagicMock()
        self.look_at_ground_in_front_of_robot = AlteredMagicMock()  # TODO: Must return a EntityInfo
        self.setPanTiltGoal = AlteredMagicMock()
        self.setLookAtGoal = AlteredMagicMock()
        self.cancelGoal = AlteredMagicMock()
        self.wait_for_motion_done = AlteredMagicMock()
        self._setHeadReferenceGoal = AlteredMagicMock()
        self.__feedbackCallback = AlteredMagicMock()
        self.__doneCallback = AlteredMagicMock()


class Perception(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        super(Perception, self).__init__(robot_name, tf_buffer)
        self.reset = AlteredMagicMock()
        self.close = AlteredMagicMock()
        self.learn_person = AlteredMagicMock()
        self.detect_faces = AlteredMagicMock()
        self.get_best_face_recognition = AlteredMagicMock()
        self.get_rgb_depth_caminfo = AlteredMagicMock()
        self.project_roi = lambda *args, **kwargs: VectorStamped(random.random(), random.random(), random.random(), "map")
        self.locate_handle_client = AlteredMagicMock()


class Lights(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        super(Lights, self).__init__(robot_name, tf_buffer)
        self.close = AlteredMagicMock()
        self.set_color = AlteredMagicMock()
        self.set_color_rgba_msg = AlteredMagicMock()
        self.on = AlteredMagicMock()
        self.off = AlteredMagicMock()


class Speech(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        super(Speech, self).__init__(robot_name, tf_buffer)
        self.close = AlteredMagicMock()
        self.speak = AlteredMagicMock()


class Torso(MockedRobotPart):
    def __init__(self, robot_name, tf_buffer, get_joint_states, *args, **kwargs):
        super(Torso, self).__init__(robot_name, tf_buffer)

        self.get_joint_states = get_joint_states

        self.close = AlteredMagicMock()
        self.send_goal = AlteredMagicMock()
        self._send_goal = AlteredMagicMock()
        self.high = AlteredMagicMock()
        self.medium = AlteredMagicMock()
        self.low = AlteredMagicMock()
        self.reset = AlteredMagicMock()
        self.wait = AlteredMagicMock()
        self.cancel_goal = AlteredMagicMock()
        self._receive_torso_measurement = AlteredMagicMock()
        self.get_position = AlteredMagicMock()
        self.wait_for_motion_done = AlteredMagicMock()


class ED(MockedRobotPart):
    @staticmethod
    def generate_random_entity(id_=None, type_=None):
            entity_info = EntityInfo()

            if not id_:
                entity_info.id = str(id(entity_info))
            else:
                entity_info.id = id_
            entity_info.type = random.choice(["random_from_magicmock", "human", "coke", "fanta"])
            # entity.data = AlteredMagicMock()
            entity_info.data = ""

            entity = from_entity_info(entity_info)
            entity._pose = random_kdl_frame()
            if type_:
                entity.type = type_
            return entity

    def __init__(self, robot_name, tf_buffer, *args, **kwargs):
        super(ED, self).__init__(robot_name, tf_buffer)
        self._dynamic_entities = defaultdict(self.generate_random_entity,
                                             {e.id: e for e in [self.generate_random_entity() for _ in range(5)]})

        self._dynamic_entities['john'] = self.generate_random_entity(id_='john', type_='person')
        self._static_entities = defaultdict(self.generate_random_entity,
                                     {e.id: e for e in [self.generate_random_entity() for _ in range(5)]})
        self._static_entities['test_waypoint_1'] = self.generate_random_entity(id_='test_waypoint_1', type_='waypoint')
        self._static_entities['cabinet'] = self.generate_random_entity(id_='cabinet')

        self.get_closest_entity = lambda *args, **kwargs: random.choice(self._entities.values())
        self.get_entity = lambda id=None: self._entities[id]
        self.reset = lambda *args, **kwargs: self._dynamic_entities.clear()
        self.navigation = AlteredMagicMock()
        self.navigation.get_position_constraint = AlteredMagicMock()
        self.update_entity = AlteredMagicMock()
        self.get_closest_possible_person_entity = lambda *args, **kwargs: self.generate_random_entity()
        self.get_closest_laser_entity = lambda *args, **kwargs: self.generate_random_entity()
        self.get_entity_info = AlteredMagicMock()
        self.wait_for_connections = AlteredMagicMock()

        self._person_names = []

    def get_entities(self, type="", center_point=VectorStamped(), radius=0, id="", ignore_z=False):

        center_point_in_map = center_point.projectToFrame("map", self.tf_buffer)

        entities = self._entities.values()
        if type:
            entities = [e for e in entities if e.is_a(type)]
        if radius:
            if ignore_z:
                entities = [e for e in entities if e.distance_to_2d(center_point_in_map.vector) <= radius]
            else:
                entities = [e for e in entities if e.distance_to_3d(center_point_in_map.vector) <= radius]
        if id:
            entities = [e for e in entities if e.id == id]

        return entities

    @property
    def _entities(self):
        entities = self._dynamic_entities.copy()
        entities.update(self._static_entities)
        return defaultdict(ED.generate_random_entity, entities.items())

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


class MockedTfBuffer(mock.MagicMock):
    def __init__(self):
        super(MockedTfBuffer, self).__init__()

    @staticmethod
    def can_transform(*args, **kwargs):
        return True

    @staticmethod
    def transform(point_stamped, frame_id):
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

        super(Mockbot, self).__init__(robot_name=robot_name, wait_services=False, tf_buffer=MockedTfBuffer())

        self.publish_target = AlteredMagicMock()
        self.tf_buffer_transform_pose = AlteredMagicMock()
        self.close = AlteredMagicMock()
        self.get_joint_states = AlteredMagicMock()

        self.add_body_part('hmi', Hmi(self.robot_name, self.tf_buffer))

        # Body parts
        self.add_body_part('base', Base(self.robot_name, self.tf_buffer))
        self.add_body_part('torso', Torso(self.robot_name, self.tf_buffer, self.get_joint_states))

        self.add_arm_part('leftArm', Arm(self.robot_name, self.tf_buffer, self.get_joint_states, "arm_left"))
        self.add_arm_part('rightArm', Arm(self.robot_name, self.tf_buffer, self.get_joint_states, "arm_right"))

        self.add_body_part('head', Head(self.robot_name, self.tf_buffer))

        # Human Robot Interaction
        self.add_body_part('speech', Speech(self.robot_name, self.tf_buffer))
        self.add_body_part('ebutton', EButton(self.robot_name, self.tf_buffer))
        self.add_body_part('lights', Lights(self.robot_name, self.tf_buffer))

        # Reasoning/world modeling
        self.add_body_part('ed', ED(self.robot_name, self.tf_buffer))
        self.add_body_part('perception', Perception(self.robot_name, self.tf_buffer))

        # Miscellaneous
        self.pub_target = AlteredMagicMock()

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

    def basego(x, y, phi):
        return mockbot.base.go(x, y, phi, frame="/mockbot/base_link")

    def insert_object(x, y, z):
        from test_tools.WorldFaker import WorldFaker
        wf = WorldFaker()
        wf.insert({"position": (x, y, z)})

    # Useful for making location-files
    def get_pose_2d():
        posestamped = mockbot.base.location
        loc, rot = posestamped.pose.point, posestamped.pose.orientation
        rot_array = [rot.w, rot.x, rot.y, rot.z]
        rot3 = tf_conversions.transformations.euler_from_quaternion(rot_array)
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

        mockbot.speech.speak("Please say: continue when I turn green", block=True)  # definitely block here, otherwise we hear ourselves

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
