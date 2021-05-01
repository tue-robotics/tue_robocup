# ROS
import actionlib
import rospy
from std_msgs.msg import String

# TMC
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice

# TU/e Robotics
from robot_skills.speech import SpeechInterface


class TmcSpeech(SpeechInterface):
    def __init__(self, robot_name, tf_listener, pre_hook=None, post_hook=None):
        """
        Speech interface to the TMC text-to-speech node.

        :param robot_name: name of the robot
        :param tf_listener: tf listener object
        :param pre_hook: method that is executed before speaking
        :param post_hook: method that is executed after speaking
        """
        super(TmcSpeech, self).__init__(
            robot_name=robot_name, tf_listener=tf_listener, pre_hook=pre_hook, post_hook=post_hook,
        )

        # Big question: how do we handle speech requests published on a topic (e.g., from the UI)?
        # Tiny question: soundboard is currently not implemented, is that desired?

        # Publish the sentence for other interfaces
        self.sentence_pub = rospy.Publisher("{}/output".format(robot_name), String, queue_size=1)

        # Client
        self.speech_client = self.create_simple_action_client("/talk_request_action", TalkRequestAction)

    # noinspection PyUnusedLocal
    def speak_impl(self, sentence, language, personality, voice, mood, block):
        # type: (string, string, string, string, string, bool) -> bool
        """
        Send a sentence to the text to speech module.

        When block=False, this method returns immediately.
        With the replace-dictionary, you can specify which characters to replace with what. By default, it replace
        underscores with spaces.

        :param sentence: string with sentence to pronounce
        :param language: string with language to speak; only 'us' can be used.
        :param personality: string indicating personality. Not used.
        :param voice: string indicating the voice to speak with. Not used.
        :param mood: string indicating the emotion. Not used.
        :param block: bool to indicate whether this function should return immediately or if it should block until the
            sentence has been spoken
        """
        str_msg = String(sentence)
        self.sentence_pub.publish(sentence)
        request = TalkRequestGoal()
        request.data.interrupting = False
        request.data.queueing = True
        if language not in ["us", "en"]:
            rospy.logwarn("TmcSpeech can only handle English, not {}".format(language))
        request.data.language = Voice.kEnglish
        request.sentence = sentence
        self.speech_client.send_goal(request)
        # ToDo: test if blocking works as desired
        if block:
            self.speech_client.wait_for_result()
        return True
