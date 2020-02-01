# ROS
import rospy

# TU/e Robotics
from robot_skills.robot_part import RobotPart
from text_to_speech.srv import Speak, SpeakRequest


class Speech(RobotPart):
    """Interface to TTS-module"""

    def __init__(self, robot_name, tf_listener, pre_hook=None, post_hook=None):
        super(Speech, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._speech_service = self.create_service_client('/%s/text_to_speech/speak' % robot_name, Speak)
        self._pre_hook = pre_hook
        self._post_hook = post_hook

        self._default_language  = self.load_param('text_to_speech/language', 'us')
        self._default_voice     = self.load_param('text_to_speech/voice', 'kyle')
        self._default_character = self.load_param('text_to_speech/character', 'default')
        self._default_emotion   = self.load_param('text_to_speech/emotion', 'neutral')

    def close(self):
        pass

    def speak(self, sentence, language=None, personality=None, voice=None, mood=None, block=True, replace=None):
        """
        Send a sentence to the text to speech module.
        You can set language, personality, voice and mood for the Philips TTS.
        When block=False, this method returns immediately.
        With the replace-dictionary, you can specify which characters to replace with what. By default, it replace
        underscores with spaces.

        :param sentence: string with sentence to pronounce
        :param language: string with language to speak. Philips TTS supports English (us) Dutch (nl)
        :param personality: string indicating the personality. Supported are Default, Man, OldMan, OldWoman, Boy,
            YoungGirl, Robot, Giant, Dwarf, Alien
        :param voice: string indicating the voice to speak with. In English, "kyle" (default), "gregory" (French
            accent) and "carlos" (Spanish accent) are supported. The Dutch voices are "david" and "marjolijn"
        :param mood: string indicating the emotion. Supported are: Neutral, Friendly, Angry, Furious, Drill, Scared,
            Emotional, Weepy, Excited, Surprised, Sad, Disgusted, Whisper.
        :param block: bool to indicate whether this function should return immediately or if it should block until the
            sentence has been spoken
        :param replace: dictionary with replacement stuff
        """
        # ToDo: replace personality by character and mood by emotion. Furthermore, change the order of the arguments.
        if not language:
            language = self._default_language
        if not voice:
            voice = self._default_voice
        if not personality:
            personality = self._default_character
        if not mood:
            mood = self._default_emotion
        if replace is None:
            replace = {"_": " "}

        if hasattr(self._pre_hook, '__call__'):
            self._pre_hook()

        for orig, replacement in replace.iteritems():
            sentence = sentence.replace(orig, replacement)

        result = False
        try:
            # ToDo: test this. This just seems utterly wrong
            if language == 'nl' and not (personality in ['david', 'marjolijn']):
                personality = 'marjolijn'  # kyle doesn't work for NL
            rospy.loginfo("\x1b[1;32m'" + sentence + "'\x1b[0m")
            # The funny stuff around sentence is for coloring the output text in the console

            req = SpeakRequest()
            req.language      = language
            req.voice         = voice
            req.character     = personality
            req.emotion       = mood
            req.sentence      = sentence
            req.blocking_call = block
            resp1 = self._speech_service(req)
            result = resp1.error_msg == ""

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {0}".format(e))
            result = False
        except Exception as e:
            rospy.logerr("Something went seriously wrong: {}".format(e))
            result = False

        if hasattr(self._post_hook, '__call__'):
            self._post_hook()

        return result
