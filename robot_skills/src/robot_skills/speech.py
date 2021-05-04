# ROS
import rospy
from text_to_speech.srv import Speak, SpeakRequest

# TU/e Robotics
from robot_skills.robot_part import RobotPart


class SpeechInterface(RobotPart):
    def __init__(self, robot_name, tf_buffer, pre_hook=None, post_hook=None):
        """
        Interface to text-to-speech module of the robot

        :param robot_name: name of the robot
        :param tf_buffer: tf listener object
        :param pre_hook: method that is executed before speaking
        :param post_hook: method that is executed after speaking
        """
        super(SpeechInterface, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self._pre_hook = pre_hook
        self._post_hook = post_hook

        self._default_language = self.load_param('text_to_speech/language', 'us')
        self._default_voice = self.load_param('text_to_speech/voice', 'kyle')
        self._default_character = self.load_param('text_to_speech/character', 'default')
        self._default_emotion = self.load_param('text_to_speech/emotion', 'neutral')

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
        language = language or self._default_language
        voice = voice or self._default_voice
        personality = personality or self._default_character
        mood = mood or self._default_emotion
        replace = replace or {"_": " "}

        if callable(self._pre_hook):
            self._pre_hook()

        for orig, replacement in replace.items():
            sentence = sentence.replace(orig, replacement)

        result = self.speak_impl(
            sentence=sentence, language=language, personality=personality, voice=voice, mood=mood, block=block
        )

        if callable(self._post_hook):
            self._post_hook()

        return result

    def speak_impl(self, sentence, language, personality, voice, mood, block):
        # type: (string, string, string, string, string, bool) -> bool
        """
        Send a sentence to the text to speech module.

        When block=False, this method returns immediately.
        With the replace-dictionary, you can specify which characters to replace with what. By default, it replace
        underscores with spaces.

        :param sentence: string with sentence to pronounce
        :param language: string with language to speak.
        :param personality: string indicating the personality.
        :param voice: string indicating the voice to speak with.
        :param mood: string indicating the emotion.
        :param block: bool to indicate whether this function should return immediately or if it should block until the
            sentence has been spoken
        """
        raise NotImplementedError("speak_impl not implemented for {}".format(self.__class__.__name__))


class TueSpeech(SpeechInterface):
    def __init__(self, robot_name, tf_buffer, pre_hook=None, post_hook=None):
        """
        Speech interface to the TU/e text-to-speech node. If present, this uses Philips TTS; otherwise, it defaults to
        espeak

        :param robot_name: name of the robot
        :param tf_buffer: tf listener object
        :param pre_hook: method that is executed before speaking
        :param post_hook: method that is executed after speaking
        """
        super(TueSpeech, self).__init__(
            robot_name=robot_name, tf_buffer=tf_buffer, pre_hook=pre_hook, post_hook=post_hook,
        )
        self._speech_service = self.create_service_client('/%s/text_to_speech/speak' % robot_name, Speak)

    def speak_impl(self, sentence, language, personality, voice, mood, block):
        # type: (string, string, string, string, string, bool) -> bool
        """
        Send a sentence to the text to speech module.

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
        """
        try:
            # ToDo: test this. This just seems utterly wrong
            if language == 'nl' and not (personality in ['david', 'marjolijn']):
                personality = 'marjolijn'  # kyle doesn't work for NL
            rospy.loginfo("\x1b[1;32m'" + sentence + "'\x1b[0m")
            # The funny stuff around sentence is for coloring the output text in the console

            req = SpeakRequest()
            req.language = language
            req.voice = voice
            req.character = personality
            req.emotion = mood
            req.sentence = sentence
            req.blocking_call = block
            resp1 = self._speech_service(req)
            return resp1.error_msg == ""

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {0}".format(e))
            return False
        except Exception as e:
            rospy.logerr("Something went seriously wrong: {}".format(e))
            return False
