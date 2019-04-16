import smach
import robot_smach_states
import rospy

from PIL import Image
import cStringIO as StringIO
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from image_recognition_msgs.srv import Recognize
import cv2
from timeout import Timeout


def _get_cropped_image_from_info(info):        

    if len(info.measurement_image_unmasked) == 0:
        rospy.logerr("Received empty image from ED. This should not happen")
        return None

    try:
        byte_array = bytearray(info.measurement_image_unmasked)
        stream = StringIO.StringIO(byte_array)
        image = Image.open(stream)
    except Exception as e:
        rospy.logerr("Failed to load image ... Try installing the latest version of PILLOW: sudo pip install -I pillow")
        rospy.logerr(e)
        return None

    try:
        image_data = np.asarray(image)
        image_data_bw = image_data.max(axis=2)
        non_empty_columns = np.where(image_data_bw.max(axis=0)>0)[0]
        non_empty_rows = np.where(image_data_bw.max(axis=1)>0)[0]
        cropBox = (min(non_empty_rows), max(non_empty_rows), min(non_empty_columns), max(non_empty_columns))

        image_data_new = image_data[cropBox[0]:cropBox[1]+1, cropBox[2]:cropBox[3]+1 , :]

        cropped_image = Image.fromarray(image_data_new)
    except:
        rospy.logerr("Could not crop image, I will use the original image as cropped image")
        cropped_image = image

    return cropped_image


class OperatorFeedback(smach.State):
    def __init__(self, robot, selected_entity_designator):
        smach.State.__init__(self, outcomes=["cleanup", "no_cleanup"])
        self._robot = robot
        self._selected_entity_designator = selected_entity_designator
        self._bridge = CvBridge()
        self._service_proxy = rospy.ServiceProxy("/manual", Recognize)
        self._garbage_type_list = []

    def _request(self, img_msg):
        res = self._service_proxy(image=img_msg)
        return res

    def _request_with_timeout(self, img_msg, timeout):
        timeout_function = Timeout(self._request, timeout)
        return timeout_function(img_msg)

    def execute(self, userdata):
        e = self._selected_entity_designator.resolve()

        # If we have cached it, clean it up
        if e.type in self._garbage_type_list:
            self._robot.speech.speak("Oh wait, I already know this item, I will clean it up immediately.")
            return "cleanup"

        if not e:
            rospy.logerr("For some reason, the entity could not be resolved, this should not happen!")
            return "no_cleanup"

        info = self._robot.ed.get_entity_info(e.id)
        image = _get_cropped_image_from_info(info)

        if not image:
            rospy.logerr("For some reason, I could nog get an image from the entity info, this should not happen")
            return "no_cleanup"

        try:
            image = np.array(image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            img_msg = self._bridge.cv2_to_imgmsg(image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            self._robot.speech.speak("I can not get an image of this object, I will not clean it up")
            return "no_cleanup"

        try:
            res = self._request_with_timeout(img_msg, 40)
        except Exception as e:
            rospy.logerr("Service call failed:")
            rospy.logerr(e)
            self._robot.speech.speak("I could not reach my operator")
            return "no_cleanup"

        rospy.loginfo(res)

        label = ""
        if res.recognitions and res.recognitions[0].categorical_distribution.probabilities:
            label = res.recognitions[0].categorical_distribution.probabilities[0].label

        # For now if it is not recognized as something useful, we clean it up
        if not label or "trash" in label.lower():
            # Keep track of what object apperently is something ;; we do have the type of the classification
            if e.type:
                self._garbage_type_list.append(e.type)

            return "cleanup"

        return "no_cleanup"


class OperatorCleanup(smach.StateMachine):
    def __init__(self, robot, selected_entity_designator, location_id, segment_area):

        smach.StateMachine.__init__(self, outcomes=['cleanup', 'no_cleanup'])

        sentences = ["Calling my operator. Can you help me with the object %s the %s" % (segment_area, location_id),
                     "Contacting my Operator. Can you tell me more about the object %s the %s?" % (segment_area, location_id),
                     "I am asking my operator now about the object %s the %s?" % (segment_area, location_id)]

        with self:

            smach.StateMachine.add('SAY_CONTACTING_OPERATOR',
                                   robot_smach_states.Say(robot, sentences, block=True),
                                   transitions={"spoken": "OPERATOR_FEEDBACK"})

            smach.StateMachine.add('OPERATOR_FEEDBACK',
                                   OperatorFeedback(robot, selected_entity_designator),
                                   transitions={"cleanup": "SAY_CLEANUP", "no_cleanup": "SAY_NO_CLEANUP"})

            smach.StateMachine.add('SAY_CLEANUP',
                                robot_smach_states.Say(robot, ["Ok, I will cleanup the object",
                                                                "That's ok!",
                                                                "As you wish"], block=True),
                                transitions={"spoken": "cleanup"})

            smach.StateMachine.add('SAY_NO_CLEANUP',
                                robot_smach_states.Say(robot, ["Ok, I will leave the object here",
                                                                "That's ok! I will continue",
                                                                "Now I know that this object is not trash"], block=True),
                                transitions={"spoken": "no_cleanup"})
