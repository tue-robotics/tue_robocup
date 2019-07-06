# ROS
from threading import Event

import message_filters
import rospy
import smach
import std_msgs.msg
from sensor_msgs.msg import CameraInfo, Image

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds

TRIGGER_TOPIC = "/trigger"
LIGHTSABER_WAYPOINT_ID = "hand_that_home_location"


class LightSaber(smach.State):
    def __init__(self, robot):
        """
        State that registers an image subscriber and continuously calls the people detection service. As soon as a
        trigger is received over a topic, the state exits

        :param robot: (Robot) api object
        """
        smach.State.__init__(self, outcomes=["done"])

        self._robot = robot
        self._ts = None
        self._event = Event()
        self._camera_base_ns = "{}/head_rgbd_sensor".format(robot.robot_name)

    def execute(self, ud=None):

        rospy.logwarn("To break out of the LightSaber execute hook, enter:"
                      "\n\n\trostopic pub --once trigger std_msgs/Empty '{}'\n\nin your terminal")

        self._robot.head.reset()

        self._robot.speech.speak("Let's show what I can do")
        self._event = Event()
        self._register_subscribers()
        rate = rospy.Rate(10.0)
        rospy.loginfo("Starting main loop")
        while not rospy.is_shutdown() and not self._event.is_set():
            rate.sleep()

        self._deregister_subscribers()
        return "done"

    def _image_callback(self, rgb, depth, depth_info):
        """
        Callback for received image data. Uses image data to call people detection service.

        :param rgb:
        :param depth:
        :param depth_info:
        """
        try:
            rospy.logdebug('Received rgb, depth, cam_info')
            image_data = (rgb, depth, depth_info)
            if any(image_data):
                t_start = rospy.Time.now()
                self._robot.perception.detect_person_3d(rgb, depth, depth_info)
                rospy.loginfo("Calling detect person 3d took {} seconds".format((rospy.Time.now() - t_start).to_sec()))
        except Exception as e:
            rospy.logerr("{}".format(e))

    def _trigger_callback(self, _):
        """
        Callback for the trigger topic. Sets the event to signal the main loop to stop.
        """
        rospy.loginfo("Trigger received")
        self._event.set()

    def _register_subscribers(self):
        """
        Registers the image subscribers. N.B.: this is based on robot_skills.perception.
        """
        rospy.loginfo("Registering image subscribers")
        # camera topics
        self._depth_info_sub = message_filters.Subscriber(
            '{}/depth_registered/camera_info'.format(self._camera_base_ns), CameraInfo)
        self._depth_sub = message_filters.Subscriber(
            '{}/depth_registered/image'.format(self._camera_base_ns), Image)
        self._rgb_sub = message_filters.Subscriber(
            '{}/rgb/image_raw'.format(self._camera_base_ns), Image)

        self._ts = message_filters.ApproximateTimeSynchronizer([self._rgb_sub, self._depth_sub, self._depth_info_sub],
                                                               queue_size=1,
                                                               slop=10)

        self._ts.registerCallback(self._image_callback)

        # trigger sub
        self._trigger_sub = rospy.Subscriber(TRIGGER_TOPIC, std_msgs.msg.Empty, self._trigger_callback, queue_size=1)

    def _deregister_subscribers(self):
        """
        Clears the callbacks and deletes the subcribers
        """
        rospy.loginfo("Deregistering image subscribers")
        self._ts.callbacks.clear()
        del self._ts, self._depth_info_sub, self._depth_sub, self._rgb_sub
        rospy.loginfo("Deregistration done")


class DriveAndSwordFight(smach.StateMachine):
    def __init__(self, robot):
        """
        Drives to the lightsaber pose and starts the lightsaber state.
        """
        smach.StateMachine.__init__(self, outcomes=["done"])

        with self:

            smach.StateMachine.add("NAVIGATE_TO_START",
                                   states.NavigateToWaypoint(
                                       robot=robot,
                                       waypoint_designator=ds.EntityByIdDesignator(robot, id=LIGHTSABER_WAYPOINT_ID),
                                   ),
                                   transitions={"arrived": "SWORDFIGHT",
                                                "unreachable": "SWORDFIGHT",  # Just take it from here
                                                "goal_not_defined": "SWORDFIGHT"})  # Just take it from here

            smach.StateMachine.add("SWORDFIGHT",
                                   LightSaber(robot),
                                   transitions={"done": "done"})
