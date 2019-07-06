# ROS
from threading import Event

import message_filters
import rospy
import smach
import std_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image

# TU/e Robotics
import robot_smach_states as states
import robot_smach_states.util.designators as ds

TRIGGER_TOPIC = "/trigger"
LIGHTSABER_WAYPOINT_ID = "hand_that_home_location"


all_possible_furniture = ['kitchen_cabinet',
                          'kitchen_table',
                          'island',
                          'sink',
                          'dishwasher',
                          'desk',
                          'coffee_table',
                          # 'couch',
                          # 'armchair',
                          'display_cabinet',
                          'sideboard']


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
        self._trigger_sub = rospy.Subscriber(TRIGGER_TOPIC, std_msgs.msg.Empty, self._trigger_callback, queue_size=1)

    def execute(self, ud=None):

        rospy.logwarn("To break out of the LightSaber execute hook, enter:"
                      "\n\n\trostopic pub --once trigger std_msgs/Empty '{}'\n\nin your terminal")

        self._robot.head.reset()

        self._robot.speech.speak("Let's show what I can do")
        self._event = Event()
        rate = rospy.Rate(10.0)
        rospy.loginfo("Starting main loop")
        while not rospy.is_shutdown() and not self._event.is_set():
            rgb, depth, depth_info = self._robot.get_rgb_depth_caminfo()
            if rgb:
                persons = self._robot.perception.detect_person_3d(rgb, depth, depth_info)
                persons = sorted(persons, key=lambda x: x.position.z)
                if persons:
                    person = persons[0]
                    if "is_pointing" in person.tags:
                        try:
                            map_pose = self._robot.tf_listener.transformPose("map", PoseStamped(
                                header=std_msgs.msg.Header(
                                    frame_id=rgb.header.frame_id,
                                    stamp=rospy.Time.now() - rospy.Duration.from_sec(0.5)
                                ),
                                pose=person.pointing_pose
                            ))
                            result = self._robot.ed.ray_trace(map_pose)  # type: RayTraceResponse
                        except Exception as e:
                            rospy.logerr("Could not get ray trace from closest person: {}".format(e))
                        else:
                            if result.entity_id in all_possible_furniture:
                                rospy.loginfo("Pointing at %s", result.entity_id)

        return "done"

    def _trigger_callback(self, _):
        """
        Callback for the trigger topic. Sets the event to signal the main loop to stop.
        """
        rospy.loginfo("Trigger received")
        self._event.set()


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
