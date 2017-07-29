# ROS
import sys
import tue_msgs.msg
import PyKDL as kdl
import rospy
import smach
import std_msgs.msg
import visualization_msgs.msg

# TU/e Robotics
from robot_skills.util.transformations import tf_transform


class OrderCounter(smach.State):
    """ Smach state to count the number of people raising their hands to order something.
    """
    def __init__(self, robot, room_id=None, beercounter=None):
        """ Constructor

        :param robot: robot object
        :param room_id: id of the room where the people should be (if None, check is skipped)
        :param beercounter: BeerCounter object to communicate the number of beers to other states
        """
        smach.State.__init__(self, outcomes=["done"])

        # Robot object
        self.robot = robot
        self.room_id = room_id
        self.beercounter = beercounter

        # Subscriber for people detections and publisher for visualization
        rospy.Subscriber("/amigo/persons", tue_msgs.msg.People, self._people_callback)
        rospy.Subscriber("/amigo/trigger", std_msgs.msg.String, self._trigger_callback)

        if 'left' in sys.argv:
            rospy.loginfo('handover detector left enabled')
            rospy.Subscriber('/' + robot.robot_name + '/handoverdetector_left/result', std_msgs.msg.Bool, self._on_handover)
            self._handover_left_on = rospy.Publisher('/' + robot.robot_name + '/handoverdetector_left/toggle_human2robot', std_msgs.msg.Bool, queue_size=1)
        if 'right' in sys.argv:
            rospy.loginfo('handover detector right enabled')
            rospy.Subscriber('/' + robot.robot_name + '/handoverdetector_right/result', std_msgs.msg.Bool, self._on_handover)
            self._handover_right_on = rospy.Publisher('/' + robot.robot_name + '/handoverdetector_right/toggle_human2robot', std_msgs.msg.Bool, queue_size=1)

        self._marker_array_pub = rospy.Publisher('/amigo/thirsty_people',
                                                 visualization_msgs.msg.MarkerArray, queue_size=1)

        # Flag for the callback to indicate whether or not to process data
        self._active = False

        # Number of thirsty people
        # self._number_of_thirsty_people = -1
        self._max_number_of_thirsty_people = 0

        # Backup trigger string
        self._trigger_string = "continue"

        # Room bounding box
        self._room_box_min = kdl.Vector()
        self._room_box_max = kdl.Vector()

    def execute(self, userdata):
        """ Execute hook

        :param userdata:
        :return:
        """
        rospy.logwarn("To stop this state manually, enter 'amigo-trigger-command {}'".format(self._trigger_string))

        # Check the room bounding box (if this fails: fix the worldmodel)
        if self.room_id is not None:
            entities = self.robot.ed.get_entities()
            rooms = [e for e in entities if e.is_a("room")]
            for room in rooms:
                if room.id == self.room_id:
                    self._room_box_min = room._pose * room.volumes["in"].min_corner
                    self._room_box_max = room._pose * room.volumes["in"].max_corner
                    break

        # Look up
        self.robot.head.reset()

        # Enable callback
        self._active = True

        # Talk to the audience
        self.robot.speech.speak("Hello dear people, my name is amigo.")
        self.robot.speech.speak("Are you thirsty?")
        # self.robot.speech.speak("Raise your hand if you would like a drink")

        # # Sleeping for 10 seconds to do magic
        # # ToDo: create something useful
        # rospy.loginfo("Counting orders")
        # rospy.sleep(rospy.Duration(10.0))
        # rospy.loginfo("Done counting orders")

        if 'left' in sys.argv:
            self._handover_left_on.publish(True)
        if 'right' in sys.argv:
            self._handover_right_on.publish(True)

        while self._active and not rospy.is_shutdown():
            rospy.sleep(rospy.Duration(0.5))

        # Disable callback
        self._active = False

        # Summarize
        if self._max_number_of_thirsty_people <= 0:
            self.robot.speech.speak("Okay, i will get some beer", block=False)
        elif self._max_number_of_thirsty_people == 1:
            self.robot.speech.speak("I've seen one thirsty person, "
                                    "let's get him something to drink", block=False)
        elif self._max_number_of_thirsty_people <= self.beercounter.MAX_COUNT:
            self.robot.speech.speak("I've seen {} thirsty people, "
                                    "let's get some drinks".format(self._max_number_of_thirsty_people), block=False)
        else:
            self.robot.speech.speak("I've seen {} thirsty people. I will start with three, "
                                    "let's get some drinks".format(self._max_number_of_thirsty_people), block=False)

        # Remember number of beers
        if self.beercounter is not None:
            self.beercounter.count = self._max_number_of_thirsty_people

        # Return
        return "done"

    def _people_callback(self, msg):
        """ Callback function for people subscriber. If this state is active, the number of
        people raising their hands is counted and markers are published on their positions
        :param msg: tue_msgs/People message
        """
        # Check if active
        if not self._active:
            return

        # Clear all markers
        self._clear_markers()

        # Iterate over all people
        marker_array_msg = visualization_msgs.msg.MarkerArray()
        count = 0
        for person in msg.people:
            # If hand is not raised: continue
            if "LWave" not in person.tags and "RWave" not in person.tags:
                continue

            # Create a marker message
            marker = self._create_marker_msg(header=msg.header, position=person.position,
                                             object_id=count)

            # Check if it is in the bounding box
            # ToDo: refactor (do transformation outside marker msg function for clarity)
            if self.room_id is not None:
                if not (self._room_box_min.x() < marker.pose.position.x < self._room_box_max.x() and
                        self._room_box_min.y() < marker.pose.position.y < self._room_box_max.y()):
                    continue

            # Count
            count += 1

            # Add the marker message
            marker_array_msg.markers.append(marker)

        # Play sound if the number of people changed
        if count > self._max_number_of_thirsty_people:
            self.robot.speech.speak("%d beer" % count)
            self._max_number_of_thirsty_people = count

        # Remember the number of thirsty people
        # self._number_of_thirsty_people = count
        # self._max_number_of_thirsty_people = max(count, self._max_number_of_thirsty_people)

        # Publish the marker array message
        self._marker_array_pub.publish(marker_array_msg)

    def _create_marker_msg(self, header, position, object_id):
        """ Creates a marker message on the provided position
        :param header: header for the marker message
        :param position: geometry_msgs/Point of the position of the marker
        :param object_id: integer with the id for the marker
        """
        height = 30
        diameter = 0.4

        msg = visualization_msgs.msg.Marker()
        msg.header.stamp = header.stamp
        msg.header.frame_id = "map"
        msg.id = object_id
        msg.type = visualization_msgs.msg.Marker.CYLINDER
        msg.action = visualization_msgs.msg.Marker.ADD
        msg.pose.position = tf_transform(position, header.frame_id, "map", self.robot.tf_listener)
        msg.pose.position.z = 0.5 * height  # Make sure the cylinder stands neatly on the floor
        msg.pose.orientation.w = 1.0
        msg.scale.x = diameter
        msg.scale.y = diameter
        msg.scale.z = height
        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.a = 0.3

        return msg

    def _clear_markers(self):
        """ Clears all previously published markers
        """
        marker_array_msg = visualization_msgs.msg.MarkerArray()
        msg = visualization_msgs.msg.Marker()
        msg.action = visualization_msgs.msg.Marker.DELETEALL
        marker_array_msg.markers.append(msg)
        self._marker_array_pub.publish(marker_array_msg)

    def _trigger_callback(self, msg):
        """ Callback function for trigger topic. If the specified message is returned, this state will exit
        :param msg: string message
        """
        if msg.data == self._trigger_string:
            rospy.loginfo("Stopping order counter by external trigger")
            self._active = False

    def _on_handover(self, data):
        rospy.loginfo("Stopping order counter by handover detect")
        self._active = False


# ------------------------------------------------------------------------------------------------

def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done'])

    with sm:

        # Start challenge via StartChallengeRobust, skipped atm
        smach.StateMachine.add("PEOPLE_COUNTER",
                               OrderCounter(robot),
                               transitions={"done": "Done"})

    return sm


if __name__ == '__main__':
    import robot_smach_states
    rospy.init_node('test_people_counter')
    robot_smach_states.util.startup(setup_statemachine, challenge_name="challenge_open")
