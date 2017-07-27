# ROS
import people_msgs.msg
import rospy
import smach
import visualization_markers

# TU/e Robotics
from robot_skills.util.transformations import tf_transform


class OrderCounter(smach.State):
    """ Smach state to count the number of people raising their hands to order something.
    """
    def __init__(self, robot):
        """ Constructor

        :param robot: robot object
        """
        smach.State.__init__(self, outcomes=["done"])

        # Robot object
        self.robot = robot

        # Subscriber for people detections and publisher for visualization
        rospy.Subscriber("/amigo/persons", people_msgs.msg.People, self._people_callback)
        self._marker_array_pub = rospy.Publisher('/amigo/thirsty_people',
                                                 visualization_markers.msg.MarkerArray, queue_size=1)

        # Flag for the callback to indicate whether or not to process data
        self._active = False

        # Number of thirsty people
        self._number_of_thirsty_people = -1

    def execute(self, userdata):
        """ Execute hook

        :param userdata:
        :return:
        """
        # Enable callback
        self._active = True

        # Talk to the audience
        self.robot.speech.speak("Hello dear people, my name is amigo.")
        self.robot.speech.speak("Are you thirsty?")
        self.robot.speech.speak("Raise your hand if you would like a drink")

        # Sleeping for 10 seconds to do magic
        # ToDo: create something useful
        rospy.loginfo("Counting orders")
        rospy.sleep(rospy.Duration(10.0))
        rospy.loginfo("Done counting orders")

        # Disable callback
        self._active = False

        # Summarize
        if self._number_of_thirsty_people > 0:
            self.robot.speech.speak("I've seen {} thirsty people, "
                                    "let's get some drinks".format(self._number_of_thirsty_people))

        # Return
        return "done"

    def _people_callback(self, msg):
        """ Callback function for people subscriber. If this state is active, the number of
        people raising their hands is counted and markers are published on their positions
        :param msg: people_msgs/People message
        """
        # Check if active
        if not self._active:
            return

        # Clear all markers
        self._clear_markers()

        # Iterate over all people
        marker_array_msg = visualization_markers.msg.MarkerArray()
        count = 0
        for person in msg.people:
            # If hand is not raised: continue
            if "raised_hand" not in person.tags:
                continue

            # Count
            count += 1

            # Create a marker message
            marker = self._create_marker_msg(header=msg.header, position=person.position,
                                             id=count)
            marker_array_msg.markers.append(marker)

        # Play sound if the number of people changed
        if count > self._number_of_thirsty_people:
			self.robot.speech.speak("ping")
		elif count > self._number_of_thirsty_people:
			self.robot.speech.speak("pong")

        # Remember the number of thirsty people
        self._number_of_thirsty_people = count

        # Publish the marker array message
        self._marker_array_pub.publish(marker_array_msg)

    def _create_marker_msg(self, header, position, object_id):
        """ Creates a marker message on the provided position
        :param header: header for the marker message
        :param position: geometry_msgs/Point of the position of the marker
        :param object_id: integer with the id for the marker
        """
        height = 2.0
        diameter = 0.4

        msg = visualization_markers.msg.Marker()
        msg.header.stamp = header.stamp
        msg.header.frame_id = "map"
        msg.id = object_id
        msg.type = visualization_markers.msg.Marker.CYLINDER
        msg.action = visualization_markers.msg.Marker.ADD
        msg.pose.position = tf_transform(position, header.frame_id, "map", self.robot.tf_listener)
        msg.pose.position.z = 0.5 * height  # Make sure the cylinder stands neatly on the floor
        msg.pose.orientation.w = 1.0
        msg.scale.x = diameter
        msg.scale.y = diameter
        msg.scale.z = height
        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.a = 0.75

        return msg

    def _clear_markers(self):
        """ Clears all previously published markers
        """
        marker_array_msg = visualization_markers.msg.MarkerArray()
        msg = visualization_markers.msg.Marker()
        msg.action = visualization_markers.msg.DELETEALL
        marker_array_msg.markers.append(msg)
        self._marker_array_pub.publish(marker_array_msg)


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
