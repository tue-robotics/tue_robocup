# System
import threading

# ROS
import people_msgs
import rospy
import smach
# import visualization_markers


class RayTraceDemo(smach.State):
    """ State to perform raytrace demo
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
        # self._marker_array_pub = rospy.Publisher('/amigo/thirsty_people',
        #                                          visualization_markers.msg.MarkerArray, queue_size=1)

        # Flag for the callback to indicate whether or not to process data
        self._active = False

        # Currently highighted objects. Maps object ids to the stamp where they were last pointed to
        self._highlighted = {}

        # Blink properties
        self._blink_duration = 3.0
        self._blink_rate = 4.0

    def execute(self, userdata):
        """ Execute function """
        return "done"

    def _people_callback(self, people):
        """ Callback function for people subscriber. If this state is active, the number of
        people raising their hands is counted and markers are published on their positions
        :param people: people_msgs/People message
        """
        # Check if active
        if not self._active:
            return

    def _blink(self, object_id):
        """ Function to be spawned as a separate thread thread. Sets highlight on and off on a fixed rate for a fixed
        duration. If this object id is already begin highlighted, the corresponding stamp is updated.

        :param object_id: id of the object to highlight
        """
        # Check if already highlighted. If so: updated the stamp and return
        if object_id in self._highlighted:
            self._highlighted[object_id] = rospy.Time.now()
            return

        # Add it to the map
        self._highlighted[object_id] = rospy.Time.now()

        # Start looping
        state = False
        rate = rospy.Rate(self._blink_rate)
        while (rospy.Time.now() - self._highlighted[object_id]).to_sec() < self._blink_duration:
            if state:
                self.robot.ed.update_entity(id=object_id, remove_flags=["highlighted"])
                state = False
            else:
                self.robot.ed.update_entity(id=object_id, add_flags=["highlighted"])
                state = True
            rate.sleep()

        # Turn everything off and remove it from the map
        self.robot.ed.update_entity(id=object_id, remove_flags=["highlighted"])
        # self._









