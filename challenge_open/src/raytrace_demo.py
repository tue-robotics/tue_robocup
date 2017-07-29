# System
import math
import threading

# ROS
import tue_msgs.msg
import rospy
import smach
import std_msgs.msg
import geometry_msgs.msg


class RayTraceDemo(smach.State):
    """ State to perform raytrace demo
    """
    def __init__(self, robot, breakout_id):
        """ Constructor

        :param robot: robot object
        :param breakout_id: string identifying the breakout object. If the operator points to this object, the state
        will finish
        """
        smach.State.__init__(self, outcomes=["done"])

        # Robot object
        self.robot = robot
        self.breakout_id = breakout_id

        # Subscriber for people detections and publisher for visualization
        rospy.Subscriber("/amigo/persons", tue_msgs.msg.People, self._people_callback)
        rospy.Subscriber("/amigo/trigger", std_msgs.msg.String, self._trigger_callback)

        # Flag for the callback to indicate whether or not to process data
        self._active = False

        # Requested and active highlight
        self._requested_highlight = ""  # Set by the people callback
        self._active_highlight = ""  # Set by the blink thread

        # Blink properties
        self._blink_duration = 3.0
        self._blink_rate = 4.0

        # Backup trigger string
        self._trigger_string = "continue"

        # Furniture objects
        self._furniture_objects = []

    def execute(self, userdata):
        """ Execute function """
        rospy.logwarn("To stop this state manually, enter 'amigo-trigger-command {}'".format(self._trigger_string))

        # Get all furniture objects: this might come in handy
        entities = self.robot.ed.get_entities()
        self._furniture_objects = [e for e in entities if e.is_a("furniture")]

        # Look up
        self.robot.head.look_at_ground_in_front_of_robot(distance=100)

        # Enable callback
        self._active = True

        self._blink()

        # Disable callback
        self._active = False

        # clear stuff
        return "done"

    def _people_callback(self, msg):
        """ Callback function for people subscriber. If this state is active, the number of
        people raising their hands is counted and markers are published on their positions
        :param msg: tue_msgs/People message
        """

        # Check if active
        if not self._active:
            return

        # Check if there are people in the message
        if not msg.people:
            self._requested_highlight = ""
            return

        sorted_by_distance = sorted(msg.people, key=lambda p: p.position.x ** 2 +
                                                              p.position.y ** 2 +
                                                              p.position.z ** 2)
        closest_person = sorted_by_distance[0]

        if "is_pointing" not in closest_person.tags:
            self._requested_highlight = ""
            return

        map_pose = self.robot.tf_listener.transformPose(
            "map", geometry_msgs.msg.PoseStamped(header=msg.header, pose=closest_person.pointing_pose))

        # Perform raytrace
        raytraceresult = self.robot.ed.ray_trace(map_pose)
        entity_id = raytraceresult.entity_id

        # ToDo: say what's pointed to
        # e = self.robot.ed.get_entity(id=raytraceresult.entity_id)
        # if raytraceresult.entity_id != "" and e is not None and raytraceresult.entity_id != self._active_highlight:
        #     self.robot.speech.speak("You are pointing to the {}".format(raytraceresult.entity_id))
        # # Only do stuff if we have pointed to an entity and this is not already active
        # if entity_id != "" and entity_id != self._active_highlight:
        #     self._requested_highlight = raytraceresult.entity_id
        #     e = self.robot.ed.get_entity(id=self._requested_highlight)
        #
        # # ToDo: check for furniture

        # If we're pointing to the same thing
        if entity_id == self._active_highlight:
            self._requested_highlight = entity_id
            return

        # If empty: set requested empty
        if entity_id == "":
            self._requested_highlight = entity_id

        # Else: get it from ed, check if it is furniture
        e = self.robot.ed.get_entity(id=entity_id)
        if e.is_a("furniture"):
            self.robot.speech.speak("{}".format(entity_id.replace("_", " ")), block=False)
            self._requested_highlight = entity_id
        else:
            rospy.loginfo("{} is not furniture".format(entity_id))

    def _blink(self):
        """ Blink thread
        """
        # Start looping
        state = False
        rate = rospy.Rate(self._blink_rate)
        while self._active and not rospy.is_shutdown():

            # If the requested is not the same as the active highlight, update this
            if self._requested_highlight != self._active_highlight:

                # First: disable the old highlight
                if self._active_highlight:
                    self.robot.ed.update_entity(id=self._active_highlight, remove_flags=["highlighted"])

                # If necessary, activate the new highlight
                if self._requested_highlight:
                    self.robot.ed.update_entity(id=self._requested_highlight, add_flags=["highlighted"])
                    state = True

                # In any case: switch stuff
                self._active_highlight = self._requested_highlight

            else:
                # If the requested is the same as the active highlight: alternate and clear the request
                if state:
                    self.robot.ed.update_entity(id=self._active_highlight, remove_flags=["highlighted"])
                    state = False
                else:
                    self.robot.ed.update_entity(id=self._active_highlight, add_flags=["highlighted"])
                    state = True

            # Sleep for nice blinking
            rate.sleep()

        # Turn everything off and remove it from the map
        if self._active_highlight:
            self.robot.ed.update_entity(id=self._active_highlight, remove_flags=["highlighted"])

    def _trigger_callback(self, msg):
        """ Callback function for trigger topic. If the specified message is returned, this state will exit
        :param msg: string message
        """
        if msg.data == self._trigger_string:
            rospy.loginfo("Stopping laser demo by external trigger")
            self._active = False


# ------------------------------------------------------------------------------------------------

def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done'])

    with sm:

        # Start challenge via StartChallengeRobust, skipped atm
        smach.StateMachine.add("RAYTRACEDEMO",
                               RayTraceDemo(robot, breakout_id="kitchen_shelf"),
                               transitions={"done": "Done"})

    return sm


if __name__ == '__main__':
    import robot_smach_states
    rospy.init_node('test_raytrace_demo')
    robot_smach_states.util.startup(setup_statemachine, challenge_name="challenge_open")







