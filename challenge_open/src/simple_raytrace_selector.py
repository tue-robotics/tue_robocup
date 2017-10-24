# System
import math
import threading

# ROS
import PyKDL as kdl
import rospy
import smach
import std_msgs.msg
import geometry_msgs.msg

# TU/e Robotics
import hmi
from robot_skills.util.kdl_conversions import FrameStamped
import robot_smach_states.util.designators as ds
import tue_msgs.msg


class SimpleRayTraceSelector(smach.State):
    """ State to perform raytrace demo. The outcome depends on the provided speech command.
    """
    def __init__(self, robot, waypoint=None, furniture_designator=None):
        """ Constructor

        :param robot: robot object
        :param waypoint: string with waypoint id
        :param furniture_designator: EdEntityDesignator to be filled
        :param
        """
        smach.State.__init__(self, outcomes=["waypoint", "furniture", "grasp", "done"])

        # Robot object
        self.robot = robot
        self.waypoint_id = waypoint
        self.furniture_designator = furniture_designator

        # Subscriber for people detections and publisher for visualization
        rospy.Subscriber("/amigo/persons", tue_msgs.msg.People, self._people_callback)
        rospy.Subscriber("/amigo/trigger", std_msgs.msg.String, self._trigger_callback)

        # Flag for the callback to indicate whether or not to process data
        self._active = False  # To start/stop this state

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

        # Remember last raytraceresult
        self._last_entity_id = ""
        self._last_intersection_point = None

        # Speech grammar
        self.grammar = '''
        T["continue"] -> continue
        T["drive"] -> drive over there | move over there | go there | move there
        T["bring"] -> grab that | grab that thing | bring me that | bring me that thing | give me that thing
        '''
        # T["move"] -> move to the object

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

        # Start blink thread
        blink_thead = threading.Thread(target=self._blink)
        blink_thead.start()

        # Do speech
        result = self._do_speech()

        # Disable callback
        self._active = False

        # Wait for blink thread to finish
        blink_thead.join()

        # clear stuff
        return result

    def _wait_for_entity(self, timeout):
        """ Waits a maximum of <timeout> seconds for the operator to point to an entity to move towards or to
         grasp from.
        """
        tstart = rospy.Time.now()
        rate = rospy.Rate(10.0)
        target = ""
        while self._active and not rospy.is_shutdown() and (rospy.Time.now() - tstart).to_sec() < timeout:
            if self._last_entity_id != "":
                rospy.loginfo("Wait for entity: operator pointed to {}".format(self._last_entity_id))
                target = self._last_entity_id
                self._last_entity_id = ""  # Reset last entity
                return target
            rate.sleep()

        return target

    def _get_assignment(self):
        """ Asks the operator what to do
        :return: string with assignment (and "continue" in case of a TimeoutException)
        """
        self.robot.speech.speak("What can I do for you", block=True)
        try:
            sentence, assignment = self.robot.hmi.query("", self.grammar, "T", timeout=5.0)
            return assignment
        except hmi.TimeoutException:
            # We probably had a false "amigo" detection
            return "continue"

    def _ask_confirmation(self):
        """ Asks for confirmation
        :return: True if confirmed, False otherwise
        """
        try:
            result = self.robot.hmi.query('', 'T -> yes | no', 'T').sentence
            if result == 'yes':
                return True
            elif result == 'no':
                self.robot.speech.speak("I am sorry, I misunderstood")
                return False
        except hmi.TimeoutException:
            # robot did not hear the confirmation, so lets assume it's False
            return False

    def _do_speech(self):
        """ Performs the speech logic. One of the state outcomes is returned as a string """
        while not rospy.is_shutdown() and self._active:  # self._active: in case of fallbacks

            self.robot.hmi.restart_dragonfly()

            try:  # Big try loop for unexpected stuff

                # Checkout what the robot wants
                assignment = self._get_assignment()
                if assignment == "continue":
                    continue

                # Wait to see if the operator pointed to an entity
                self.robot.lights.set_color(255, 0, 255)
                target = self._wait_for_entity(timeout=5.0)

                # If no target: continue
                if target == "":
                    self.robot.speech.speak("I am sorry but I did not see where you want me to go")
                    continue

                # Check if feasible
                if assignment == "continue":
                    continue
                # elif assignment == "drive" and target != "":  # Drive do waypoint
                #     conf_sentence = "Do you want me to drive to that point"
                #     result = "waypoint"
                elif assignment == "drive":  # Drive to furniture
                    conf_sentence = "Do you want me to drive to the {}".format(target)
                    result = "furniture"
                elif assignment == "bring" and target != "":  # Grab something
                    conf_sentence = "Do you want me to grasp something from the {}".format(target)
                    result = "grasp"
                # elif assignment == "bring" and target == "":
                #     self.robot.speech.speak("I am sorry but i cannot grasp anything from the floor", block=True)
                else:
                    rospy.logwarn("Something went terribly wrong, I cannot process {}".format(assignment))
                    continue

                # Ask for confirmation
                self.robot.speech.speak(conf_sentence, block=True)
                if self._ask_confirmation():
                    self._set_furniture_designator(target)
                    return result
                else:
                    continue
            except Exception as e:
                rospy.logerr("RaytraceSelector: Something went terribly wrong in speech: %s", e)
                continue

        return "done"

    # def _set_waypoint(self, intersection_point, person_position):
    #     """ Puts the goal as a waypoint in ED
    #     :param intersection_point: geometry_msgs PointStamped of the last raytrace intersection (in map)
    #     :param person_position: geometry_msgs PointStamped of the last measured person position (in map)
    #     """
    #     yaw = math.atan2(person_position.point.y - intersection_point.point.y,
    #                      person_position.point.x - intersection_point.point.x)
    #     position = kdl.Vector(intersection_point.point.x, intersection_point.point.y, 0.0)
    #     orientation = kdl.Rotation.RPY(0.0, 0.0, yaw)
    #     waypoint = FrameStamped(frame=kdl.Frame(orientation, position), frame_id="/map")
    #     self.robot.ed.update_entity(id=self.waypoint_id, type="waypoint", frame_stamped=waypoint)
    #     # import ipdb;ipdb.set_trace()
    #     return

    def _set_furniture_designator(self, identifier):
        """ Sets the id of the furniture designator
        :param identifier: string with furniture ID
        """
        # import ipdb;ipdb.set_trace()
        self.furniture_designator.id_ = identifier
        return

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

        # Person pose in map frame
        # person_position = self.robot.tf_listener.transformPoint(
        #     "map", geometry_msgs.msg.PointStamped(header=msg.header, point=closest_person.position))

        # Pointing pose in map frame
        map_pose = self.robot.tf_listener.transformPose(
            "map", geometry_msgs.msg.PoseStamped(header=msg.header, pose=closest_person.pointing_pose))

        # Perform raytrace
        raytraceresult = self.robot.ed.ray_trace(map_pose)
        entity_id = raytraceresult.entity_id

        # Remember results
        if entity_id == "" or "wall" in entity_id or "floor" in entity_id:  # Remember the waypoint on the floor
            self._last_intersection_point = self.robot.tf_listener.transformPoint("map",
                                                                                  raytraceresult.intersection_point)
            self._last_entity_id = ""
            # self._set_waypoint(self._last_intersection_point, person_position)
        else:  # Remember the entity id
            self._last_intersection_point = None
            self._last_entity_id = entity_id
            rospy.logwarn("Saw entity id: {}".format(entity_id))
            # self._set_furniture_designator(entity_id)

        # If we're pointing to the same thing
        if entity_id == self._active_highlight:
            self._requested_highlight = entity_id
            return

        # If empty: set requested empty
        if entity_id == "":
            self._requested_highlight = entity_id

        # Else: get it from ed, check if it is furniture
        e = self.robot.ed.get_entity(id=entity_id)
        if e.is_a("furniture") and "wall" not in e.id:
            # self.robot.speech.speak("{}".format(entity_id.replace("_", " ")), block=False)
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
                               SimpleRayTraceSelector(robot, waypoint="final_challenge",
                                                furniture_designator=ds.EntityByIdDesignator(robot, id="temp")),
                               transitions={outcome: "Done" for outcome in ["waypoint", "furniture", "grasp", "done"]})

    return sm


if __name__ == '__main__':
    import robot_smach_states
    rospy.init_node('test_raytrace_demo')
    robot_smach_states.util.startup(setup_statemachine, challenge_name="challenge_open")








