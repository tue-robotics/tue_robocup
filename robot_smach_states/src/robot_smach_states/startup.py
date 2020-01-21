# System
from functools import partial
import math
from threading import Event

# ROS
from geometry_msgs.msg import Quaternion, PolygonStamped
import rospy
from sensor_msgs.msg import LaserScan
import smach

# TU/e Robotics
from robot_skills.util.kdl_conversions import quaternion_msg_to_kdl_rotation
import check_ebutton
import human_interaction
from navigation import ForceDrive
import utility


class StartChallengeRobust(smach.StateMachine):
    """
    Initialize, wait for the door to be opened and drive inside
    """

    def __init__(self, robot, initial_pose, use_entry_points=False, door=True):
        """
        Initialization method

        :param robot: (Robot)
        :param initial_pose: (str) identifies the (waypoint) entity to be used as initial pose. For testing purposes,
            a tuple(float, float, float) representing x, y and yaw in map frame can be used.
        :param use_entry_points: (bool) (not yet implemented)
        :param door: (bool) indicates whether to wait for a door to open and whether to 'force-drive' inside
        """
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted", "Failed"])
        assert hasattr(robot, "base")
        assert hasattr(robot, "speech")
        if use_entry_points:
            raise NotImplementedError("Use entry points is not yet implemented in StartChallengeRobust/EnterArena")

        with self:
            smach.StateMachine.add("NOTIFY_EBUTTON",
                                   check_ebutton.NotifyEButton(robot),
                                   transitions={"succeeded": "INITIALIZE"})

            smach.StateMachine.add("INITIALIZE",
                                   utility.Initialize(robot),
                                   transitions={"initialized": "INSTRUCT_WAIT_FOR_DOOR" if door else "WAIT_FOR_LOCAL_PLANNER",
                                                "abort": "Aborted"})

            # We wait till the  local planner is ready before starting the challenge,
            # otherwise the robot will not drive.
            smach.StateMachine.add("WAIT_FOR_LOCAL_PLANNER",
                                   WaitForLocalPlanner(robot, 10),
                                   transitions={"ready": "INIT_POSE",
                                                "timeout": "Aborted"})

            smach.StateMachine.add("INSTRUCT_WAIT_FOR_DOOR",
                                   human_interaction.Say(robot, ["Hi there, I will now wait until the door is opened",
                                                                 "I'm waiting for the door"], block=False),
                                   transitions={"spoken": "WAIT_FOR_DOOR"})

            # Start laser sensor that may change the state of the door if the door is open:
            smach.StateMachine.add("WAIT_FOR_DOOR",
                                   WaitForDoorOpen(robot, timeout=10),
                                   transitions={"closed": "DOOR_CLOSED",
                                                "open": "DOOR_OPEN"})

            smach.StateMachine.add("DOOR_CLOSED",
                                   human_interaction.Say(robot, ["Door is closed, please open the door",
                                                                 "I'd start, if only you'd let me in",
                                                                 "Please let me in"]),
                                   transitions={"spoken": "WAIT_FOR_DOOR"})

            smach.StateMachine.add("DOOR_OPEN",
                                   human_interaction.Say(robot, "Door is open!", block=False),
                                   transitions={"spoken": "INIT_POSE"})

            # Initial pose is set after opening door, otherwise snapmap will fail if door is still closed and initial
            # pose is set, since it is thinks the robot is standing in front of a wall if door is closed and
            # localization can(/will) be messed up.
            smach.StateMachine.add('INIT_POSE',
                                   utility.SetInitialPose(robot, initial_pose),
                                   transitions={'done': 'ENTER_ROOM' if door else "Done",
                                                'preempted': 'Aborted',
                                                # This transition will never happen at the moment.
                                                # It should never go to aborted.
                                                'error': 'ENTER_ROOM' if door else "Done"})

            # Enter the arena with force drive as back-up
            smach.StateMachine.add('ENTER_ROOM',
                                   EnterArena(robot),
                                   transitions={"done": "Done"})


# Enter the arena with force drive as back-up
class EnterArena(smach.StateMachine):
    def __init__(self, robot):
        """
        Enter the arena by force driving through the door

        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=['done'])
        self.robot = robot

        with self:
            # If the door is open, amigo will say that it goes to the registration table
            smach.StateMachine.add("THROUGH_DOOR",
                                   human_interaction.Say(robot, ["I will start my task now", "Let's rock and roll!",
                                                                 "Let's kick some ass!"], block=False),
                                   transitions={"spoken": "FORCE_DRIVE_THROUGH_DOOR"})

            smach.StateMachine.add('FORCE_DRIVE_THROUGH_DOOR',
                                   ForceDrive(robot, 0.25, 0, 0, 5.0),
                                   transitions={"done": "done"})


class WaitForDoorOpen(smach.State):
    def __init__(self, robot, timeout):
        """
        Wait till the average distance in front of the robot is bigger than 1 meter. Only point in front of the middle
        are taken into account

        :param robot: robot object
        :param timeout: timeout for waiting till the door is opened
        """
        smach.State.__init__(self, outcomes=["open", "closed"])
        self._robot = robot
        self.timeout = timeout
        # ToDo Loy: Keeping all of these is quite ugly. Would a ring buffer or collections.deque suffice?
        self.distances = []
        self.door_open = Event()

    @staticmethod
    def avg(lst):
        """
        return the mean of the element of a list. NaNs are ignored

        :param lst: list of numbers
        :return: mean of the elements
        """
        lst = [point for point in lst if not math.isnan(point)]
        return sum(lst) / max(len(lst), 1)

    def process_scan(self, laser_upside_down, laser_yaw, door_open, scan_msg):
        """
        callback function checking the distance of the laser points in front of the robot.

        :param laser_upside_down: (int) 1 as normal, -1 as upside down
        :param laser_yaw: yaw angle of the laser
        :param door_open: Event to be set when ready
        :param scan_msg: sensor_msgs.msg.LaserScan
        :return: no return
        """
        number_beams = len(scan_msg.ranges)
        front_index = int(
            math.floor((laser_upside_down*laser_yaw - scan_msg.angle_min) / max(scan_msg.angle_increment, 1e-10)))

        if front_index < 2 or front_index > number_beams - 2:
            raise IndexError("Base laser can't see in front of the robot")

        ranges_at_center = scan_msg.ranges[front_index - 2:front_index + 2]  # Get some points around the middle
        distance_to_door = self.avg(ranges_at_center)  # and the average of the middle range
        rospy.logdebug("AVG distance: {}".format(distance_to_door))
        self.distances += [distance_to_door]  # store all distances

        avg_distance_now = self.avg(self.distances[-5:])  # And the latest 5

        if avg_distance_now > 1.0:
            rospy.loginfo("Distance to door is more than a meter")
            door_open.set()  # Then set a threading Event that execute is waiting for.

    def execute(self, userdata=None):
        rospy.loginfo("Waiting for door...")
        r = Quaternion()
        _, (r.x, r.y, r.z, r.w) = self._robot.tf_listener.lookupTransform(self._robot.base_link_frame,
                                                                          self._robot.robot_name + '/base_laser',
                                                                          rospy.Time(0))
        laser_rotation = quaternion_msg_to_kdl_rotation(r)
        laser_upside_down = -math.copysign(1, math.cos(laser_rotation.GetRPY()[0]))  # -1 normal, 1 upside down
        laser_yaw = laser_rotation.GetRPY()[2]

        door_open = Event()
        laser_sub = rospy.Subscriber(self._robot.laser_topic, LaserScan,
                                     partial(self.process_scan, laser_upside_down, laser_yaw, door_open))

        opened_before_timout = door_open.wait(self.timeout)

        rospy.loginfo("Unregistering laser listener and clearing data")
        laser_sub.unregister()
        self.distances = []

        if opened_before_timout:
            rospy.loginfo("Door is open")
            return "open"

        rospy.loginfo("Timed out with door still closed")
        return "closed"


class WaitForLocalPlanner(smach.State):
    """
    Wait till a valid footprint message from the local planner is received. A footprint is valid if it contains at least
    3 points. If no valid message is received within the timeout, "timeout" is returned.
    """
    def __init__(self, robot, timeout):
        """
        Constructor

        :param robot: robot object
        :type robot: robot
        :param timeout: timeout
        :type timeout: (float, int)
        """
        smach.State.__init__(self, outcomes=["ready", "timeout"])
        self._robot = robot
        self._timeout = timeout

    @staticmethod
    def msg_cb(ready_event, msg):
        # type: (Event, PolygonStamped) -> None
        """
        Footprint message callback

        :param ready_event: Event to set when ready
        :type ready_event: Event
        :param msg: footprint message
        :type msg: PolygonStamped
        """
        if len(msg.polygon.points) >= 3:
            rospy.loginfo("Valid footprint received")
            ready_event.set()

    def execute(self, userdate=None):
        rospy.loginfo("Waiting for local planner footprint")
        footprint_topic = "/{}/local_planner/local_costmap/robot_footprint/footprint_stamped".\
            format(self._robot.robot_name)

        ready_event = Event()

        footprint_sub = rospy.Subscriber(footprint_topic, PolygonStamped, partial(self.msg_cb, ready_event))

        ready_before_timeout = ready_event.wait(self._timeout)

        rospy.loginfo("Unregistering footprint subscriber")
        footprint_sub.unregister()

        if ready_before_timeout:
            rospy.loginfo("Local Planner is ready")
            return "ready"

        rospy.loginfo("Local Planner not ready during timeout")
        return "timeout"
