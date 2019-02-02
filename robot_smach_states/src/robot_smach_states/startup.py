# System
import math
from threading import Event

# ROS
from geometry_msgs.msg import Quaternion
import rospy
from sensor_msgs.msg import LaserScan
import smach

# TU/e Robotics
from robot_skills.util.kdl_conversions import quaternion_msg_to_kdl_rotation
import check_ebutton
import human_interaction
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
                                   transitions={"initialized": "INSTRUCT_WAIT_FOR_DOOR" if door else "INIT_POSE",
                                                "abort": "Aborted"})

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
                                   EnterArena(robot, initial_pose, use_entry_points),
                                   transitions={"done": "Done"})


# Enter the arena with force drive as back-up
class EnterArena(smach.StateMachine):
    class GoToEntryPoint(smach.State):
        def __init__(self, robot, initial_pose, use_entry_points=False):
            """
            not implemented
            :param robot: robot object
            :param initial_pose:
            :param use_entry_points:
            """
            smach.State.__init__(self, outcomes=["no_goal", "found", "not_found", "all_unreachable"])
            self.robot = robot
            self.initial_pose = initial_pose
            self.use_entry_points = use_entry_points

        def execute(self, userdata=None):
            print("TODO: IMPLEMENT THIS STATE")
            return "no_goal"

    class ForceDrive(smach.State):
        def __init__(self, robot):
            """
            Force drive through the door
            :param robot: robot object
            """
            smach.State.__init__(self, outcomes=["done"])
            self.robot = robot

        def execute(self, userdata=None):
            rospy.loginfo("{} uses force drive as a back-up scenario!".format(self.robot.robot_name))
            self.robot.base.force_drive(0.25, 0, 0, 5.0)  # x, y, z, time in seconds
            self.robot.ed.reset()
            return "done"

    def __init__(self, robot, initial_pose, use_entry_points=False):
        """
        Enter the arena by force driving through the door
        :param robot: robot object
        :param initial_pose:
        :param use_entry_points:
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
                                   self.ForceDrive(robot),
                                   transitions={"done": "GO_TO_ENTRY_POINT"})

            smach.StateMachine.add('GO_TO_ENTRY_POINT',
                                   self.GoToEntryPoint(robot, initial_pose, use_entry_points),
                                   transitions={"found": "done",
                                                "not_found": "GO_TO_ENTRY_POINT",
                                                "no_goal": "done",
                                                "all_unreachable": "done"})


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

        self.laser_upside_down = None
        self.laser_yaw = None
        self.laser_sub = None

    @staticmethod
    def avg(lst):
        """
        return the mean of the element of a list. NaNs are ignored
        :param lst: list of numbers
        :return: mean of the elements
        """
        lst = [point for point in lst if not math.isnan(point)]
        return sum(lst) / max(len(lst), 1)

    def process_scan(self, scan_msg):
        """
        callback function checking the distance of the laser points in front of the robot.
        :param scan_msg: sensor_msgs.msg.LaserScan
        :return: no return
        """
        try:
            number_beams = len(scan_msg.ranges)
            front_index = int(
                math.floor((self.laser_upside_down*self.laser_yaw - scan_msg.angle_min) / max(scan_msg.angle_increment,
                                                                                              1e-10)))

            if front_index < 2 or front_index > number_beams - 2:
                rospy.logerr("Base laser can't see in front of the robot")
                self.laser_sub.unregister()
                raise IndexError()  # TODO Matthijs: very ugly

            ranges_at_center = scan_msg.ranges[front_index - 2:front_index + 2]  # Get some points around the middle
            distance_to_door = self.avg(ranges_at_center)  # and the average of the middle range
            rospy.logdebug("AVG distance: {}".format(distance_to_door))
            self.distances += [distance_to_door]  # store all distances

            avg_distance_now = self.avg(self.distances[-5:])  # And the latest 5

            if avg_distance_now > 1.0:
                rospy.loginfo("Distance to door is more than a meter")
                self.door_open.set()  # Then set a threading Event that execute is waiting for.
        except Exception as e:
            rospy.logerr("Receiving laser failed so unsubscribing: {0}".format(e))
            self.laser_sub.unregister()

    def execute(self, userdata=None):
        rospy.loginfo("Waiting for door...")
        r = Quaternion()
        _, (r.x, r.y, r.z, r.w) = self._robot.tf_listener.lookupTransform(self._robot.base_link_frame,
                                                                          self._robot.robot_name + '/base_laser',
                                                                          rospy.Time(0))
        laser_rotation = quaternion_msg_to_kdl_rotation(r)
        self.laser_upside_down = -math.copysign(1, math.cos(laser_rotation.GetRPY()[0]))  # -1 normal, 1 upside down
        self.laser_yaw = laser_rotation.GetRPY()[2]
        self.laser_sub = rospy.Subscriber(self._robot.laser_topic, LaserScan,
                                          self.process_scan)

        opened_before_timout = self.door_open.wait(self.timeout)

        rospy.loginfo("Unregistering laser listener and clearing data")
        self.laser_sub.unregister()
        self.distances = []

        self.door_open.clear()
        if opened_before_timout:
            rospy.loginfo("Door is open")
            return "open"

        rospy.loginfo("Timed out with door still closed")
        return "closed"
