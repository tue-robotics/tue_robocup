# System
import numpy as np
from threading import Event

# ROS
from geometry_msgs.msg import PolygonStamped, Point, PoseStamped, Pose
import rospy
from sensor_msgs.msg import LaserScan
import smach
from visualization_msgs.msg import Marker, MarkerArray

# TU/e Robotics
from cb_planner_msgs_srvs.msg import PositionConstraint
from ed_msgs.msg import EntityInfo
from robot_skills.base import computePathLength
import robot_smach_states as states
import robot_smach_states.util.designators as ds


class ForceDriveToTouchDoor(smach.State):
    """
    Once a door is no longer locked, the lever is not latched into locking plate, it can be pushed open.
    This can be done either with the arms or even with the base, if you push gently,
        as a person would use his/her butt to bump open a door when you're holding something with both hands :-)

    Assumptions: We are already in front of the door, e.g. on the door mat so to speak.

    To do this, the following steps are needed:
     - Move slowly to put the edge of the base against the door
     - Now move forward, perpendicular to the wall the door is in.

    Test in amigo-console with
    do = state_machine.ForceDriveToTouchDoor(amigo, None); do.execute(); do.execute()
    """

    def __init__(self, robot, approach_speed=0.1):
        smach.State.__init__(self, outcomes=['front', 'left', 'right', 'failed'])
        self.robot = robot

        self.approach_speed = approach_speed

        self.debug_vizualizer = rospy.Publisher("visualization_marker_array", MarkerArray)

    def execute(self, userdata=None):
        # Align with the door frame; the X-axis should be perpendicular to the wall; y parallel to the wall

        # Get a laserscan message from topic /{robotname}/base_laser/scan
        # Convert this to a list of cartesian points in base_link for easier calculations
        #   (the assumption that the base_laser is in line with the base_link X-axis makes this easier, just addition)

        # Get footprint of the robot: topic /{robotname}/local_planner/local_costmap/robot_footprint/footprint_stamped
        # Convert this to /base_link coordinates

        # Find a distance (in base_link X+) when the footprint and the scan will intersect
        # Sample the X distances for the conversion of the footprint above or do this algebraically?

        # To determine the side, get the intersection point and the Y coordinate. If in y-, then right, if y+ then left.

        np.set_printoptions(precision=4, suppress=True)

        footprint = self.get_footprint()
        footprint = self.select_frontside_of_footprint(footprint)

        scan = self.get_laserscan()
        scan_in_bl = self.scan_to_base_link_points(scan)
        scan_in_bl = self.select_scanpoints_in_front_of_footprint(scan_in_bl, footprint)

        footprint_point, drive_dist, scan_point = self.find_drive_distance_to_first_obstacle(footprint, scan_in_bl, stepsize=0.01)

        self.publish_debug_info(footprint_point, drive_dist, scan_point)

        collision_side = "front"
        if footprint_point[1] > 0: collision_side = 'left' # Left side
        else: collision_side = 'right'  # Right side

        drive_time = drive_dist / self.approach_speed

        rospy.loginfo("Will drive {}m forward towards door. "
                      "Collision will happen on my {} side".format(drive_dist, collision_side))

        if self.robot.base.force_drive(self.approach_speed, 0, 0, drive_time*0.9):
            return collision_side
        else:
            return "failed"

    def get_laserscan(self):
        """
        Subscribe to laserscans and unsubscribe when we get one.
        This is realized through a threading.Event which is set when a message is received
        :return: the most recent laser scan
        :rtype LaserScan
        """
        received = Event()

        laserscan = [None]
        def get_scan(scan):
            laserscan[0] = scan  # Going via a list is apparently the way to have a proper closure. Py3 has 'nonlocal'
            received.set()

        laser_sub = rospy.Subscriber("/{}/base_laser/scan".format(self.robot.robot_name), LaserScan, get_scan)
        received.wait(5.0)

        laser_sub.unregister()

        return laserscan[0]

    def get_footprint(self):
        """
        Subscribe to footprint and unsubscribe when we get one.
        This is realized through a threading.Event which is set when a message is received
        :return:
        """
        received = Event()

        footprint = [None]
        def get_footprint(fp):
            # print "Received footprint"
            footprint[0] = fp
            received.set()

        footprint_sub = rospy.Subscriber("/{}/local_planner/local_costmap/robot_footprint/footprint_stamped".
                                         format(self.robot.robot_name), PolygonStamped, get_footprint)

        received.wait(5.0)
        footprint_sub.unregister()
        footprint = footprint[0]

        posestampeds = [PoseStamped(footprint.header, Pose(position=point)) for point in footprint.polygon.points]
        in_baselink = [self.robot.tf_transform_pose(ps, '{}/base_link'.format(self.robot.robot_name)) for ps in posestampeds]

        X = [ps.pose.position.x for ps in in_baselink]
        Y = [ps.pose.position.y for ps in in_baselink]
        # Z = [ps.point.z for ps in in_baselink]
        as_array = np.array([X, Y]).T

        return as_array

    def select_frontside_of_footprint(self, footprint_points):
        return footprint_points[footprint_points[:,0] > 0]

    def select_scanpoints_in_front_of_footprint(self, scan_points, footprint, tolerance=0.1):
        y_min = np.min(footprint[:,1])-tolerance
        y_max = np.max(footprint[:,1])+tolerance

        larger_than_min = scan_points[scan_points[:,1] > y_min]
        smaller_than_max = larger_than_min[larger_than_min[:,1] < y_max]

        return smaller_than_max

    def scan_to_base_link_points(self, scan):
        """
        Convert a sensor_msgs.msg.LaserScan message to a list of point in base_link frame
        Returns a numpy.ndarray of shape (N, 2)

        :param scan:
        :type scan: LaserScan
        :return:
        """

        #angles = np.arange(scan.angle_min, scan.angle_max-scan.angle_increment, scan.angle_increment)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        X = scan.ranges * np.cos(angles) # TODO: get distance between laser frame and base_link
        Y = scan.ranges * np.sin(angles)

        posestampeds = [PoseStamped(scan.header, Pose(position=Point(x,y, 0))) for (x, y) in zip(X, Y)]
        in_baselink = [self.robot.tf_transform_pose(ps, '{}/base_link'.format(self.robot.robot_name)) for ps in posestampeds]

        Xb = [ps.pose.position.x for ps in in_baselink]
        Yb = [ps.pose.position.y for ps in in_baselink]

        return np.array([Xb,Yb]).T

    def add_delta_to_points(self, points, delta):
        """
        Adds a distance to a list of points. Points should be an (N, 2)-shaped numpy.ndarray
        and delta must be a (2)-shaped np.ndarray.

        :param points:
        :param delta:
        :return:
        """
        return np.add(points, delta)

    def closest_other_point(self, point, points):
        dx = points[:,0] - point[0]
        dy = points[:,1] - point[1]

        distances = np.hypot(dx, dy)
        smallest_index = np.argmin(distances)
        return (distances[smallest_index], points[smallest_index])

    def distance_between_footprint_and_scan(self, footprint_points, scan_points):
        """
        Find the distance between the two closest points in the two sets of points and return those points as well.

        :param footprint_points: Points (in base_link) that define the base's footprint
        :param scan_points: Points (in base_link) that are scanned by the front laser
        :return: tuple of (footprint_point, distance, scan_point)
        """

        fp_dist_sp = [(point, self.closest_other_point(point, scan_points)) for point in footprint_points]
        # fp_dist_sp is a list of (footprint_point, (distance, scan_point))-tuples
        smallest = min(fp_dist_sp, key=lambda pair: pair[1][0])

        return (smallest[0], smallest[1][0], smallest[1][1])

    def find_drive_distance_to_first_obstacle(self, footprint_points, scan_points, stepsize=0.025):
        """
        Samples the distance with which to shift the footprint_points forwards (through a delta in X)
        and find the deltaX that makes for the smallest distance between these sets of points.

        This value is returned with the footprint_point at which this happens.

        :param footprint_points:
        :param scan_points:
        :return: A tuple giving the footprint point closest to the scan, distance to the scan and scan point closest to the footprint
        :rtype tuple of (np.array([x, y]), distance, np.array([x, y]))
        """

        def calc(forward_travel):
            shifted_points = self.add_delta_to_points(footprint_points, delta=np.array([forward_travel, 0]))
            fp, distance, sp = self.distance_between_footprint_and_scan(shifted_points, scan_points)
            # print "If I drive {0:.3f}, distance will be {2} between {1} and {3}".format(forward_travel, fp, distance, sp)

            original_fp = self.add_delta_to_points(fp, delta=np.array([-forward_travel, 0]))
            return original_fp, distance, sp

        forward_travels = np.arange(0, 1, stepsize)
        fp_distance_sp = map(calc, forward_travels)

        distances = np.array([fp_dist_sp[1] for fp_dist_sp in fp_distance_sp])

        smallest_distance_index = np.argmin(distances)

        forward_travel_to_minimize_distance = forward_travels[smallest_distance_index]
        footprint_point_closest_to_scan = fp_distance_sp[smallest_distance_index][0]
        scan_point_closest_to_footprint = fp_distance_sp[smallest_distance_index][2]

        return (footprint_point_closest_to_scan, forward_travel_to_minimize_distance, scan_point_closest_to_footprint)

    def publish_debug_info(self, footprint_point, distance, scan_point):
        array = MarkerArray()

        fp = Marker()
        fp.id = 1
        fp.type = fp.SPHERE
        scale = 0.1
        fp.pose.position.x,fp.pose.position.y, fp.pose.position.z = footprint_point[0], footprint_point[1], 0
        fp.scale.x, fp.scale.y, fp.scale.z = scale, scale, scale
        fp.color.r, fp.color.g, fp.color.b, fp.color.a = (0, 0, 1, 1)
        fp.header.frame_id = "{}/base_link".format(self.robot.robot_name)
        fp.frame_locked = True
        fp.action = fp.ADD
        fp.ns = "door_opening"
        array.markers += [fp]

        sp = Marker()
        sp.id = 2
        sp.type = sp.SPHERE
        scale = 0.1
        sp.pose.position.x,sp.pose.position.y, sp.pose.position.z = scan_point[0], scan_point[1], 0
        sp.scale.x, sp.scale.y, sp.scale.z = scale, scale, scale
        sp.color.r, sp.color.g, sp.color.b, sp.color.a = (1, 0, 0, 1)
        sp.header.frame_id = "{}/base_link".format(self.robot.robot_name)
        sp.frame_locked = False
        sp.action = sp.ADD
        sp.ns = "door_opening"
        array.markers += [sp]

        self.debug_vizualizer.publish(array)


class PushPerpendicularToDoor(smach.State):
    """
    Get the angle of the door via the laser or world model and
    force_drive to push with a force perpendicular to the door plane
    """
    pass


class PushSidewaysAndBack(smach.State):
    """
    Drive sideways for some distance (in base_link) and move back again

    Test in amigo-console with state_machine.PushSidewaysAndBack(amigo, 0.2, 0.05).execute()
    """
    def __init__(self, robot, y_dist, speed=0.1):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.y_dist = y_dist
        self.speed = speed

    def execute(self, userdata=None):
        duration = self.y_dist / self.speed
        if self.robot.base.force_drive(0, self.speed, 0, duration):  # There
            if self.robot.base.force_drive(0, -self.speed, 0, duration):  # and back again
                return 'succeeded'
            else:
                return 'failed'
        else:
            return 'failed'


class CheckDoorPassable(smach.State):
    """Check whether a given door can be passed. The precondition is that the robot is in front of said door.

    Test in amigo-console with
    state_machine.CheckDoorPassable(amigo, ds.EdEntityDesignator(amigo, id='door_navigation')).execute() """

    def __init__(self, robot, destination_designator, door_entity_designator=None):
        """
        :param robot: Robot on which to perform this state
        :param destination_designator: The destination to reach through this door.
        :param door_entity_designator: The door which to pass
        :return:
        """
        smach.State.__init__(self, outcomes=['passable', 'blocked'])

        self.robot = robot
        self.door_entity_designator = door_entity_designator
        self.destination_designator = destination_designator

    def execute(self, userdata=None):
        e = self.destination_designator.resolve()

        if not e:
            rospy.logerr("CheckDoorPassable::execute: No entity could be resolved from designator '%s'" % self.destination_designator)
            return None

        try:
            x = e.pose.position.x
            y = e.pose.position.y
        except:
            return None

        pc = PositionConstraint(constraint="(x-%f)^2+(y-%f)^2 < %f^2"%(x, y, 0.5), frame="/map")
        plan = self.robot.base.global_planner.getPlan(pc)

        if plan and len(plan) < 3 and computePathLength(plan) < 3.0:
            return "passable"
        else:
            return "blocked"


class WaypointOfDoorDesignator(ds.Designator):
    """
    Resolves to a waypoint associated with a door-entity
    """
    def __init__(self, robot, door_entity_designator, direction, name=None):
        """
        :param robot: The robot with which to query ED
        :param door_entity_designator: Designator resolving to an entity of type door that has a push_start_waypoint and push_destination_waypoint data field
        :param direction: 'start' or 'destination' to select one of the two data fields
        :param name: Name for this designator to aid in debugging.
        """
        super(WaypointOfDoorDesignator, self).__init__(resolve_type=EntityInfo, name=name)
        self.robot = robot

        ds.check_resolve_type(door_entity_designator, EntityInfo)
        self.door_entity_designator = door_entity_designator
        self.data_field = {'start':'push_start_waypoint', 'destination':'push_destination_waypoint'}[direction]

    def _resolve(self):
        rospy.logdebug("Finding from where to open a door")
        door_entity = self.door_entity_designator.resolve()
        import ipdb; ipdb.set_trace()
        if door_entity:
            waypoint_id = door_entity.data[self.data_field]
            waypoint = self.robot.ed.get_entity(id=waypoint_id)
            if waypoint:
                rospy.loginfo("Door {} should be opened from {}".format(door_entity.id, waypoint.id))
                return waypoint
        return None


class OpenDoorByPushing(smach.StateMachine):
    """
    Test in amigo-console with
    door = ds.EdEntityDesignator(amigo, id='door1'); do = state_machine.OpenDoorByPushing(amigo, door); print do.execute();
    """
    def __init__(self, robot, door_start_wp_designator, door_dest_wp_designator, approach_speed=0.1, push_speed=0.05, attempts=10):
        """
        Push against a door until its open

        :param robot: Robot on which to execute this state machine
        :param door_entity_designator: The door entity. Defaults to None, which implies the door its in front of
            This entity must have 2 fields in its data: push_start_waypoint and push_destination_waypoint,
            which must both contain an ID of a waypoint.
            push_start_waypoint is from where the robot will start pushing
            push_destination_waypoint simply needs to be reachable for the door the be considered 'open'
        :param approach_speed: Speed with which to approach the door
        :param push_speed: Speed with which to push against the door
        :return:
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed'])

        #door_start_wp_designator = WaypointOfDoorDesignator(robot, door_entity_designator, direction='start', name='door_open_start')
        #door_dest_wp_designator = WaypointOfDoorDesignator(robot, door_entity_designator, direction='destination', name='door_open_dest')

        with self:
            smach.StateMachine.add( 'GOTO_DOOR_START',
                                            states.NavigateToWaypoint(robot, door_start_wp_designator, radius=0.05),
                                            transitions={'arrived'          :'PUSH_DOOR_ITERATOR',
                                                         'unreachable'      :'failed',
                                                         'goal_not_defined' :'failed'})

            # START REPEAT DOOR OPENING

            push_door_iterator = smach.Iterator(outcomes=['open', 'closed', 'failed'],
                                                it = lambda:range(0, attempts),
                                                it_label='counter',
                                                input_keys=[],
                                                output_keys=[],
                                                exhausted_outcome = 'failed')
            with push_door_iterator:
                push_door = smach.StateMachine( outcomes = ['open', 'closed', 'failed'])
                with push_door:
                    smach.StateMachine.add( 'APPROACH_AGAINST_N',
                                            ForceDriveToTouchDoor(robot, approach_speed),
                                            transitions={'front':'closed',
                                                        'left':'CHECK_DOOR_PASSABLE',
                                                        'right':'CHECK_DOOR_PASSABLE',
                                                        'failed':'failed'})

                    smach.StateMachine.add( 'CHECK_DOOR_PASSABLE',
                                            CheckDoorPassable(robot,
                                                              destination_designator=door_dest_wp_designator),
                                            transitions={'blocked':'closed',
                                                         'passable':'open'})

                smach.Iterator.set_contained_state( 'APPROACH_AGAINST',
                                                     push_door,
                                                     loop_outcomes=['failed', 'closed'],
                                                     break_outcomes=['open'])

            smach.StateMachine.add( 'PUSH_DOOR_ITERATOR',
                                    push_door_iterator,
                                    {   'open':'succeeded',
                                        'closed':'failed',
                                        'failed':'failed'})
            # END REPEAT DOOR OPENING

            # smach.StateMachine.add('PUSH_LEFT',
            #                        PushSidewaysAndBack(robot, y_dist=0.1, speed=push_speed),
            #                        transitions={'succeeded':'succeeded',
            #                                     'failed':'failed'})
            #
            # smach.StateMachine.add('PUSH_RIGHT',
            #                        PushSidewaysAndBack(robot, y_dist=-0.1, speed=push_speed),
            #                        transitions={'succeeded':'succeeded',
            #                                     'failed':'failed'})
