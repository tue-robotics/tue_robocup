__author__ = 'amigo'

import rospy
import smach
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped, PointStamped, Point, PoseStamped, Pose
from threading import Event
from visualization_msgs.msg import Marker, MarkerArray

class ForceDriveToTouchDoor(smach.State):
    """
    Once a door is no longer locked, the lever is not latched into locking plate, it can be pushed open.
    This can be done either with the arms or even with the base, if you push gently,
        as a person would use his/her butt to bump open a door when you're holding something with both hands :-)

    Assumptions: We are already in front of the door, e.g. on the door mat so to speak.

    To do this, the following steps are needed:
     - Move slowly to put the edge of the base against the door
     - Now move forward, perpendicular to the wall the door is in.
    """

    def __init__(self, robot, door_entity_designator, approach_speed=0.1):
        smach.State.__init__(self, outcomes=['front', 'left', 'right', 'failed'])
        self.robot = robot
        self.door_entity_designator = door_entity_designator

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


        scan = self.get_laserscan()
        scan_in_bl = self.scan_to_base_link_points(scan)

        footprint = self.get_footprint()

        footprint_point, drive_dist, scan_point = self.find_drive_distance_to_first_obstacle(footprint, scan_in_bl)

        self.publish_debug_info(footprint_point, drive_dist, scan_point)

        collision_side = "front"
        if footprint_point[1] > 0: collision_side = 'left' # Left side
        else: collision_side = 'right'  # Right side

        drive_time = drive_dist / self.approach_speed

        rospy.loginfo("Will drive {}m forward towards door. "
                      "Collision will happen on my {} side".format(drive_dist, collision_side))

        import ipdb; ipdb.set_trace()
        if self.robot.base.force_drive(self.approach_speed, 0, 0, drive_time):
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

    def scan_to_base_link_points(self, scan):
        """
        Convert a sensor_msgs.msg.LaserScan message to a list of point in base_link frame
        Returns a numpy.ndarray of shape (N, 2)
        :param scan:
        :type scan LaserScan
        :return:
        """

        angles = np.arange(scan.angle_min, scan.angle_max-scan.angle_increment, scan.angle_increment)
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