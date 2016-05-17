__author__ = 'amigo'

import smach

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

    def __init__(self, robot, door_entity_designator):
        self.robot = robot
        self.door_entity_designator = door_entity_designator

    def execute(self, userdata=None):
        # Align with the door frame; the X-axis should be perpendicular to the wall; y parallel to the wall

        # Get a laserscan message from topic /{robotname}/base_laser/scan
        # Convert this to a list of cartesian points in base_link for easier calculations
        #   (the assumption that the base_laser is in line with the base_link X-axis makes this easier, just addition)

        # Get footprint of the robot: topic /{robotname}/local_planner/local_costmap/robot_footprint/footprint_stamped
        # Convert this to /base_link coordinates

        # Find a distance (in base_link X +) when the footprint and the scan will intersect
        # Sample the X distances for the conversion of the footprint above or do this algebraically?

        # To determine the side, get the intersection point and the Y coordinate. If in y-, then right, if y+ then left.

        pass

    def scan_to_base_link_points(self, scan):
        pass

    def add_delta_to_points(self, points):
        pass

    def distance_between_footprint_and_scan(self, footprint_points, scan_points):
        """
        Find the distance between the two closest points in the two sets of points and return those points as well.
        :param footprint_points: Points (in base_link) that define the base's footprint
        :param scan_points: Points (in base_link) that are scanned by the front laser
        :return: tuple of (footprint_point, distance, scan_point)
        """
        pass

    def find_drive_distance_to_first_obstacle(self, footprint_points, scan_points):
        """
        Samples the distance with which to shift the footprint_points forwards (through a delta in X)
        and find the deltaX that makes for the smallest distance between these sets of points.

        This value is returned with the footprint_point at which this happens.

        :param footprint_points:
        :param scan_points:
        :return:
        """

class PushPerpendicularToDoor(smach.State):
    """
    Get the angle of the door via the laser or world model and
    force_drive to push with a force perpendicular to the door plane
    """
    pass