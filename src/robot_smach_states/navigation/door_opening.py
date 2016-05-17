__author__ = 'amigo'

import smach

class PushDoorToOpen(smach.State):
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
        # Convert this to a list of cartesian points for easier calculations

        # Get footprint of the robot: topic /{robotname}/local_planner/local_costmap/robot_footprint/footprint_stamped
        # Convert this to /base_link coordinates

        # Find a distance (in base_link X +) when the footprint and the scan will intersect
        # Sample the X distances for the conversion of the footprint above or do this algebraically?

        # Drive this distance forwards, plus ca 10cm
        # Then, move sideways also for 10cm and back again to have the door fully open.

        # To determine the side, get the intersection point and the Y coordinate. If in y-, then right, if y+ then left.

        pass