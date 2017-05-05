#!/usr/bin/python

# ROS
import smach


class StoreWaypoint(smach.State):
    """ Smach state to store the current location as a waypoint in ED. """
    def __init__(self, robot, waypoint_id):
        """ Constructor

        :param robot: robot object
        :param waypoint_id: string designating the id of the waypoint to store
        """
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot
        self._waypoint_id = waypoint_id

    def execute(self, userdata):
        """ Performs the actual work """

        self._robot.ed.update_entity(id=self._waypoint_id, frame_stamped=self._robot.base.get_location(),
                                     type="waypoint")

        return "done"
