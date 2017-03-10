import smach

class SegmentShelf(smach.State):
    """ Segments the entities on a specific shelf. This assumes that the robot is already looking in the right
    direction. """
    def __init__(self, robot, entity_id, area_id):
        """
        :param robot: robot object
        :param entity_id: string with the id of the entity
        :param area_id: string with the id of the area
        """
        smach.State.__init__(self, outcomes=['done'])

        self.robot = robot
        self._entity_id = entity_id
        self._area_id = area_id

    def execute(self, userdata=None):
        self.robot.ed.update_kinect("{} {}".format(self._area_id, self._entity_id))
        rospy.sleep(rospy.Duration(0.5))  # Is this necessary???

        return 'done'

