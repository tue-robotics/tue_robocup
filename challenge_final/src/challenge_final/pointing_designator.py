import rospy
from robot_smach_states.util.designators.ed_designators import EdEntityDesignator


class PointingDesignator(EdEntityDesignator):
    """ Specific designator for challenge final """

    def __init__(self, robot):
        """ Constructor

        :param robot: robot object
        """
        super(PointingDesignator, self).__init__(robot)

    def set_id(self, identifier):
        """ Sets the internal id """
        self.id = identifier

    def resolve(self):
        """ Resolves
        :return Entity if found, None otherwise
        """
        # If ID not set, return None
        if not self.id:
            rospy.logerr("PointingDesignator: no ID set")
            return None

        # Else: return the entity
        return self.ed.get_entity(id=self.id)
