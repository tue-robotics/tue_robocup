import rospy
import smach
from ed_robocup_msgs.srv import FitEntityInImage, FitEntityInImageRequest


class FitEntity(smach.State):
    """ Fits an entity """

    def __init__(self, robot, entity_str):
        """
        Constructor

        :param robot: robot object
        :param entity_str: string with the entity type to fit
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        self._robot = robot
        self._srv = rospy.ServiceProxy(robot.robot_name + '/ed/fit_entity_in_image', FitEntityInImage)
        self._entity_str = entity_str

    def execute(self, userdata=None):
        """ Executes the state """
        # Make sure the robot looks at the entity
        self._robot.head.reset()  # ToDo: this is abuse of the reset function
        self._robot.head.wait_for_motion_done(5.0)

        rospy.sleep(rospy.Duration(1.0))

        rospy.loginfo("Trying to fit...")

        # Try to fit the object
        req = FitEntityInImageRequest()
        req.entity_type = self._entity_str  # 1280 1024
        req.px = 0.5
        req.py = 0.5
        result = self._srv(req)

        # Cancel the head goal and return
        self._robot.head.cancel_goal()
        if result.error_msg:
            rospy.logerr("Fit entity: {0}".format(result))
            return 'failed'
        else:
            return 'succeeded'


