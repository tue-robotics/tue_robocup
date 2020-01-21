import rospy
import smach

class ForceDrive(smach.State):
    """ Force drives... """
    def __init__(self, robot, vx, vy, vth, duration):
        """
        Constructor

        :param robot: robot object
        :param vx: velocity in x-direction
        :param vy: velocity in y-direction
        :param vth: yaw-velocity
        :param duration: float indicating how long to drive
        """
        smach.State.__init__(self, outcomes=['done'])
        self._robot = robot
        self._vx = vx
        self._vy = vy
        self._vth = vth
        self._duration = duration

    def execute(self, userdata=None):
        """ Executes the state """
        self._robot.base.force_drive(self._vx, self._vy, self._vth, self._duration)
        return 'done'



class ForceRotate(smach.State):
    """ Force forth and back. If a timeout is exceeded, we won't do this anymore """

    def __init__(self, robot, vth, duration, timeout):
        """
        Constructor

        :param robot: robot object
        :param vth: yaw-velocity
        :param duration: float indicating how long to drive
        :param timeout: after this, timedout is returned
        """
        smach.State.__init__(self, outcomes=['done', 'timedout'])
        self._robot = robot
        self._vth = vth
        self._duration = duration
        self._timeout = timeout
        self._first_stamp = None

    def execute(self, userdata=None):
        """
        Executes the state

        :param userdata:
        :return: "done" if rotated back and forth, or "timedout" if this takes too long
        """
        if self._first_stamp is None:
            self._first_stamp = rospy.Time.now()

        if (rospy.Time.now() - self._first_stamp).to_sec() > self._timeout:
            rospy.loginfo("ForceRotate timed out {0}, timeout is {1}...".format((rospy.Time.now() - self._first_stamp).to_sec(), self._timeout))
            return 'timedout'

        self._robot.base.force_drive(0, 0, self._vth, self._duration)
        self._vth = -self._vth
        self._robot.base.force_drive(0, 0, self._vth, self._duration)
        return 'done'
