# ROS
import rospy
import smach
import std_msgs.msg

# TU/e Robotics
import robot_smach_states.util.designators as ds
from util.robocup_recorder import start_robocup_recorder


class Initialize(smach.State):
    """
    Resets the robot (arms, torso, head, etc.) and checks if the tf listener works correctly
    """
    def __init__(self, robot):
        """ Initialization

        :param robot: (Robot)
        """
        start_robocup_recorder(robot.robot_name)
        smach.State.__init__(self, outcomes=['initialized', 'abort'])
        self.robot = robot

    def execute(self, userdata=None):
        self.robot.reset()

        # Check if TF link between /map and /base_link is set
        # If not error at initialize in stead of during first navigate execution
        rospy.loginfo("TF link between /map and /base_link is checked. "
                      "If it takes longer than a second, probably an error. Do a restart!!!")
        self.robot.base.get_location().frame

        return 'initialized'


class SetInitialPose(smach.State):
    """
    Sets the initial pose for correct localization
    """
    def __init__(self, robot, init_position):
        """
        Initialization

        :param robot: (Robot)
        :param init_position: (str) identifies the (waypoint) entity to be used as initial pose. For testing purposes,
            a tuple(float, float, float) representing x, y and yaw in map frame can be used.
        """
        smach.State.__init__(self, outcomes=["done", "preempted", "error"])

        self.robot = robot
        assert isinstance(init_position, str) or isinstance(init_position, tuple) and len(init_position) == 3, \
            "Use a string or a tuple of length 3 (x, y, yaw) to initialize the SetInitialPose"
        self.initial_position = init_position

    def location_2d(self, location):
        """
        Gets the 2D location from a string identifying an wm entity

        :param location: (str) identifies the entity
        :return: tuple(float, float, float) x, y, yaw in map frame
        :raises: Exception
        """
        location_entity = self.robot.ed.get_entity(id=location)

        if not location_entity:
            raise Exception("SetInitialPose: ED entity '" + location + "' does not exist.")

        rz, _, _ = location_entity.pose.frame.M.GetEulerZYX()

        return location_entity.pose.frame.p.x(), location_entity.pose.frame.p.y(), rz

    def execute(self, userdata=None):
        if isinstance(self.initial_position, str):
            x, y, phi = self.location_2d(self.initial_position)
        elif len(self.initial_position) == 3:  # Tuple or list
            x = self.initial_position[0]
            y = self.initial_position[1]
            phi = self.initial_position[2]
        else:
            rospy.logerr("Initial pose {0} could not be set".format(self.initial_position))
            return "error"

        rospy.loginfo('Set initial pose to {0}, {1}, {2}'.format(x, y, phi))

        self.robot.base.set_initial_pose(x, y, phi)

        # Reset costmap: costmap is obviously entirely off if the localization was wrong before giving the initial pose
        # self.robot.base.reset_costmap()
        # Wait 0.5 s just to be sure
        rospy.sleep(rospy.Duration(0.5))

        return "done"


class Trigger(smach.State):

    def __init__(self, robot, trigger, topic):
        smach.State.__init__(self,
                             outcomes=["triggered"])
        self.robot = robot
        self.trigger = trigger

        self.pub = rospy.Publisher(topic, std_msgs.msg.String, queue_size=10)

    def execute(self, userdata=None):
        self.pub.publish(std_msgs.String(data=self.trigger))
        return 'triggered'


class WaitForTriggerTimeout(smach.State):
    """
    Same as WaitForTrigger with timeout
    """

    def __init__(self, robot, timeout, triggers, topic):
        smach.State.__init__(self,
                             outcomes=triggers+['preempted', 'timeout'])
        self.timeout = timeout
        self.robot = robot
        self.triggers = triggers
        self.trigger_received = None

        # Get the ~private namespace parameters from command line or launch file.
        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

        rospy.loginfo('topic: %s', topic)

    def execute(self, userdata=None):
        if self.trigger_received is None:
            self.trigger_received = False

        rospy.sleep(self.timeout)

        if self.trigger_received:
            return self.trigger_received
        else:
            return 'timeout'

    def callback(self, data):
        # Simply print out values in our custom message.
        if data.data in self.triggers:
            rospy.loginfo('trigger received: %s', data.data)
            self.trigger_received = data.data
        else:
            rospy.logwarn('wrong trigger received: %s', data.data)


class WaitForTrigger(smach.State):
    """
    This state will block execution until a suitable trigger command is received on the channel /trigger
    It will receive std_msgs.String and will compare it to the strings in the array that is given.

    Example to wait for one of the strings 'allow' or 'deny' (could be sent from a gui):

        WaitForTrigger(robot, ['allow', 'deny'], "/trigger"),
                       transitions={    'allow':     'DO_SOMETHING',
                                        'deny':      'DO_SOMETHING',
                                        'preempted': 'failed'})
    """
    def __init__(self, robot, triggers, topic, rate=1.0):
        smach.State.__init__(self,
                             outcomes=triggers+['preempted'])
        self.robot = robot
        self.triggers = triggers
        self.trigger_received = False

        # Get the ~private namespace parameters from command line or launch file.
        self.rate = rate
        topic = topic

        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

        rospy.loginfo('topic: /%s', topic)
        rospy.loginfo('rate:  %d Hz', self.rate)

    def execute(self, userdata=None):
        self.trigger_received = False

        while not rospy.is_shutdown() and not self.trigger_received:
            rospy.sleep(1/self.rate)

        if self.trigger_received:
            return self.trigger_received
        else:
            return 'preempted'

    def callback(self, data):
        # Simply print out values in our custom message.
        if data.data in self.triggers:
            rospy.loginfo('trigger received: %s', data.data)
            self.trigger_received = data.data
        else:
            rospy.logwarn('wrong trigger received: %s', data.data)


class WaitTime(smach.State):
    def __init__(self, robot=None, waittime=10):
        smach.State.__init__(self, outcomes=['waited','preempted'])
        self.robot = robot
        self.waittime = waittime

    def execute(self, *args, **kwargs):
        total_sleep = 0
        sleep_interval = 0.1

        while total_sleep <= self.waittime:
            rospy.sleep(sleep_interval)
            total_sleep += sleep_interval
            if self.preempt_requested():
                rospy.loginfo('WaitTime preempted at {0} of {1}'.format(total_sleep, self.waittime))
                self.service_preempt()
                return 'preempted'
        return 'waited'


class WaitCondition(smach.State):
    """
    Wait until a condition is satisfied, possible on a robot.
    When the condtion is satisfied, the value that matched the condition is stored in the userdata.
    The callback must return that value or something that evaluates to False otherwise.
    The arguments to the callback are userdata, robot
    """
    def __init__(self, robot, condition_callback, timeout):
        smach.State.__init__(self,
                             outcomes=['satisfied', 'timed_out', 'preempted'],
                             output_keys=['trigger_value'])
        self.condition_callback = condition_callback
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata):
        starttime = rospy.Time.now()

        while (rospy.Time.now() - starttime) < rospy.Duration(self.timeout) and not rospy.is_shutdown():
            cb_output = self.condition_callback(userdata, self.robot)
            if cb_output:
                userdata.trigger_value = cb_output
                return "satisfied"
            if self.preempt_requested():
                self.service_preempt()
                return "preempted"
            rospy.sleep(0.1)
        return 'timed_out'


class SetTimeMarker(smach.State):
    def __init__(self, robot, designator):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        ds.is_writeable(designator)
        self.designator = designator

    def execute(self, userdata=None):
        self.designator.write(rospy.Time.now())
        return "done"


class WaitForDesignator(smach.State):
    """
    Waits for a given designator to answer. It will retry to resolve the designator a given number of times, with
    given sleep intervals (in seconds)
    """
    def __init__(self, robot, designator, attempts = 1, sleep_interval = 1, outcomes=['success','failed']):
        smach.State.__init__(self, outcomes=["success", "failed"])
        self.robot = robot
        self.designator = designator
        self.attempts = attempts
        self.sleep_interval = sleep_interval

    def execute(self, userdata=None):
        counter = 0

        while counter < self.attempts:
            rospy.loginfo("WaitForDesignator: waiting {0}/{1}".format(counter, self.attempts))

            result = self.designator.resolve()
            if result:
                return "success"

            counter += 1
            rospy.sleep(self.sleep_interval)

        return "failed"


class LockDesignator(smach.State):
    def __init__(self, locking_designator):
        smach.State.__init__(self, outcomes=['locked'])
        self.locking_designator = locking_designator

    def execute(self, userdata=None):
        self.locking_designator.lock()
        rospy.loginfo("locking_designator {1} is now locked to {0}".format(
            str(self.locking_designator.resolve())[:10], self.locking_designator))
        return 'locked'


class UnlockDesignator(smach.State):
    def __init__(self, locking_designator):
        smach.State.__init__(self, outcomes=['unlocked'])
        self.locking_designator = locking_designator

    def execute(self, userdata=None):
        rospy.loginfo("locking_designator {1} is going to unlock from {0}".format(
            str(self.locking_designator.resolve())[:10], self.locking_designator))
        self.locking_designator.unlock()
        return 'unlocked'


class CheckBool(smach.State):
    """
    Provide a different transition based on a boolean designator.
    """
    def __init__(self, check_designator):
        """
        :param check_designator: designator resolving to True or False
        """
        super(CheckBool, self).__init__(outcomes=["true", "false"])
        ds.check_type(check_designator, bool)
        self._check_designator = check_designator

    def execute(self, userdata=None):
        val = self._check_designator.resolve() if hasattr(self._check_designator, "resolve") else self._check_designator
        if val:
            return "true"
        else:
            return "false"


class ToggleBool(smach.State):
    """
    Toggle a boolean designator from True to False and from False to True
    """
    def __init__(self, check_designator):
        """
        :param check_designator: boolean designator to be toggled
        """
        super(ToggleBool, self).__init__(outcomes=["done"])
        ds.is_writeable(check_designator)
        ds.check_type(check_designator, bool)
        self._check_designator = check_designator

    def execute(self, userdata=None):
        val = self._check_designator.resolve()
        if val:
            self._check_designator.write(False)
        else:
            self._check_designator.write(True)

        return "done"


if __name__ == "__main__":
    import doctest
    doctest.testmod()
