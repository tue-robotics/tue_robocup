from typing import Union

# ROS
import rospy
import smach
import std_msgs.msg

# TU/e Robotics
from .util.designators import (
    check_resolve_type,
    check_type,
    is_writeable,
    Designator,
    LockingDesignator,
    VariableDesignator,
    value_or_resolve,
)
from .util.robocup_recorder import start_robocup_recorder


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

        # Check if TF link between map and base_link is set
        # If not error at initialize in stead of during first navigate execution
        rospy.loginfo("TF link between map and base_link is checked. "
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
        location_entity = self.robot.ed.get_entity(uuid=location)

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
    When the condition is satisfied, the value that matched the condition is stored in the userdata.
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
        is_writeable(designator)
        self.designator = designator

    def execute(self, userdata=None):
        self.designator.write(rospy.Time.now())
        return "done"


class CheckTimeOut(smach.State):
    def __init__(self, time_out_seconds, reset_des):
        smach.State.__init__(self, outcomes=["not_yet", "time_out"])
        self.time_out_seconds = time_out_seconds
        self.reset_des = reset_des

        check_type(reset_des, bool)
        is_writeable(reset_des)
        self.start = None

    def execute(self, userdata=None):
        current_seconds = rospy.Time.now().to_sec()

        if self.reset_des.resolve():
            rospy.loginfo("Resetting timeout")
            self.start = None
            self.reset_des.write(False)

        if self.start is None:
            self.start = current_seconds

        dt = current_seconds - self.start

        if dt > self.time_out_seconds:
            rospy.loginfo("Timer reached timeout")
            return "time_out"

        return "not_yet"


class CheckTries(smach.State):
    """
    This state will check if the number of tries is below a certain number.

    >>> reset_des = VariableDesignator(False, resolve_type=bool, name="reset").writeable
    >>> check_tries = CheckTries(max_tries=3, reset_des=reset_des)
    >>> check_tries.execute()
    'not_yet'
    >>> check_tries.execute()
    'not_yet'
    >>> check_tries.execute()
    'max_tries'
    >>> check_tries.execute()
    'max_tries'
    >>> reset_des.write(True)
    >>> check_tries.execute()
    'not_yet'
    """
    def __init__(self, max_tries: Union[int, Designator], reset_des: Designator):
        smach.State.__init__(self, outcomes=["not_yet", "max_tries"])
        self.max_tries = max_tries
        self.reset_des = reset_des

        check_type(max_tries, int)
        check_resolve_type(reset_des, bool)
        is_writeable(reset_des)
        self._counter = 0

    def execute(self, userdata=None):
        if self.reset_des.resolve():
            rospy.loginfo("Resetting counter")
            self._counter = 0
            self.reset_des.write(False)

        self._counter += 1

        max_tries = value_or_resolve(self.max_tries)
        if self._counter >= max_tries:
            rospy.loginfo(f"Max number of tries ({max_tries}) reached")
            return "max_tries"

        return "not_yet"


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


class WriteDesignator(smach.State):
    def __init__(self, write_designator, value):
        """
        Writes a value to a designator each time this state is executed. The value to be written can both
        be the value or a designator. In the latter, the resolved value is written to the designator.

        :param write_designator: Writeable designator
        :param value: Value or designator, which resolves to the value, to be written to the designator
        """
        smach.State.__init__(self, outcomes=['written'])
        is_writeable(write_designator)
        self.write_designator = write_designator
        self.value = value

    def execute(self, ud=None):
        value = self.value.resolve() if hasattr(self.value, 'resolve') else self.value
        rospy.loginfo(f"Writing '{value}' to designator: {self.write_designator}")
        self.write_designator.write(value)
        return 'written'


class CheckBool(smach.State):
    """
    Provide a different transition based on a boolean designator.
    """
    def __init__(self, check_designator):
        """
        :param check_designator: designator resolving to True or False
        """
        super(CheckBool, self).__init__(outcomes=["true", "false"])
        check_type(check_designator, bool)
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
        is_writeable(check_designator)
        check_type(check_designator, bool)
        self._check_designator = check_designator

    def execute(self, userdata=None):
        val = self._check_designator.resolve()
        if val:
            self._check_designator.write(False)
        else:
            self._check_designator.write(True)

        return "done"


class ResolveArm(smach.State):
    def __init__(self, arm, state_machine):
        """ Resolves, if possible, an arm for a state machine taking into account all the arm requirements

        :param arm: lockable arm designator
        :param state_machine: used state machine
        """
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        if not isinstance(arm, LockingDesignator):
            raise TypeError("No locking arm designator is given, which is required")
        self.arm = arm
        self.state_machine = state_machine

    def execute(self, userdata=None):
        arm_requirements = collect_arm_requirements(self.state_machine)
        for k, v in arm_requirements.items():
            if k in self.arm.to_be_locked.arm_properties:
                for val in arm_requirements[k]:
                    if val not in arm_requirements[k]:
                        self.arm.to_be_locked.arm_properties[k] += val
            else:
                self.arm.to_be_locked.arm_properties[k] = v
        self.arm.lock()
        if self.arm.resolve() is None:
            rospy.logerr("Didn't find an arm")  # ToDo: improve error message
            return "failed"
        else:
            return "succeeded"


def check_arm_requirements(state_machine, robot):
    """
    Checks if the robot has an arm that meets the requirements of all children states of this state machine

    :param state_machine: The state machine for which the requirements have to be checked
    :param robot: Robot to use
    :return: Check whether an arm is available that satisfies the requirements of the state machine
    """
    arm_requirements = collect_arm_requirements(state_machine)
    try:
        assert robot.get_arm(**arm_requirements) is not None,\
            "None of the available arms meets all this state machine's requirements: {}".format(arm_requirements)
    except AssertionError as e:
        rospy.logerr("Getting arm requirements failed, arm requirements: {}".format(arm_requirements))
        raise


def collect_arm_requirements(state_machine):
    """
    Collects all requirements on the arm of this specific state machine

    :param state_machine: State (machine) for which the requirements need to be collected
    :return: All arm requirements of the state machine
    """
    def update_requirements(update_dict):
        """
        Checks the input state for arm requirements and updates the current arm requirements if necessary

        :param update_dict: New arm requirements which should be checked against the current arm requirements.
            Dict can only contain lists or sets as values
        :type update_dict: dict
        :return: current arm requirements, updated (if necessary) given the update
        """

        # Update scheme for 1 level dict of sets
        for k, v in update_dict.items():
            if not isinstance(v, list) and not isinstance(v, set):
                raise ValueError("Key: {}, has a value which isn't a list or set.  Value: {}".format(k, v))
            if k not in arm_requirements:
                arm_requirements[k] = set()

            arm_requirements[k].update(v)

    # Check arm requirements
    arm_requirements = {}

    if isinstance(state_machine, smach.State):
        if hasattr(state_machine, "REQUIRED_ARM_PROPERTIES"):
            update_requirements(state_machine.REQUIRED_ARM_PROPERTIES)

    if isinstance(state_machine, smach.StateMachine):
        for child_state in state_machine.get_children().values():

            # Check if the child_state is a state_machine (must be done before checking for arm properties!)
            if isinstance(child_state, smach.StateMachine):
                child_sm_arm_requirements = collect_arm_requirements(child_state)
                update_requirements(child_sm_arm_requirements)

            if hasattr(child_state, "REQUIRED_ARM_PROPERTIES"):
                update_requirements(child_state.REQUIRED_ARM_PROPERTIES)

    rospy.logdebug("These are the collected arm requirements:{}".format(arm_requirements))
    return arm_requirements


if __name__ == "__main__":
    import doctest
    doctest.testmod()
