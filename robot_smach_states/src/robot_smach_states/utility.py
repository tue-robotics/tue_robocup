#! /usr/bin/env python
import rospy
import smach

''' For siren demo challenge '''
import thread
from visualization_msgs.msg import Marker

import robot_skills.util.msg_constructors as msgs
import std_msgs.msg
from ed.msg import EntityInfo
import robot_smach_states.util.designators as ds

from util.robocup_recorder import start_robocup_recorder


# ----------------------------------------------------------------------------------------------------

class Initialize(smach.State):
    def __init__(self, robot=None):
        start_robocup_recorder(robot.robot_name)
        smach.State.__init__(self, outcomes=['initialized',
                                             'abort'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.lights.set_color(0,0,1)  #be sure lights are blue

        self.robot.head.reset()
        self.robot.leftArm.reset()
        self.robot.leftArm.send_gripper_goal('close',0.0)
        self.robot.rightArm.reset()
        self.robot.rightArm.send_gripper_goal('close',0.0)
        self.robot.ed.reset()
        self.robot.torso.reset()

        ## Check if TF link between /map and /base_link is set, if not error at initialize in stead of during first navigate execution
        rospy.loginfo("TF link between /map and /base_link is checked. If it takes longer than a second, probably an error. Do a restart!!!")
        self.robot.base.get_location()

        return 'initialized'

# ----------------------------------------------------------------------------------------------------

class SetInitialPose(smach.State):
    ## To call upon this state:
    # example 1: Set_initial_pose(robot, "front_of_door"),
    # OR
    # Set_initial_pose(robot, [1,0,0])
    def __init__(self, robot, init_position):
        smach.State.__init__(self, outcomes=["done", "preempted", "error"])

        self.robot = robot
        self.preempted = False

        self.initial_position = init_position

    def location_2d(self, location):
        e_loc = self.robot.ed.get_entity(id=location)

        if not e_loc:
            rospy.logerr("SetInitialPose: ED entity '" + location + "' does not exist.")
            return []

        print e_loc

        try:
            rz = e_loc.data["pose"]["rz"]
        except KeyError:
            rz = 0

        return e_loc.pose.position.x, e_loc.pose.position.y, rz

    def execute(self, userdata):
        if isinstance(self.initial_position, str):
            x,y,phi = self.location_2d(self.initial_position)
        elif len(self.initial_position) == 3: #Tuple or list
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

# ----------------------------------------------------------------------------------------------------

class Trigger(smach.State):

    def __init__(self, robot, trigger, topic):
        smach.State.__init__(self,
                             outcomes=["triggered"])
        self.robot = robot
        self.trigger = trigger

        self.pub = rospy.Publisher(topic, std_msgs.msg.String, queue_size=10)

    def execute(self, userdata):
        self.pub.publish(std_msgs.String(data=trigger))
        return 'triggered'

# ----------------------------------------------------------------------------------------------------

class WaitForTrigger(smach.State):
    '''
    This state will block execution until a suitable trigger command is received on the channel /trigger
    It will receive std_msgs.String and will compare it to the strings in the array that is given.

    Example to wait for one of the strings 'allow' or 'deny' (could be sent from a gui):

        WaitForTrigger(robot, ['allow', 'deny'], "/trigger"),
                       transitions={    'allow':     'DO_SOMETHING',
                                        'deny':      'DO_SOMETHING',
                                        'preempted': 'failed'})
    '''

    def __init__(self, robot, triggers, topic, rate = 1.0):
        smach.State.__init__(self,
                             outcomes=triggers+['preempted'])
        self.robot = robot
        self.triggers = triggers

        # Get the ~private namespace parameters from command line or launch file.
        self.rate = rate
        topic     = topic

        rospy.Subscriber(topic, std_msgs.msg.String, self.callback)

        rospy.loginfo('topic: /%s', topic)
        rospy.loginfo('rate:  %d Hz', self.rate)

    def execute(self, userdata):
        self.trigger_received = False
        #rospy.logwarn("Waiting for trigger (any of {0}) on topic /trigger".format(self.triggers))
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

# ----------------------------------------------------------------------------------------------------

class WaitTime(smach.State):
    def __init__(self, robot=None, waittime=10):
        smach.State.__init__(self, outcomes=['waited','preempted'])
        self.robot = robot
        self.waittime = waittime

    def execute(self, *args, **kwargs):
        total_sleep = 0
        sleep_interval = 0.1
        #import ipdb; ipdb.set_trace()
        while total_sleep <= self.waittime:
            rospy.sleep(sleep_interval)
            total_sleep += sleep_interval
            if self.preempt_requested():
                rospy.loginfo('WaitTime preempted at {0} of {1}'.format(total_sleep, self.waittime))
                self.service_preempt()
                return 'preempted'
        return 'waited'

# ----------------------------------------------------------------------------------------------------

class WaitCondition(smach.State):
    '''Wait until a condition is satisfied, possible on a robot.
    When the condtion is satisfied, the value that matched the condition is stored in the userdata.
    The callback must return that value or something that evaluates to False otherwise.
    The arguments to the callback are userdata, robot'''
    def __init__(self, robot, condition_callback, timeout):
        ''' '''
        smach.State.__init__(self,
                             outcomes=['satisfied', 'timed_out', 'preempted'],
                             output_keys=['trigger_value'])
        self.condition_callback = condition_callback
        self.robot = robot
        self.timeout = timeout

    def execute(self, userdata):
        starttime = rospy.Time.now()

        while (rospy.Time.now() - starttime) < rospy.Duration(self.timeout)\
         and not rospy.is_shutdown():
            cb_output = self.condition_callback(userdata, self.robot)
            if cb_output:
                userdata.trigger_value = cb_output
                return "satisfied"
            if self.preempt_requested():
                self.service_preempt()
                return "preempted"
            rospy.sleep(0.1)
        return 'timed_out'

# ----------------------------------------------------------------------------------------------------

class Counter(smach.State):
    '''Smach state that counts the number of times it is executed. Returns 'counted' when it simply
    counted another value. Returns limit_reached when the counter designator resolves to a number
    that is greater than or equal to its set limit.'''
    def __init__(self, counter, limit):
        smach.State.__init__(self, outcomes=['counted', 'limit_reached'])
        self.limit = limit

        ds.check_resolve_type(counter, int)
        ds.is_writeable(counter)
        self.counter = counter

    def execute(self, userdata):
        count = self.counter.resolve()

        if count >= self.limit:
            self.counter.write(0)
            return 'limit_reached'
        else:
            self.counter.write(self.counter.resolve() + 1)
            return 'counted'

# ----------------------------------------------------------------------------------------------------

# class FinishOld(smach.State):
#     def __init__(self, robot=None):
#         smach.State.__init__(self,
#                                    outcomes=['stop'],input_keys=['challenge','start_time'])
#         self.robot = robot

#     def execute(self, gl):

#         duration = calculate_duration(gl.start_time)
#         print "Finished executing task", gl.challenge, "in", duration, "seconds."
#         return 'stop'

# ----------------------------------------------------------------------------------------------------

class PlaySound(smach.State):
    def __init__(self, filename, blocking=False):
        smach.State.__init__(self, outcomes=['played','error'])
        self.file = filename
        self.blocking = blocking

    def execute(self, ud):
        import os
        extension = os.path.splitext(self.file)[1]
        player = {".mp3":"mpg123", ".wav":"aplay"}[extension]
        command = (player, self.file)
        try:
            rospy.loginfo("Running '{0}'".format(*command))
            if self.blocking:
                os.system("{0} {1}".format(*command))
            else:
                thread.start_new_thread(os.system,("".join(command)))
            return "played"
        except Exception, e:
            rospy.logerr(e)
            return "error"

# ----------------------------------------------------------------------------------------------------

class SetTimeMarker(smach.State):
    def __init__(self, robot, designator):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        ds.is_writeable(designator)
        self.designator = designator

    def execute(self, userdata=None):
        self.designator.write(rospy.Time.now())
        return "done"

# ----------------------------------------------------------------------------------------------------

class CheckTime(smach.State):
    def __init__(self, robot, designator, max_duration):
        smach.State.__init__(self, outcomes=["ok", "timeout"])
        self.robot = robot
        self.max_duration = rospy.Duration(max_duration)

        ds.check_resolve_type(designator, int)
        self.designator = designator

    def execute(self, userdata=None):
        if rospy.Time.now() - self.designator.resolve() > self.max_duration:
            return "timeout"
        else:
            return "ok"

# ----------------------------------------------------------------------------------------------------

class LookAtHand(smach.State):
    def __init__(self, robot, side, keep_tracking=False, timeout=0.0):
        smach.State.__init__(self, outcomes=["done"])
        self.robot = robot
        self.timeout = timeout
        self.side = side
        self.keep_tracking = keep_tracking

    def execute(self, userdata=None):
        side_string = {self.robot.leftArm:"left", self.robot.rightArm:"right"}[self.side]
        self.robot.head.look_at_hand(side_string, keep_tracking=self.keep_tracking) #TODO: Unify side as string or/and object
        return "done"

# ----------------------------------------------------------------------------------------------------

class WaitForDesignator(smach.State):
    '''
        Waits for a given designator to answer. It will retry to resolve the
            designator a given number of times, with given sleep intervals (in seconds)
    '''

    def __init__(self, robot, designator, attempts = 1, sleep_interval = 1, outcomes=['success','failed']):
        smach.State.__init__(self, outcomes=["success", "failed"])
        self.robot = robot
        self.designator = designator
        self.attempts = attempts
        self.sleep_interval = sleep_interval

    def execute(self, userdata):
        counter = 0

        while counter < self.attempts:
            print "WaitForDesignator: waiting {0}/{1}".format(counter, self.attempts)

            result = self.designator.resolve()
            if result:
                return "success"

            counter += 1
            rospy.sleep(self.sleep_interval)

        return "failed"

# ----------------------------------------------------------------------------------------------------

class CallFunction(smach.State):
    """Call a (lambda) function with the given arguments

    >>> def p2(robot, val1, val2): print robot, val1, val2
    >>> l = CallFunction("amigo_dummy_str", p2, "hello", "world")
    >>> l.execute()
    amigo_dummy_str hello world
    'succeeded'

    >>> def faal(robot, val1, val2): raise Exception("Fail on purpose")
    >>> l2 = CallFunction("amigo_dummy_str", faal, "hello", "world")
    >>> l2.execute()
    ...
    'failed'
    """
    def __init__(self, robot, function, *args, **kwargs):
        smach.State.__init__(self, outcomes=["succeeded", "failed"])
        self.robot = robot
        self.function = function
        self.args = args
        self.kwargs = kwargs

    def execute(self, userdata=None):
        try:
            self.function(self.robot, *self.args, **self.kwargs)
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'failed'

# ----------------------------------------------------------------------------------------------------

class Evaluate(smach.State):
    '''Evaluates a dictionary of tasks and their results. 'results' is a designator containing a
    dictionary of task names to booleans which indicate whether this task succeeded or not. '''
    def __init__(self, results):
        smach.State.__init__(self, outcomes=['all_succeeded','partly_succeeded','all_failed'])
        ds.check_resolve_type(results, dict)
        self.results_designator = results

    def execute(self, userdata):
        results = self.results_designator.resolve()
        bools = [result for option, result in results.iteritems()]

        if True in bools and False in bools:
            return 'partly_succeeded'
        elif True in bools and not False in bools:
            return 'all_succeeded'
        elif not True in bools and False in bools:
            return 'all_failed'
        else:
            return 'all_failed'

# ----------------------------------------------------------------------------------------------------

class AddPositiveResult(smach.State):
    '''Smach state to add a positive result to the dictionary of results the results designator
    resolves to. Uses the string in item_designator as key to this dict. '''
    def __init__(self, results_designator, item_designator):
        smach.State.__init__(self, outcomes=['done'])

        ds.is_writeable(results_designator)
        ds.check_resolve_type(results_designator, dict)
        self.results = results_designator

        ds.check_resolve_type(item_designator, str)
        self.item = item_designator

    def execute(self, userdata):
        self.results.current[self.item.resolve()] = True
        return "done"

# ----------------------------------------------------------------------------------------------------

class AddNegativeResult(smach.State):
    '''Smach state to add a negative result to the dictionary of results the results designator
    resolves to. Uses the string in item_designator as key to this dict. '''
    def __init__(self, results_designator, item_designator):
        smach.State.__init__(self, outcomes=['done'])
        self.results = results_designator
        self.item = item_designator

    def execute(self, userdata):
        self.results.current[self.item.resolve()] = False
        return "done"

# ----------------------------------------------------------------------------------------------------

class LockDesignator(smach.State):
    def __init__(self, locking_designator):
        smach.State.__init__(self, outcomes=['locked'])
        self.locking_designator = locking_designator

    def execute(self, userdata=None):
        self.locking_designator.lock()
        rospy.loginfo("locking_designator {1} is now locked to {0}".format(str(self.locking_designator.resolve())[:10], self.locking_designator))
        return 'locked'

# ----------------------------------------------------------------------------------------------------

class UnlockDesignator(smach.State):
    def __init__(self, locking_designator):
        smach.State.__init__(self, outcomes=['unlocked'])
        self.locking_designator = locking_designator

    def execute(self, userdata=None):
        rospy.loginfo("locking_designator {1} is going to unlock from {0}".format(str(self.locking_designator.resolve())[:10], self.locking_designator))
        self.locking_designator.unlock()
        return 'unlocked'

# ----------------------------------------------------------------------------------------------------

class MarkEntityInRviz(smach.State):
    def __init__(self, entity_designator, namespace="designator"):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        ds.check_resolve_type(entity_designator, EntityInfo)
        self.entity_designator = entity_designator

        self.namespace = namespace
        self.publisher = rospy.Publisher("executive_markers", Marker)

    def execute(self, userdata=None):
        entity = self.entity_designator.resolve()
        if not entity:
            rospy.logerr("Cannot find entity for {0}".format(self.entity_designator))
            return 'failed'

        marker = Marker()
        #Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()

        #Set the namespace and id for this marker.  This serves to create a unique ID
        #Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = self.namespace
        marker.id = 0

        #Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = Marker.ARROW

        #Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = Marker.ADD

        #Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose = entity.pose

        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()

        self.publisher.publish(marker)
        return 'succeeded'

# ----------------------------------------------------------------------------------------------------

class IteratorState(smach.State):
    """
    >>> import robot_smach_states.util.designators as ds
    >>> iterable = ds.VariableDesignator(range(3), resolve_type=[int]) #Set up the collection we want to iterate over
    >>> i = ds.VariableDesignator(resolve_type=int) #iterable is a collection of integers (range(3))
    >>>
    >>> iterator_state = IteratorState(iterable.writeable, i.writeable)
    >>> iterable.resolve()
    [0, 1, 2]
    >>> iterator_state.execute()
    'next'
    >>> iterable.resolve()
    [1, 2]
    >>> i.resolve()
    0

    >>> iterator_state.execute() #Assigns the next value of iterable to i
    'next'
    >>> i.resolve()
    1
    >>>
    >>> iterator_state.execute() #Assigns the next value of iterable to i
    'next'
    >>> i.resolve() #Use the designator that now resolves to the (next) human.
    2
    >>>
    >>> iterator_state.execute() #If there are no more items in iterable, just return a different outcome
    'stop_iteration'
    """
    def __init__(self, iterable_designator, element_designator):
        smach.State.__init__(self, outcomes=['next', 'stop_iteration'])
        self.iterable_designator = iterable_designator

        ds.is_writeable(element_designator)
        self.element_designator = element_designator

    def execute(self, userdata=None):
        elements = self.iterable_designator.resolve()
        if elements:

            element = elements.pop(0)
            self.element_designator.write(element)
            self.iterable_designator.write(elements)
            rospy.loginfo("{0} iterates to next: {1}".format(self, str(element)[:20]))
            return "next"
        else:
            # self.element_designator.write(None)
            return "stop_iteration"

# ----------------------------------------------------------------------------------------------------

def test_iteration():
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
    numbers = ds.VariableDesignator(range(3), resolve_type=[int])
    number = ds.VariableDesignator(resolve_type=int)

    global gather
    gather = []

    with sm:
        smach.StateMachine.add( "STEP",
                                IteratorState(numbers.writeable, number.writeable),
                                transitions={   "next"          :"USE_NUMBER",
                                                "stop_iteration":"succeeded"})

        def lengthen_list(*args, **kwargs):
            resolved = number.resolve()
            print resolved
            global gather
            gather += [resolved]
        smach.StateMachine.add( "USE_NUMBER",
                                CallFunction("amigo", lengthen_list),
                                transitions={   "succeeded"     :"STEP",
                                                "failed"        :"failed"})

    sm.execute()
    print "gather = {0}".format(gather)
    print "numbers.resolve() = {0}".format(numbers.resolve())

    assert numbers.resolve() == []
    assert gather == [0, 1, 2]

if __name__ == "__main__":
    import doctest
    doctest.testmod()

    test_iteration()

# ----------------------------------------------------------------------------------------------------

class CancelHeadGoals(smach.State):
    """
        State wrapper for the function to cancels all head goals
    """
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done'])
        self.robot = robot

    def execute(self, robot):
        self.robot.head.cancel_goal()

        return 'done'
