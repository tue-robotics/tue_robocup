import rospy

namings = dict()

#TODO: These could all be shorter or even one, but we somehow we need the number of arguments. Or give every method *args, **kwargs
mapping = dict()
def register_key(key, label=None):
    """Call the action given to the decorator """
    def decorator(action):
        if key in namings:
            raise ValueError("key {0} already taken by {1}".format(key, namings[key]))

        mapping[key] = action
        if label: 
            namings[key] = label
        else:
            namings[key] = action.__name__
        return action
    return decorator

robot_mapping = dict()
def register_robot_key(key, label=None):
    """Call the action given to the decorator """
    def decorator(action):
        if key in namings:
            raise ValueError("key {0} already taken by {1}".format(key, namings[key]))

        robot_mapping[key] = action
        if label: 
            namings[key] = label
        else:
            namings[key] = action.__name__

        return action
    return decorator

robot_arm_mapping = dict()
def register_robot_arm_key(key, label=None):
    """Call the action given to the decorator """
    def decorator(action):
        if key in namings:
            raise ValueError("key {0} already taken by {1}".format(key, namings[key]))

        robot_arm_mapping[key] = action
        if label: 
            namings[key] = label
        else:
            namings[key] = action.__name__

        return action
    return decorator

def run_statemachine(sm_class):
    """Decorator for StateMachine classes.
    When applied to a StateMachine class (using the syntax: "
    >>> @run_statemachine
    >>> class BlaBla(smach.StateMachine):
    >>>     ...
    "
    it *returns a function* that will run the state machine with introspection.
    When also stacking the @register_*_key decorator on top, its very clean to add a new state machine to the demo.
    """
    def run_sm(robot):
        """Run a state machine with introspection"""
        sm = sm_class(robot)

        rospy.loginfo("State machine set up, start execution...")
        
        #import pdb; pdb.set_trace()
        import smach_ros
        introserver = smach_ros.IntrospectionServer(sm_class.__name__, sm, '/SM_ROOT_PRIMARY')
        introserver.start()

        result = sm.execute()
        
        introserver.stop()

        rospy.loginfo("State machine result: {0}".format(result))
    return run_sm

import smach
import robot_smach_states as states
@register_robot_key("=")
@run_statemachine
class SayHi(smach.StateMachine):
    """Scan (with the torso laser) for persons, 
    go to the closest one and say something nice."""

    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done', 'Aborted', 'Failed'])
        with self:
            smach.StateMachine.add( "SAY_HI",
                                    states.Say(robot, "Hi "),
                                    transitions={   "spoken":"Done"})