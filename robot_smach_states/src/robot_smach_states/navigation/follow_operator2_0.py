#!/usr/bin/env python

import smach
import rospy
import sys
import smach_ros
import math
from robot_smach_states.util.startup import startup


class LearnOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_follow'])

    def execute(self, userdata=None):
        num_detections = 0
        while num_detections < 5:
            var = raw_input("Did I succesfully detect you? Yes/No: ")
            if var == "Yes":
                num_detections += 1
        print ("Detected operator successfully 5 times, start following...")
        # The operator should be added to the breadcrumb list here.
        return 'go_follow'


class Track(smach.State): # Updates the breadcrumb path
    def __init__(self):
        smach.State.__init__(self, outcomes=['ask_finalize', 'keep_following', 'recover_operator'])
        self.counter = 0

    def execute(self, userdata=None):
        if self.counter < 10:
            if self.counter == 5:
                var = raw_input("Did I lose you? Yes/No: ")
                if var == "No":
                    self.counter += 1
                    return 'keep_following'
                return 'recover_operator'
            self.counter += 1
            return 'keep_following'
        return 'ask_finalize'


class AskFinalize(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['go_follow', 'Done'])
        self.counter2 = 0

    def execute(self, userdata=None):
        var = raw_input("Am I done following you? Yes/No: ")
        if var == "No":
            return 'go_follow'
        print "Okidoki, we reached the final destination."
        return 'Done'


class Recovery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Failed', 'go_follow'])

    def execute(self, userdata=None):
        var = raw_input("Did I find you again? Yes/No: ")
        if var == "Yes":
            return 'go_follow'
        print "Oooh noooo, I give up."
        return 'Failed'

class FollowPath(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ask_finalize', 'keep_following'])

    def execute(self, userdata=None):



def setup_statemachine(robot):
    sm = smach.StateMachine(outcomes=['Done', 'Aborted', 'Failed'])

    with sm:

        smach.StateMachine.add('LEARN_OPERATOR', LearnOperator(),
                               transitions={'go_follow': 'TRACK'})
        smach.StateMachine.add('TRACK', Track(),
                               transitions={'ask_finalize': 'ASK_FINALIZE',
                                            'keep_following': 'TRACK',
                                            'recover_operator': 'RECOVERY'})
        smach.StateMachine.add('ASK_FINALIZE', AskFinalize(),
                               transitions={'go_follow': 'FOLLOW',
                                            'finalized': 'Done'})
        smach.StateMachine.add('RECOVERY', Recovery(),
                               transitions={'failed': 'Failed',
                                            'go_follow': 'FOLLOW'})
        return sm

if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "Please provide robot name as argument."
        exit(1)

    rospy.init_node('test_follow_operator')
    startup(setup_statemachine, robot_name=robot_name)

import copy, threading

## Sample code Janno
# This class should rather be named Buffer instead of BreadCrumb: it is only used to pass data. The 'breadcrumb' is
# the list that is kept in the execute function of the Follow state (hence NOT a member of the class)
class BreadCrumb(object):
    def __init__(self):
        self._breadcrumb = []
        self._lock = threading.Lock()

    def set_data(self, data):
        # Dit moet waarschijnlijk iets van 'append' worden. Alleen zetten is niet voldoende: het kan zomaar zijn dat ik
        # meerdere malen een punt toe voeg voordat ik weer uitlees
        # Bij het 'appenden' kun je ook meteen een distance check doen.
        with self._lock:
            self._list = copy.deepcopy(data)

            # Equivalent to
            # self._lock.acquire()
            # self._list = copy.deepcopy(data)
            # self._lock.release()

    def append(self):
        # ToDo Josja
        pass

    def get_data(self):
        with self._lock:
            result = self._breadcrumb
            self._breadcrumb = []
        return result


class Track(smach.State):
    def __init__(self, robot, breadcrumb):
        self.robot = robot
        self._breadcrumb = breadcrumb

        # Possible new way:
        # self._tracking_sub = rospy.Subscriber("bla", bla_msgs.Blaat, self._tracking_callback)

    def execute(self):
        while True:
            operator = self.robot.ed.get_person()
            self._breadcrumb.append(operator)

            # self._breadcrumb.append(self._new_data)
            # self._new_data = []

            # def _tracking_callback(self, msg):
            #     self._new_data = msg....
            #     self._breadcrumb.append(msg.data)


class Follow(smach.State):
    def __init__(self, robot, breadcrumb):
        self._breadcrumb = breadcrumb

        self._lookat_radius = 1.0  # Something like that

        def execute(self):
            breadcrumb = []  # Create an empty list at the top of our execute hook: we don't want any state remaining

            while True:
                new_list
                self._breadcrumb.get_data()
                breadcrumb = breadcrumb + new_list

                # Do smart stuff with breadcrumb

                # Pitfall: breadcrumb does still contain data when entering this hook the second time. How do we solve this?

                # ToDo: remove points that have been visited by the robot


class FollowMachine(smach.ConcurrentStateMachine):
    breadcrumb = BreadCrumb()

    with sm:
        smach.StateMachine.add("track", Track(breadcrumb=breadcrumb))
        smach.StateMachine.add("Follow", Follow(breadcrumb=breadcrumb))



