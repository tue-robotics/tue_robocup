#!/usr/bin/env python

import smach
import rospy
import sys
import smach_ros
import math
import collections
from robot_smach_states.util.startup import startup


class LearnOperator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['follow'])

    def execute(self, userdata=None):
        num_detections = 0
        while num_detections < 2:   #  Should eventually be increased, 2 is too small
            var = raw_input("Did I successfully detect you? Yes/No: ")
            if var == "Yes":
                num_detections += 1
        print ("Detected operator successfully 2 times, start following...")

        # The operator should be added to the breadcrumb list here.
        return 'follow'


class Track(smach.State):  # Updates the breadcrumb path
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['track', 'no_track'],
                             input_keys=['track_buffer_in'],
                             output_keys=['track_buffer_out'])
        self.counter = 0
        # self.track_buffer_in = track_buffer_in

    def execute(self, userdata):
        if self.counter == 4:
            userdata.track_buffer_out = userdata.track_buffer_in.append(self.counter)
            print ("New breadcrumb added to buffer")
            # print userdata.track_buffer_out
            self.counter = 0
            return 'no_track'
        userdata.track_buffer_out.append(userdata.track_buffer_in)
        self.counter += 1
        print (".")
        return 'track'


class FollowBread(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['follow_bread', 'no_follow_bread'],
                             input_keys=['followbread_buffer_in'],
                             output_keys=['followbread_buffer_out']
                             )
        # self.counter = 0
        # self.navigation_data = [0]

    def execute(self, userdata):
        # if self.counter == 5:
        #     return 'no_follow_bread'
        # self.counter += 1

        if len(userdata.followbread_buffer_in) is not None:
            print userdata.followbread_buffer_in
            # userdata.followbread_buffer_in.popleft()
        # userdata.followbread_buffer_out = userdata.followbread_buffer_in

        # print userdata.followbread_buffer_out

        # print self.navigation_data
        # userdata.followbread_buffer_out = userdata.followbread_buffer_in
        return 'follow_bread'


class AskFinalize(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['follow', 'Done'])
        self.counter2 = 0

    def execute(self, userdata=None):
        var = raw_input("Am I done following you? Yes/No: ")
        if var == "No":
            return 'follow'
        print "Okidoki, we reached the final destination."
        return 'Done'


class Recovery(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Failed', 'follow'])

    def execute(self, userdata=None):
        var = raw_input("Did I find you again? Yes/No: ")
        if var == "Yes":
            return 'follow'
        print "Oooh noooo, I give up."
        return 'Failed'


def setup_statemachine(robot):
    sm_top = smach.StateMachine(outcomes=['Done', 'Aborted', 'Failed'])

    # sm_top.userdata.buffer = collections.deque()  #  Als buffer ook in Learn Operator moet komen

    with sm_top:

        smach.StateMachine.add('LEARN_OPERATOR', LearnOperator(),
                               transitions={'follow': 'CON_FOLLOW'})

        #  remapping={'learn_op_buffer_in': 'buffer', 'learn_op_buffer_out': 'buffer'}
        #  Buffer should be initialized in Learn Operator since the first breadcrumb is found here
        smach.StateMachine.add('ASK_FINALIZE', AskFinalize(),
                               transitions={'follow': 'CON_FOLLOW',
                                            'Done': 'Done'})
        smach.StateMachine.add('RECOVERY', Recovery(),
                               transitions={'Failed': 'Failed',
                                            'follow': 'CON_FOLLOW'})

        sm_con = smach.Concurrence(outcomes=['recover_operator', 'ask_finalize', 'keep_following'],
                                   default_outcome='keep_following',
                                   outcome_map={'ask_finalize': {'FOLLOWBREAD': 'no_follow_bread',
                                                                 'TRACK': 'track'},
                                                'recover_operator': {'FOLLOWBREAD': 'no_follow_bread',
                                                                     'TRACK': 'no_track'}})

        sm_con.userdata.buffer = collections.deque()

        with sm_con:
            smach.Concurrence.add('FOLLOWBREAD', FollowBread(), remapping={'followbread_buffer_in': 'buffer',
                                                                            'followbread_buffer_out': 'buffer'})

            smach.Concurrence.add('TRACK', Track()
                                   , remapping={'track_buffer_in': 'buffer',
                                                                'track_buffer_out': 'buffer'}
                                  )

        smach.StateMachine.add('CON_FOLLOW', sm_con,
                               transitions={'recover_operator': 'RECOVERY',
                                            'ask_finalize': 'ASK_FINALIZE',
                                            'keep_following': 'CON_FOLLOW'},
                               # remapping={#'track_buffer_in': 'buffer',
                               #            #'track_buffer_out': 'buffer',
                               #            'followbread_buffer_in': 'buffer',
                               #            'followbread_buffer_out': 'buffer'}
                               )

        return sm_top


if __name__ == "__main__":
    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "Please provide robot name as argument."
        exit(1)

    rospy.init_node('test_follow_operator')
    startup(setup_statemachine, robot_name=robot_name)


# import copy, threading

# Sample code Janno
# This class should rather be named Buffer instead of BreadCrumb: it is only used to pass data. The 'breadcrumb' is
# the list that is kept in the execute function of the Follow state (hence NOT a member of the class)
# class BreadCrumb(object):
#     def __init__(self):
#         self._breadcrumb = []
#         self._lock = threading.Lock()
#
#     def set_data(self, data):
#         # Dit moet waarschijnlijk iets van 'append' worden. Alleen zetten is niet voldoende: het kan zomaar zijn dat ik
#         # meerdere malen een punt toe voeg voordat ik weer uitlees
#         # Bij het 'appenden' kun je ook meteen een distance check doen.
#         with self._lock:
#             self._list = copy.deepcopy(data)
#
#             # Equivalent to
#             # self._lock.acquire()
#             # self._list = copy.deepcopy(data)
#             # self._lock.release()
#
#     def append(self):
#         # ToDo Josja
#         pass
#
#     def get_data(self):
#         with self._lock:
#             result = self._breadcrumb
#             self._breadcrumb = []
#         return result
#
#
# class Track(smach.State):
#     def __init__(self, robot, breadcrumb):
#         self.robot = robot
#         self._breadcrumb = breadcrumb
#
#         # Possible new way:
#         # self._tracking_sub = rospy.Subscriber("bla", bla_msgs.Blaat, self._tracking_callback)
#
#     def execute(self):
#         while True:
#             operator = self.robot.ed.get_person()
#             self._breadcrumb.append(operator)
#
#             # self._breadcrumb.append(self._new_data)
#             # self._new_data = []
#
#             # def _tracking_callback(self, msg):
#             #     self._new_data = msg....
#             #     self._breadcrumb.append(msg.data)
#
#
# class Follow(smach.State):
#     def __init__(self, robot, breadcrumb):
#         self._breadcrumb = breadcrumb
#
#         self._lookat_radius = 1.0  # Something like that
#
#         def execute(self):
#             breadcrumb = []  # Create an empty list at the top of our execute hook: we don't want any state remaining
#
#             while True:
#                 new_list
#                 self._breadcrumb.get_data()
#                 breadcrumb = breadcrumb + new_list
#
#                 # Do smart stuff with breadcrumb
#
#                 # Pitfall: breadcrumb does still contain data when entering this hook the second time. How do we solve this?
#
#                 # ToDo: remove points that have been visited by the robot
#
#
# class FollowMachine(smach.ConcurrentStateMachine):
#     breadcrumb = BreadCrumb()
#
#     with sm:
#         smach.StateMachine.add("track", Track(breadcrumb=breadcrumb))
#         smach.StateMachine.add("Follow", Follow(breadcrumb=breadcrumb))
#
#
#


### new idea::
# class BreadCrumb(object):
#     def __init__(self):
#         self._breadcrumb = collections.deque()
#
#     # def set_data(self, data):
#     #     # Dit moet waarschijnlijk iets van 'append' worden. Alleen zetten is niet voldoende: het kan zomaar zijn dat ik
#     #     # meerdere malen een punt toe voeg voordat ik weer uitlees
#     #     # Bij het 'appenden' kun je ook meteen een distance check doen.
#     #     with self._lock:
#     #         self._list = copy.deepcopy(data)
#     #
#     #         # Equivalent to
#     #         # self._lock.acquire()
#     #         # self._list = copy.deepcopy(data)
#     #         # self._lock.release()
#
#     def append(self, data):
#         # ToDo Josja
#         self._breadcrumb.append(data)
#
#
## note that all the buffervariables are popped and placed in a different variable which can be used for planning
#     def get_data(self):
#         result = []
#         while len(self._breadcrumb) > 0:
#             result.append(self._breadcrumb.pop())  # Pop or popleft?
#         return result


