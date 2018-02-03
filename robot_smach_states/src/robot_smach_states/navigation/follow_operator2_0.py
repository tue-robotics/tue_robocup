#!/usr/bin/env python

import smach
import rospy
import sys
import smach_ros
import math
import collections
from robot_smach_states.util.startup import startup
from robot_skills.util import kdl_conversions
import PyKDL as kdl


class LearnOperator(smach.State):
    def __init__(self, robot, operator_timeout = 20, ask_follow=True, learn_face=True, learn_person_timeout = 10.0):
        smach.State.__init__(self, outcomes=['follow', 'Failed'])
        self.robot = robot
        self.operator_timeout = operator_timeout
        self.ask_follow = ask_follow
        self.learn_face = learn_face
        self.learn_person_timeout = learn_person_timeout


    def execute(self, userdata=None):
        operator = None                                             # local vs global variables?!?!
        start_time = rospy.Time.now()
        self.robot.head.look_at_standing_person()
        while not operator:
            if self.preempt_requested():
                return 'Failed'

            if(rospy.Time.now() - start_time).to_sec() > self.operator_timeout:
                return 'Failed'

            operator = self.robot.ed.get_closest_laser_entity(
                radius=0.5,
                center_point=kdl_conversions.VectorStamped(x=1.0, y=0, z=1,
                                                           frame_id="/%s/base_link" % self.robot.robot_name))
            rospy.loginfo("Operator: {op}".format(op=operator))
            if not operator:
                self.robot.speech.speak("Please stand in front of me")
            else:
                if self._learn_face:
                    self.robot.speech.speak("Please look at me while I learn to recognize you.",
                                             block=True)
                    self.robot.head.look_at_standing_person()
                    learn_person_start_time = rospy.Time.now()
                    num_detections = 0
                    while num_detections < 5:
                        if self.robot.perception.learn_person(self._operator_name):
                            num_detections += 1
                        elif (rospy.Time.now() - learn_person_start_time).to_sec() > self.learn_person_timeout:
                            self.robot.speech.speak("Please stand in front of me and look at me")
                            operator = None
                            break
        return 'follow'


class Track(smach.State):  # Updates the breadcrumb path
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['track', 'no_track'],
                             input_keys=['buffer'])
        self.counter = 0

    def execute(self, userdata):
        if self.counter == 4:
            userdata.buffer.append(self.counter)
            print ("New breadcrumb added to buffer")
            self.counter = 0
            return 'no_track'
        self.counter += 1
        return 'track'


class FollowBread(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['follow_bread', 'no_follow_bread'],
                             input_keys=['buffer'])

    def execute(self, userdata):
        print list(userdata.buffer)
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

    with sm_top:
        smach.StateMachine.add('LEARN_OPERATOR', LearnOperator(robot),
                               transitions={'follow': 'CON_FOLLOW',
                                            'Failed': 'Failed'})

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

        sm_con.userdata.buffer = collections.deque([1])

        with sm_con:
            smach.Concurrence.add('FOLLOWBREAD', FollowBread(), remapping={'buffer': 'buffer'})

            smach.Concurrence.add('TRACK', Track(), remapping={'buffer': 'buffer'}
                                  )

        smach.StateMachine.add('CON_FOLLOW', sm_con,
                               transitions={'recover_operator': 'RECOVERY',
                                            'ask_finalize': 'ASK_FINALIZE',
                                            'keep_following': 'CON_FOLLOW'})

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


