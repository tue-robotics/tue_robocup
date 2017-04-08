#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient, GoalStatus
from dragonfly_speech_recognition.srv import GetSpeechResponse
from hmi_msgs.msg import QueryAction
from hmi_server.abstract_server import queryToROS, resultFromROS
from hmi_server.common import random_fold_spec

from robot_part import RobotPart


class TimeoutException(Exception):
    pass


def _truncate(data):
    return (data[:75] + '..') if len(data) > 75 else data


def _print_example(spec, choices):
    # Copy request
    spec = random_fold_spec(spec, choices)
    rospy.loginfo("Example: \x1b[1;44m'{}'\x1b[0m".format(spec.strip()))


def _print_answer(answer):
    rospy.loginfo("Robot heard \x1b[1;42m'{}'\x1b[0m {}".format(answer.raw_result, resultFromROS(answer)))


def _print_timeout():
    rospy.loginfo("Robot did not hear you \x1b[1;43m(timeout)\x1b[0m")


def _print_generic_failure():
    rospy.logerr("Robot did not hear you \x1b[1;37;41m(speech failed)\x1b[0m")


class Api(RobotPart):
    def __init__(self, robot_name, tf_listener):
        '''
        Wrap the actionlib interface with the API
        '''
        super(Api, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self._client = self.create_simple_action_client('/' + robot_name + '/hmi', QueryAction)
        self._feedback = False
        self.last_talker_id = ""

    def _send_query(self, description, spec, choices):
        goal = queryToROS(description, spec, choices)
        state = self._client.send_goal(goal, feedback_cb=self._feedback_callback)

    def _feedback_callback(self, feedback):
        rospy.loginfo("Received feedback")
        self._feedback = True

    def _wait_for_result_and_get(self, timeout=None):
        execute_timeout = rospy.Duration(timeout) if timeout else rospy.Duration(10)
        preempt_timeout = rospy.Duration(1)

        while not self._client.wait_for_result(execute_timeout):
            if not self._feedback:
                # preempt action
                rospy.logdebug("Canceling goal")
                self._client.cancel_goal()
                if self._client.wait_for_result(preempt_timeout):
                    rospy.loginfo("Preempt finished within specified preempt_timeout [%.2f]", preempt_timeout.to_sec());
                else:
                    rospy.logwarn("Preempt didn't finish specified preempt_timeout [%.2f]", preempt_timeout.to_sec());
                break
            else:
                self._feedback = False
                rospy.loginfo("I received feedback, let's wait another %.2f seconds" % execute_timeout.to_sec())

        state = self._client.get_state()
        if state != GoalStatus.SUCCEEDED:
            if state == GoalStatus.PREEMPTED:
                # Timeout
                _print_timeout()
                raise TimeoutException("Goal did not succeed within the time limit")
            else:
                _print_generic_failure()
                raise Exception("Goal did not succeed, it was: %s" % GoalStatus.to_string(state))

        return self._client.get_result()

    def query(self, description, spec, choices, timeout=10):
        '''
        Perform a HMI query, returns a dict of {choicename: value}
        '''
        rospy.loginfo('Question: %s, spec: %s', description, _truncate(spec))
        _print_example(spec, choices)

        self._send_query(description, spec, choices)
        answer = self._wait_for_result_and_get(timeout=timeout)

        self.last_talker_id = answer.talker_id  # Keep track of the last talker_id

        _print_answer(answer)
        return resultFromROS(answer)

    def query_raw(self, description, spec, timeout=10):
        '''
        Perform a HMI query without choices, returns a string
        '''
        rospy.loginfo('Question: %s, spec: %s', description, _truncate(spec))
        _print_example(spec, {})

        self._send_query(description, spec, {})
        answer = self._wait_for_result_and_get(timeout=timeout)

        self.last_talker_id = answer.talker_id  # Keep track of the last talker_id

        _print_answer(answer)
        return answer.raw_result

    def old_query(self, spec, choices, timeout=10):
        '''
        Convert old queryies to a HMI query
        '''
        rospy.loginfo('spec: %s', _truncate(spec))
        _print_example(spec, choices)

        self._send_query('', spec, choices)
        try:
            answer = self._wait_for_result_and_get(timeout=timeout)
        except TimeoutException:
            return GetSpeechResponse(result="")
        except:
            return None
        else:
            # so we've got an answer
            self.last_talker_id = answer.talker_id  # Keep track of the last talker_id
            _print_answer(answer)

            # convert it to the old message
            choices = resultFromROS(answer)
            result = GetSpeechResponse(result=answer.raw_result)
            result.choices = choices

            return result

    def set_description(self, description):
        pass

    def set_grammar(self, spec):
        pass

    def wait_for_grammar_set(self, spec):
        pass
