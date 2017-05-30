#!/usr/bin/env python
import rospy
from hmi_msgs.msg import QueryAction
from hmi import Client

from robot_part import RobotPart


class Api(RobotPart):
    def __init__(self, robot_name, tf_listener):
        super(Api, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        client = self.create_simple_action_client('/' + robot_name + '/hmi', QueryAction)
        self._client = Client(simple_action_client=client)


    def query(self, description, grammar, target, timeout=10):
        """
        Perform a HMI query, returns a HMIResult
        """
        return self._client.query(description, grammar, target, timeout)

    @property
    def last_talker_id(self):
        return self._client.last_talker_id

    def old_query(self, spec, choices, timeout=10):
        msg = 'robot.ears.recognize IS REMOVED. Use `robot.hmi.query`'
        rospy.logerr(msg)
        raise Exception(msg)
