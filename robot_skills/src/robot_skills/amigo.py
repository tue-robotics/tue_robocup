#! /usr/bin/env python

import robot


class Amigo(robot.Robot):

    """docstring for Amigo"""
    def __init__(self, dontInclude=[], wait_services=False):
        super(Amigo, self).__init__(robot_name="amigo", wait_services=wait_services)
