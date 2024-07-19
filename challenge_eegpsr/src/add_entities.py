#! /usr/bin/env python

import itertools
import os

import rospy
from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    env = os.environ['ROBOT_ENV']

    if env == "robotics_testlabs":

        # Testlab B: Position of the shelf: 3.18; 2.713; 0.35
        models = ["coke", "milk", "fanta"]
        X = [3.10, 3.12]
        Y = [2.56, 2.71, 2.86]
        Z = [0.75]

        for index, (model, x,y,z) in enumerate(list(itertools.product(models, X, Y, Z))[:6]):
            W.add_object(model+"-"+str(index), model, x, y, z)

    if env == "rwc2015":

        # Right_bookcase
        W.add_object("coke11", "sim-coke", 1.15, -8.13, 1.10)
        W.add_object("coke12", "sim-coke", 1.15, -8.13, 0.75)
        W.add_object("coke13", "sim-coke", 1.15, -8.33, 0.75)
        W.add_object("coke14", "sim-coke", 1.15, -8.7,  0.75)

    if env == "reo2016":

        # Bookcase
        W.add_object("coke12", "sim-coke", 1.052, -4.960, 0.8)
        W.add_object("coke13", "sim-coke", 0.992, -4.740, 0.8)
        W.add_object("coke14", "sim-coke", 0.852, -4.520, 0.8)

        # Kitchen counter
        W.add_object("coke-1", "sim-coke",  4.918, 4.066, 1.1)
        W.add_object("coke-2", "sim-coke",  4.923, 2.274, 1.1)

        # TV stand
        W.add_object("coke-3", "sim-coke",  3.295, -1.491, 0.55)

        # Dinner table
        W.add_object("coke-4", "sim-coke",  3.104, -5.163, 0.85)

        # People
        W.add_object("person-1", "loy", 4.196,  3.842, 0.0, 0, 0, 1)
        W.add_object("person-2", "loy", 1.416, -5.296, 0.0, 0, 0, 1)
