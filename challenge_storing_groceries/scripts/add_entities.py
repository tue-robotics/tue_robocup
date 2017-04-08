#!/usr/bin/python
import rospy
import itertools
import os

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

        for index, (model, x, y, z) in enumerate(list(itertools.product(models, X, Y, Z))[:6]):
            W.add_object(model+"-"+str(index), model, x, y, z)

        # Put coke, milk and fanta on the table
        X = [0.915, 0.926, 0.908]
        Y = [1.699, 1.954, 2.238]
        Z = [0.85]
        for index, (model, x, y, z) in enumerate(list(itertools.product(models, X, Y, Z))[:6]):
            W.add_object("grab-" + model + "-" + str(index), model, x, y, z)

    if env == "rwc2015":

        # Right_bookcase
        W.add_object("coke11", "sim-coke", 1.15, -8.13, 1.10)
        W.add_object("coke12", "sim-coke", 1.15, -8.13, 0.75)
        W.add_object("coke13", "sim-coke", 1.15, -8.33, 0.75)
        W.add_object("coke14", "sim-coke", 1.15, -8.7,  0.75)

    if env == "reo2016":

        # Bookcase
        W.add_object("coke11", "sim-coke", 0.992, -4.740, 0.15)
        W.add_object("coke12", "sim-coke", 1.052, -4.960, 0.8)
        W.add_object("coke13", "sim-coke", 0.992, -4.740, 0.8)
        W.add_object("coke14", "sim-coke", 0.852, -4.520, 0.8)

    if env == "rwc2016" or env == "rwc2016a" or env == "rwc2016b" or env == "rwc2016_common":

        # Bookcase
        W.add_object("coke11", "sim-coke", 9.16, 5.67, 0.8)
        W.add_object("coke12", "sim-coke", 9.41, 5.67, 0.8)
        W.add_object("coke13", "sim-coke", 9.63, 5.68, 0.8)

        # SERGIO force drive example
        W.add_object("testthingie", "sim-coke", 2, 0, 0)

