#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import itertools

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    models = ["coke", "milk", "fanta"]
    X = [3.05, 3.22]
    Y = [2.513, 2.70, 2.913]
    Z = [1.095]

    for index, (model, x,y,z) in enumerate(list(itertools.product(models, X, Y, Z))[:6]):
        W.add_object(model+"-"+str(index), model, x, y, z)
