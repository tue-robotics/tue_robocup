#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import itertools

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    # Testlab B: Position of the shelf: 3.18; 2.713; 0.35
    models = ["coke", "milk", "fanta"]
    X = [3.10, 3.12]
    Y = [2.56, 2.71, 2.86]
    Z = [0.75]

    for index, (model, x,y,z) in enumerate(list(itertools.product(models, X, Y, Z))[:6]):
        W.add_object(model+"-"+str(index), model, x, y, z)
