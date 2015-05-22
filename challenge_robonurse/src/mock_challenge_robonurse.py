#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy
import numpy as np
import random

from fast_simulator import client

def animate_path(name, _type, start, end, duration, steps=50, noise=0):
    xs = np.linspace(start[0], end[0], num=steps)
    ys = np.linspace(start[1], end[1], num=steps)
    zs = np.linspace(start[2], end[2], num=steps)

    if noise:
        xs += np.random.normal(0, noise, steps)  # median, stddev, samples
        ys += np.random.normal(0, noise, steps)  # median, stddev, samples
        zs += np.random.normal(0, noise, steps)  # median, stddev, samples
 
    for x,y,z in zip(xs, ys, zs):
        print x,y,z
        W.add_object(name, _type, x,y,z)
        rospy.sleep(float(duration)/steps)

if __name__ == "__main__":
    rospy.init_node('challenge_robonurse_object_spawner')

    W = client.SimWorld()

 
    W.add_object("bottle-1", "sim-coke", 3.05, 2.513, 1.095) 
    W.add_object("bottle-2", "apple_juice", 3.05, 2.70, 1.095)
    W.add_object("bottle-3", "tea_pack", 3.05, 2.913, 1.095)

    W.add_object("granny", "loy", 0.76, 1.08, 0.0)

    human_start = (0.76, 1.08, 0.0)
    human_end =   (0.76, -1.5, 0.05)
    animate_path("granny", "loy", human_start, human_end, 30, noise=0.05) 
    animate_path("granny", "loy", human_end, human_start, 30, noise=0.05) 