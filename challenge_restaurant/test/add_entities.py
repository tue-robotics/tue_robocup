#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_object_spawner')

    W = client.SimWorld()

    # person at the door 
    person = W.add_object("loy-1", "loy", 1.5, 0, 0, 0, 0, 0.7)

    rospy.sleep(5)

    person.set_path([   [1.540, -0.021, 0.000],
                        [7.031, -0.024, 0.000],
                        [7.834, 2.987, 0.000],
                        [7.892, 5.444, 0.000],
                        [5.526, 7.008, 0.000],
                        [4.694, 5.917, 0.000],
                        [3.817, 5.120, 0.000],
                        [2.995, 4.969, 0.000],
                        [2.987, 2.681, 0.000],
                        [3.901, 1.687, 0.000],
                        [3.229, -0.079, 0.000],
                        [-3.084, -0.038, 0.000]], 0.2)  # Last number is path velocity in m/s
