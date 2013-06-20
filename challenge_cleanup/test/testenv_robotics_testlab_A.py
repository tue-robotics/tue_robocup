#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_challenge_tester')

    W = client.SimWorld()

    coke1 = W.add_object("coke-1", "coke", 3.738, -2.134, 0.8)    
    coke2 = W.add_object("coke-2", "coke", 4.705, -1.605, 0.76)    
    coke3 = W.add_object("coke-3", "coke", 4.964, 2.021, 0.87)    
    coke4 = W.add_object("coke-4", "coke", 0.994, 0.985, 0.53)    
    coke5 = W.add_object("coke-5", "coke", 0.724, 2.118, 0.63)    
    coke6 = W.add_object("coke-6", "coke", 0.293, -1.117, 0.84)    
    coke7 = W.add_object("coke-7", "coke", 2.049, -1.064, 0.84)    
