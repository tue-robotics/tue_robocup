#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('fast_simulator_challenge_tester')

    W = client.SimWorld()

    # couch table
    coke1 = W.add_object("coke-1", "coke", 8.664, -1.166, 0.5)    

    # large table
    coke2 = W.add_object("coke-2", "coke", 5.703, -0.932, 0.8)    
    coke3 = W.add_object("coke-3", "coke", 5.688, -2.471, 0.8)  

    # bed_cabinet_1
    coke4 = W.add_object("coke-4", "coke", 10.229, -5.404, 0.5)    

    # bed_cabinet_2
    coke5 = W.add_object("coke-5", "coke", 10.219, -7.630, 0.5)    

    # kitchen table
    coke6 = W.add_object("coke-6", "coke", 3.514, -6.331, 0.8)    
    coke7 = W.add_object("coke-7", "coke", 2.619, -6.179, 0.8)    

    # bedroom cabinet
    coke4 = W.add_object("coke-8", "coke", 6.155, -6.470, 0.5)    
    coke5 = W.add_object("coke-9", "coke", 6.039, -7.215, 0.5)
