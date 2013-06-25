#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client

if __name__ == "__main__":
    rospy.init_node('cocktailparty_rgo2013_tester')

    W = client.SimWorld()

    person = W.add_object("person-1", "person", 4.35, -0.8, 0)
    person = W.add_object("person-2", "person", 5.19, -2.7, 0)   
    person = W.add_object("person-3", "person", 9.44, -2.8, 0)   
    person = W.add_object("person-4", "person", 10.07, -0.1, 0)   
    person = W.add_object("person-5", "person", 7.54, -0.7, 0)   

    coke = W.add_object("coke-1", "coke", 0.829, -5.958, 1.05)
    tea_pack = W.add_object("tea_pack-1", "tea_pack",0.848, -6.992, 1.05)     
