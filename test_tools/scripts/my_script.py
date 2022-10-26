#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class my_class:
    def __init__(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('ny_node', anonymous=True)

        self.laser_sub = rospy.Subscriber("laser", LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.loginfo("my_class init complete")

    def laser_callback(self, data):
        rospy.loginfo("got laser data")

    def odom_callback(self, data):
        rospy.loginfo("got odom data")

    def pub_velocity(self):
        rospy.loginfo("my_method")
        twist = Twist()
        twist.linear.x = 0.1
        self.pub.publish(twist)

    def run(self):
        rospy.loginfo("my_class run")
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    mything = my_class()
    mything.run()

