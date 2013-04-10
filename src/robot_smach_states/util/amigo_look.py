#!/usr/bin/env python
import roslib; roslib.load_manifest('tue_execution_pack')
import rospy
from geometry_msgs.msg import PointStamped
import sys

def talker(x,y, z=1, frame="/base"):
	pub = rospy.Publisher('head_target', PointStamped)
	rospy.init_node('head_target_sender')
	
	rospy.sleep(0.5);
	try:
		for y in range(-3, 3):
			head_goal = PointStamped()
			
			head_goal.header.stamp = rospy.get_rostime()
			head_goal.header.frame_id = frame
			head_goal.point.x = x
			head_goal.point.y = y
			head_goal.point.z = z
			
			rospy.sleep(1)
			
			rospy.loginfo(head_goal)
			pub.publish(head_goal)
			
		for y in range(-3, 3):
			y = -y
			head_goal = PointStamped()
			
			head_goal.header.stamp = rospy.get_rostime()
			head_goal.header.frame_id = frame
			head_goal.point.x = x
			head_goal.point.y = y
			head_goal.point.z = z
			
			rospy.sleep(1)
			
			rospy.loginfo(head_goal)
			pub.publish(head_goal)
	except KeyboardInterrupt:
		exit()

if __name__ == '__main__':
	
	if len(sys.argv) >= 3:
		x = float(sys.argv[1])
		y = float(sys.argv[2])
	else:
		x = 0
		y = 0
	
	try:
		talker(x,y)
	except rospy.ROSInterruptException: pass
