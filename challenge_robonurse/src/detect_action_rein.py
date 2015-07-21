#!/usr/bin/python

import rospy
from tf import TransformListener
from robot_skills import world_model_ed
from geometry_msgs.msg import Point
import math

def _get_entity_from_roi_and_center_point(ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, closest_x, closest_y):
	entity = ed.get_closest_entity(center_point=Point(x=closest_x, y=closest_y))
	if not entity:
		return None
	if entity.pose.position.x > roi_x_start and entity.pose.position.x < roi_x_end and entity.pose.position.y > roi_y_start and entity.pose.position.y < roi_y_end:
		return entity
	return None 


def detect_action(ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, closest_x, closest_y, timeout, distance_threshold):
	print "Find the person in the room in ROI x: [%fx%f], y: [%fx%f] from closest (x,y) : (%f,%f)" % (roi_x_start, roi_x_end, roi_y_start, roi_y_end, closest_x, closest_y)

	total_start = rospy.Time.now()

	# Find an operator
	operator = None
	while not operator:
		if rospy.Time.now() - total_start > rospy.Duration(timeout):
			return "sit"

		tmp_operator = _get_entity_from_roi_and_center_point(ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, closest_x, closest_y)
		if tmp_operator:
			start = rospy.Time.now()

			# Check if ID is alive for more than 3 seconds
			while rospy.Time.now() - start < rospy.Duration(1):
				# check if id is the same
				check_operator = _get_entity_from_roi_and_center_point(ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, closest_x, closest_y)
				if not check_operator:
					operator = None
					break

				if check_operator.id != tmp_operator.id:
					operator = None
					break

				operator = tmp_operator

				rospy.sleep(0.2)

		rospy.sleep(0.2)

	# We found an operator
	print "We have found an operator: %s"%operator.id

	last_seen_operator = None

	# Continuously the operator
	while rospy.Time.now() - total_start < rospy.Duration(timeout):
		tmp_operator = ed.get_entity(id=operator.id)
		if tmp_operator:
			last_seen_operator = tmp_operator
		else:
			if last_seen_operator:
				tmp_operator = _get_entity_from_roi_and_center_point(ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, last_seen_operator.pose.position.x, last_seen_operator.pose.position.y)
				if tmp_operator:
					last_seen_operator = tmp_operator
			else:
				tmp_operator = _get_entity_from_roi_and_center_point(ed, roi_x_start, roi_x_end, roi_y_start, roi_y_end, operator.pose.position.x, operator.pose.position.y)
				if tmp_operator:
					last_seen_operator = tmp_operator

		rospy.sleep(0.2)

	# Check the result 
	if not last_seen_operator:
		return "fall"

	dr = math.hypot(last_seen_operator.pose.position.x - operator.pose.position.x, last_seen_operator.pose.position.x - operator.pose.position.y)
	dt = (last_seen_operator.last_update_time - operator.last_update_time).to_sec()

	print "dr: " + dr
	print "dt: " + dt

	# Compare last seen with original
	if dr > distance_threshold:
		return "walk"

	if dt > 0.8 * timeout:
		return "sit"

	return "fall"

if __name__ == "__main__":
	rospy.init_node("test_detection_action_rein")
	rospy.sleep(1.0)
	ed = world_model_ed.ED('sergio', TransformListener(), wait_service=False)
	print detect_action(ed, roi_x_start=1.59, roi_x_end=4.7, roi_y_start=-9.17, roi_y_end=-4, closest_x=2.9, closest_y=-7.4, timeout=10, distance_threshold=1.0)
