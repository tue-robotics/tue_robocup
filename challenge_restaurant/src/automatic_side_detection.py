#!/usr/bin/python

import rospy
import smach
import math
import numpy as np

from robot_smach_states.util.startup import startup
from geometry_msgs.msg import PointStamped

def _get_area(convex_hull):
    pts = [ [c.x, c.y] for c in convex_hull ]
    lines = np.hstack([pts,np.roll(pts,-1,axis=0)])
    area = 0.5*abs(sum(x1*y2-x2*y1 for x1,y1,x2,y2 in lines))
    return area

class WaitSay(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["done"])
        self._robot = robot

    def execute(self, userdata):
        self._robot.head.look_at_standing_person()
        answer = None
        while not answer or answer.result != "side detection":
            answer = self._robot.ears.recognize("side detection")
        self._robot.head.cancel_goal()

        return "done"

class AutomaticSideDetection(smach.State):
    def __init__(self, robot, background_padding=0.3, look_x = .2, look_y = 1.5, max_radius = 1.5, min_area = 0.3):
        self._sides = {
            "left": {
                "x": look_x,
                "y": look_y,
                "score": {},
                "entities": []
            },
            "right": {
                "x": look_x,
                "y": -look_y,
                "score": {},
                "entities": []
            },
        }
        smach.State.__init__(self, outcomes=self._sides.keys())
        self._robot = robot
        self._background_padding = background_padding
        self._max_radius = max_radius
        self._min_area = min_area

    def _get_head_goal(self, spec):
        goal = PointStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "/"+self._robot.robot_name+"/base_link"
        goal.point.x = spec["x"]
        goal.point.y = spec["y"]
        return goal

    def _inspect_sides(self):
        for side, spec in self._sides.iteritems():
            # Look at the side
            rospy.loginfo("looking at side %s" % side)
            goal = self._get_head_goal(spec)
            self._robot.head.look_at_point(goal)
            self._robot.head.wait_for_motion_done()
            rospy.sleep(0.2)

            base_position = self._robot.base.get_location().pose.position

            # Update kinect
            try:
                rospy.loginfo("Updating kinect for side %s" % side)
                kinect_update = self._robot.ed.update_kinect(background_padding=self._background_padding)
            except:
                rospy.logerr("Could not update_kinect")
                continue

            self._sides[side]["entities"] = [self._robot.ed.get_entity(id=id) for id in set(kinect_update.new_ids + kinect_update.updated_ids)]

            rospy.loginfo("Found %d entities for side %s" % (len(self._sides[side]["entities"]), side))

            # Filter subset
            self._sides[side]["entities"] = [ e for e in self._sides[side]["entities"] if self._subset_selection(base_position, e) ]

            # Score entities
            self._sides[side]["score"]["area_sum"] = sum([ self._score_area(e) for e in self._sides[side]["entities"] ])
            self._sides[side]["score"]["min_distance"] = self._score_closest_point(base_position, self._sides[side]["entities"])

            goal.point.z = 0.8
            self._robot.head.look_at_point(goal)
            self._robot.head.wait_for_motion_done()
            rospy.sleep(0.2)

            persons = self._robot.ed.detect_persons()
            self._sides[side]["score"]["face_found"] = False
            if persons:
                for person in persons:
                    p = person.pose.pose.position
                    if math.hypot(p.x - base_position.x, p.y - base_position.y) < self._max_radius:
                        self._sides[side]["score"]["face_found"] = True

    def _subset_selection(self, base_position, e):
        distance = math.hypot(e.pose.position.x - base_position.x, e.pose.position.y - base_position.y)
        return distance < self._max_radius

    def _score_area(self, e):
        return _get_area(e.convex_hull)

    def _score_closest_point(self, base_position, entities):
        distances = [ math.hypot(e.pose.position.x - base_position.x, e.pose.position.y - base_position.y) for e in entities ]
        if distances:
            min_distance = min(distances)
        else:
            min_distance = self._max_radius

        return (self._max_radius - min_distance) / self._max_radius

    def _get_best_side(self):

        best_side = None
        for side, spec in self._sides.iteritems():
            end_score = self._sides[side]["score"]["area_sum"] + self._sides[side]["score"]["min_distance"] + 1 * self._sides[side]["score"]["face_found"]
            rospy.loginfo("Side %s scoring (%f): %s" % (side, end_score, self._sides[side]["score"]))

            if best_side is None or self._sides[side]["score"] > self._sides[best_side]["score"]:
                best_side = side

        return best_side

    def execute(self, userdata):
        rospy.sleep(0.2)
        self._inspect_sides()

        best_side = self._get_best_side()

        self._robot.speech.speak(best_side)
        return best_side

def setup_statemachine(robot):

    sm = smach.StateMachine(outcomes=['done', 'aborted'])
    robot.ed.reset()

    with sm:
        smach.StateMachine.add('WAIT_SAY', WaitSay(robot), transitions={ 'done' :'AUTOMATIC_SIDE_DETECTION'})
        smach.StateMachine.add('AUTOMATIC_SIDE_DETECTION', AutomaticSideDetection(robot), transitions={ 'left' :'WAIT_SAY', 'right' : 'WAIT_SAY'})

    return sm

if __name__ == '__main__':
    rospy.init_node('automatic_side_detection')

    startup(setup_statemachine, challenge_name="automatic_side_detection")
