from __future__ import print_function

import PyKDL as kdl
import rospy
from ed.entity import Entity
from pykdl_ros import FrameStamped
from ed.shape import RightPrism
from ed.entity import PersonProperties
import robot_smach_states.util.designators as ds
from challenge_receptionist.introduce_guest import SayForIntroduceGuest
import sys
from robot_skills import get_robot

if __name__ == "__main__":

    if len(sys.argv) < 1:
        print("Please provide robot_name, room and seats_to_inspect as arguments. Eg. 'hero'")
        sys.exit(1)

    robot_name = sys.argv[1]

    rospy.init_node('test_find_emtpy_seat')
    robot = get_robot(robot_name)

    entity_id = "test_item"
    pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(0.0, 0.0, 0.0)),
                        stamp=rospy.Time.now(),
                        frame_id="map")
    person_properties = PersonProperties('Freek', 20, 'happy', 1.0, 0.9, 'left', 'standing', 0.9, ['orange', 'black'],
                                         '', '', 0, 'map')
    robot.ed.update_entity(uuid=entity_id, frame_stamped=pose)
    shape = RightPrism([kdl.Vector(0, 0, 0), kdl.Vector(0, 0.05, 0), kdl.Vector(0.05, 0.05, 0), kdl.Vector(0.05, 0, 0)],
                       -0.1, 0.1)
    item = Entity(entity_id, "person", pose.header.frame_id, pose.frame, shape, None, None, rospy.Time.now(),
                  person_properties)

    item = ds.VariableDesignator(item)

    sm = SayForIntroduceGuest(robot, item)

    sm.execute()

    rospy.loginfo("Guest is {}".format(item.resolve()))
