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
    person_properties = PersonProperties('John', 20, 'happy', 0.0, 0.9, 'left', 'standing', 0.9, ['orange', 'black'],
                                         '', '', 0, 'map')
    robot.ed.update_entity(uuid=entity_id, frame_stamped=pose)
    shape = RightPrism([kdl.Vector(0, 0, 0), kdl.Vector(0, 0.05, 0), kdl.Vector(0.05, 0.05, 0), kdl.Vector(0.05, 0, 0)],
                       -0.1, 0.1)
    item = Entity(entity_id, "person", pose.header.frame_id, pose.frame, shape, None, None, rospy.Time.now(),
                  person_properties)

    item_des = ds.VariableDesignator(item)
    from hmi import HMIResult
    blaat = HMIResult('coke', {'drink': 'coke'})
    guest_drinkname_des = ds.VariableDesignator(resolve_type=HMIResult, name='previous_guest_drink').writeable
    guest_drink_name_des = ds.FieldOfHMIResult(guest_drinkname_des, semantics_path=['drink'], name='previous_guest_drinkname')
    guest_drinkname_des.write(blaat)

    assume_john = True
    previous_guest_drink_des = ds.VariableDesignator(resolve_type=HMIResult, name='previous_guest_drink')
    previous_guest_drinkname_des = ds.FieldOfHMIResult(previous_guest_drink_des, semantics_path=['drink'],
                                                       name='previous_guest_drinkname')

    sm = SayForIntroduceGuest(robot, item_des,guest_drinkname_des, assume_john, previous_guest_drink_des.writeable,
                              previous_guest_drinkname_des)

    sm.execute()

    assume_john = False
    entity_id2 = "test_item2"
    person_properties2 = PersonProperties('Anna', 20, 'happy', 1.0, 0.9, 'left', 'standing', 0.9, ['orange', 'black'],
                                         '', '', 0, 'map')
    item2 = Entity(entity_id2, "person", pose.header.frame_id, pose.frame, shape, None, None, rospy.Time.now(),
                  person_properties2)
    item2_des = ds.VariableDesignator(item2)
    sm2 = SayForIntroduceGuest(robot, item2_des, guest_drinkname_des, assume_john, previous_guest_drink_des.writeable,
                              previous_guest_drinkname_des)

    sm2.execute()

    rospy.loginfo("Guest is {}".format(item.resolve()))
