#!/usr/bin/env python
import rospy
from robocup_knowledge import load_knowledge
from robot_smach_states.util.startup import startup
from robot_smach_states.util.designators import EdEntityByIdDesignator

from empty_shelf_designator import EmptyShelfDesignator

challenge_knowledge = load_knowledge('challenge_manipulation')

USE_SLAM = True  # Indicates whether or not to use SLAM for localization
if USE_SLAM:
    CABINET = challenge_knowledge.cabinet_slam
else:
    CABINET = challenge_knowledge.cabinet_amcl

PLACE_SHELF = challenge_knowledge.place_shelf

'''
def main(robot):
    place_position = LockingDesignator(
        EmptyShelfDesignator(robot, self.cabinet, name="placement", area=PLACE_SHELF), name="place_position")

    @smach.cb_interface(outcomes=['done'])
    def lock_pp(ud, x, y, z):
        place_position.lock()
        return 'success'

    @smach.cb_interface(outcomes=['done'])
    def unlock_pp(ud, x, y, z):
        place_position.unlock()
        return 'done'


    sm = smach.StateMachine(outcomes=['done'])
    with sm:
        smach.StateMachine.add('LOCK_PLACE', CBState(lock_pp),
                               transitions={'done': 'PLACE_ITEM'})

        smach.StateMachine.add("PLACE_ITEM",
                               Place(robot, current_item, place_position, arm_with_item_designator),
                               transitions={'done': 'RESET_HEAD_PLACE',
                                            'failed': 'RESET_HEAD_HUMAN'})

        smach.StateMachine.add('UNLOCK_PLACE', CBState(unlock_pp),
                               transitions={'done': 'done'})

        # Execute SMACH plan
        outcome = sm.execute()

if __name__ == '__main__':
    rospy.init_node('test_empty_shelf_designator')
    startup(main)
'''


def setup_statemachine(robot):
    cabinet = EdEntityByIdDesignator(robot, id="bookcase", name="pick_shelf")

    ds = EmptyShelfDesignator(robot, cabinet, name="placement", area=PLACE_SHELF)

    for i in range(5):
        ds.resolve()
        import ipdb;ipdb.set_trace()


if __name__ == '__main__':
    rospy.init_node('test_empty_shelf_designator')

    startup(setup_statemachine)
