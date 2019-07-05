#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import os

import rospy
from geometry_msgs.msg import PoseStamped
from robot_skills import Hero
from robot_smach_states import Entity, VariableDesignator
from robot_smach_states.util.designators import is_writeable
from rospy import ServiceException
from smach import StateMachine, cb_interface, CBState
from std_msgs.msg import Header

OPERATOR = None


class GetFurnitureFromOperatorPose(StateMachine):
    def __init__(self, robot, furniture_designator):
        # type: (Hero, VariableDesignator) -> None
        StateMachine.__init__(self, outcomes=['done'])

        is_writeable(furniture_designator)

        def _show_view(timeout=5):
            rgb, depth, depth_info = robot.perception.get_rgb_depth_caminfo()
            robot.hmi.show_image_from_msg(rgb, timeout)
            return rgb, depth, depth_info

        @cb_interface(outcomes=['done'])
        def _prepare_operator(_):
            global OPERATOR
            OPERATOR = None

            robot.head.reset()
            robot.speech.speak("Let's point, please stand in front of me!", block=False)
            _show_view(timeout=2)
            rospy.sleep(0.4)
            _show_view(timeout=2)
            rospy.sleep(0.4)
            _show_view(timeout=2)
            robot.speech.speak("Please point at the object you want me to hand you", block=False)  # hmm, weird sentence
            rospy.sleep(0.4)
            _show_view(timeout=2)
            rospy.sleep(0.4)
            _show_view(timeout=2)
            rospy.sleep(0.4)

            _show_view(timeout=1)
            robot.speech.speak("Three")
            _show_view(timeout=1)
            robot.speech.speak("Two")
            _show_view(timeout=1)
            robot.speech.speak("One")

            return 'done'

        @cb_interface(outcomes=['done'])
        def _get_operator(_):
            global OPERATOR

            def _is_operator(person):
                if "is_pointing" not in person.tags:
                    return False

                if person.position.z > 2.5:
                    return False
                if person.position.z < 1.5:
                    robot.speech.speak("You are too close")
                    return False

                return True

            while not rospy.is_shutdown() and OPERATOR is None:
                persons = robot.perception.detect_person_3d(*_show_view())
                for person in persons:
                    if _is_operator(person):
                        OPERATOR = person
                        break

            robot.speech.speak("I see an operator at %.2f meter in front of me" % OPERATOR.position.z)

            return 'done'

        @cb_interface(outcomes=['done'])
        def _get_furniture(_):
            global OPERATOR

            final_result = None
            while not rospy.is_shutdown() and final_result is None:
                result = None
                while not rospy.is_shutdown() and result is None:
                    try:
                        map_pose = robot.tf_listener.transformPose("map", PoseStamped(
                            header=Header(
                                frame_id=OPERATOR.header.frame_id,
                                stamp=rospy.Time.now() - rospy.Duration.from_sec(0.5)
                            ),
                            pose=OPERATOR.pointing_pose
                        ))
                        result = robot.ed.ray_trace(map_pose)
                    except Exception as e:
                        rospy.logerr("Could not get ray trace from closest person: {}".format(e))
                    rospy.sleep(1.)

                # TODO: LOY
                # result.intersection_point type: PointStamped
                # result.entity_id: string

                if result.entity_id in ["cabinet", "etc", "even", "invullen", "aub"]:
                    final_result = result

            # TODO: LOY
            # fill the designator and user data for Janno

            return 'done'

        with self:
            self.add('PREPARE_OPERATOR', CBState(_prepare_operator), transitions={'done': 'GET_OPERATOR'})
            self.add('GET_OPERATOR', CBState(_get_operator), transitions={'done': 'GET_FURNITURE'})
            self.add('GET_FURNITURE', CBState(_get_furniture), transitions={'done': 'done'})


if __name__ == '__main__':
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    furniture_designator = VariableDesignator(resolve_type=Entity)
    hero = Hero()
    hero.reset()
    GetFurnitureFromOperatorPose(hero, furniture_designator.writeable).execute()

    print(furniture_designator.resolve())
