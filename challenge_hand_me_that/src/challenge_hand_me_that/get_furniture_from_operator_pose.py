#
# Copyright (c) 2019, TU/e Robotics, Netherlands
# All rights reserved.
#
# \author Rein Appeldoorn

import os

import rospy
from ed.entity import Entity
from geometry_msgs.msg import PoseStamped
from robot_skills import get_robot, robot
from robot_smach_states.util.designators import is_writeable, VariableDesignator
from smach import StateMachine, cb_interface, CBState
from std_msgs.msg import Header
from ed_sensor_integration_msgs.srv import RayTraceResponse
from people_recognition_msgs.msg import Person3D


OPERATOR = None


class GetFurnitureFromOperatorPose(StateMachine):
    def __init__(self, robot, furniture_designator, possible_furniture):
        # type: (robot.Robot, VariableDesignator) -> None
        StateMachine.__init__(self, outcomes=['done'], output_keys=["laser_dot"])

        is_writeable(furniture_designator)

        def _show_view(timeout=5):
            rgb, depth, depth_info = robot.perception.get_rgb_depth_caminfo()
            robot.hmi.show_image_from_msg(rgb, timeout)
            return rgb, depth, depth_info

        @cb_interface(outcomes=['done'])
        def _prepare_operator(_):
            global OPERATOR
            OPERATOR = None

            robot.ed.reset()
            robot.head.reset()
            robot.speech.speak("Let's point, please stand in front of me!", block=False)
            for _ in range(10):
                _show_view(timeout=2)
                rospy.sleep(0.4)

            _show_view(timeout=2)
            robot.speech.speak("Please point at the object you want me to hand you", block=False)  # hmm, weird sentence
            rospy.sleep(0.4)
            for _ in range(2):
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
                if person.position.z > 2.5:
                    robot.speech.speak("Please stand in my view with your full body")
                    return False

                if person.position.z < 1.5:
                    robot.speech.speak("Please stand in my view with your full body")
                    return False

                if "is_pointing" not in person.tags:
                    robot.speech.speak("Please point with your arm stretched")
                    return False

                return True

            # # Shortcut people recognition; For testing
            # OPERATOR = Person3D()
            # OPERATOR.header.frame_id = "map"
            # OPERATOR.position.z = 5

            while not rospy.is_shutdown() and OPERATOR is None:
                persons = robot.perception.detect_person_3d(*_show_view())
                if persons:
                    persons = sorted(persons, key=lambda x: x.position.z)
                    person = persons[0]
                    if _is_operator(person):
                        OPERATOR = person

            # robot.speech.speak("I see an operator at %.2f meter in front of me" % OPERATOR.position.z)
            rospy.loginfo("I see an operator at %.2f meter in front of me" % OPERATOR.position.z)

            return 'done'

        @cb_interface(outcomes=['done', 'failed'], output_keys=['laser_dot'])
        def _get_furniture(user_data):
            global OPERATOR

            final_result = None
            while not rospy.is_shutdown() and final_result is None:
                result = None
                attempts = 0
                while not rospy.is_shutdown() and attempts < 5:
                    try:
                        attempts += 1
                        map_pose = robot.tf_buffer.transform(PoseStamped(
                            header=Header(
                                frame_id=OPERATOR.header.frame_id,
                                stamp=rospy.Time.now() - rospy.Duration.from_sec(0.5)
                            ),
                            pose=OPERATOR.pointing_pose
                        ), "map")
                        rospy.logwarn(f"{map_pose=}")
                        result = robot.ed.ray_trace(map_pose)
                        if result is not None:
                            break
                        # For testing
                        # result = RayTraceResponse()
                        # result.entity_id = "desk"
                        # result.intersection_point.header.frame_id = "map"
                        # result.intersection_point.point.x = 3.73
                        # result.intersection_point.point.y = 6.05
                        # result.intersection_point.point.z = 0.75
                    except Exception as e:
                        rospy.logerr("Could not get ray trace from closest person: {}".format(e))
                    rospy.sleep(0.5)
                else:
                    rospy.logerr("Could not find an entity with the current operator pose. Let him point again")
                    OPERATOR = None
                    return "failed"


                # result.intersection_point type: PointStamped
                # result.entity_id: string
                rospy.loginfo("There is a ray intersection with {i} at ({p.x:.4}, {p.y:.4}, {p.z:.4})".format(i=result.entity_id, p=result.intersection_point.point))

                if result.entity_id in possible_furniture:
                    final_result = result
                else:
                    rospy.loginfo("{} is not furniture".format(result.entity_id))
                    robot.speech.speak("That's not furniture, you dummy.")
                    rospy.sleep(1)
                    OPERATOR = None
                    robot.get_arm(required_goals=["reset"]).send_joint_goal("reset")
                    return 'failed'

            robot.speech.speak("You pointed at %s" % final_result.entity_id)
            robot.get_arm(required_goals=["reset"]).send_joint_goal("reset")
            # Fill the designator and user data the furniture inspection
            furniture_designator.write(robot.ed.get_entity(final_result.entity_id))
            user_data['laser_dot'] = result.intersection_point
            return 'done'

        with self:
            self.add('PREPARE_OPERATOR', CBState(_prepare_operator), transitions={'done': 'GET_OPERATOR'})
            self.add('GET_OPERATOR', CBState(_get_operator), transitions={'done': 'GET_FURNITURE'})
            self.add('GET_FURNITURE', CBState(_get_furniture), transitions={'done': 'done', 'failed': 'GET_OPERATOR'})


if __name__ == '__main__':
    from robocup_knowledge import load_knowledge
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    furniture_designator_ = VariableDesignator(resolve_type=Entity)
    robot_instance = get_robot("hero")
    robot_instance.reset()
    knowledge = load_knowledge("challenge_hand_me_that")
    while not rospy.is_shutdown():
        GetFurnitureFromOperatorPose(robot_instance, furniture_designator_.writeable, knowledge.all_possible_furniture).execute()
