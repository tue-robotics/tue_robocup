import os

import rospy
from geometry_msgs.msg import PoseStamped
from smach.state import CBState
from smach.state_machine import StateMachine
from smach.util import cb_interface
from std_msgs.msg import Header

from robot_skills import get_robot


class GetCupboardRay(StateMachine):
    def __init__(self, robot, timeout=30.):
        StateMachine.__init__(self, outcomes=['done', 'failed'])

        def _show_view():
            rgb, depth, depth_info = robot.perception.get_rgb_depth_caminfo()
            robot.hmi.show_image_from_msg(rgb, 5.)
            return rgb, depth, depth_info

        @cb_interface(outcomes=['done', 'failed'])
        def _get_cupboard_ray(_):
            def _is_operator(person):
                if "is_pointing" not in person.tags:
                    rospy.loginfo("Please point with your arm stretched")
                    return False

                return True

            robot.head.look_down()

            start = rospy.Time.now()
            while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < timeout:
                persons = robot.perception.detect_person_3d(*_show_view())
                if persons:
                    persons = sorted(persons, key=lambda x: x.position.z)
                    person = persons[0]
                    if _is_operator(person):
                        map_pose = robot.tf_buffer.transform(PoseStamped(
                            header=Header(
                                frame_id=person.header.frame_id,
                                stamp=rospy.Time.now() - rospy.Duration.from_sec(0.5)
                            ),
                            pose=person.pointing_pose
                        ), "map")
                        rospy.logwarn(f"{map_pose=}")
                        try:
                            result = robot.ed.ray_trace(map_pose)
                            if result is not None and result.entity_id == "cupboard":
                                return "done"
                        except:
                            pass
                rospy.sleep(0.2)

            return 'failed'

        with self:
            self.add('GET_CUPBOARD_RAY', CBState(_get_cupboard_ray), transitions={'done': 'done', 'failed': 'failed'})


class CallNeighbor(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])

        with self:
            self.add("GET_CUPBOARD_RAY", GetCupboardRay(robot), transitions={"done": "done", "failed": "preempted"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    CallNeighbor(robot_instance).execute()
