import os

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from smach.state import State, CBState
from smach.state_machine import StateMachine
from smach.util import cb_interface
from std_msgs.msg import Header, String

from robot_skills import get_robot
from robot_smach_states.human_interaction import Say


class GetCupboardRay(StateMachine):
    def __init__(self, robot, timeout=30.0):
        StateMachine.__init__(self, outcomes=["done", "failed"])

        def _show_view():
            rgb, depth, depth_info = robot.perception.get_rgb_depth_caminfo()
            robot.hmi.show_image_from_msg(rgb, 5.0)
            return rgb, depth, depth_info

        @cb_interface(outcomes=["done", "failed"])
        def _get_cupboard_ray(_):
            def _is_operator(person):
                if "is_pointing" not in person.tags:
                    rospy.loginfo("Please point with your arm stretched")
                    return False

                return True

            start = rospy.Time.now()
            while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < timeout:
                persons = robot.perception.detect_person_3d(*_show_view())
                if persons:
                    persons = sorted(persons, key=lambda x: x.position.z)
                    person = persons[0]
                    if _is_operator(person):
                        map_pose = robot.tf_buffer.transform(
                            PoseStamped(
                                header=Header(
                                    frame_id=person.header.frame_id,
                                    stamp=rospy.Time.now() - rospy.Duration.from_sec(0.5),
                                ),
                                pose=person.pointing_pose,
                            ),
                            "map",
                        )
                        rospy.logwarn(f"{map_pose=}")
                        try:
                            result = robot.ed.ray_trace(map_pose)
                            if result is not None and result.entity_id == "cupboard":
                                return "done"
                            rospy.logwarn(f"Ray trace result: {result=}")
                        except:
                            rospy.logwarn("Ray trace failed")
                            pass
                rospy.sleep(0.2)

            return "failed"

        with self:
            self.add("GET_CUPBOARD_RAY", CBState(_get_cupboard_ray), transitions={"done": "done", "failed": "failed"})


class SendTelegram(State):
    def __init__(self, robot, neighbor="Lotte", victim="Arpit", victim_pronoun="He") -> None:
        super().__init__(outcomes=["done"])

        self.robot = robot
        self.neighbor = neighbor
        self.victim = victim
        self.victim_pronoun = victim_pronoun

        self._telegram_message_pub = rospy.Publisher(
            f"/{self.robot.robot_name}/message_from_ros", String, queue_size=10
        )
        self._telegram_image_pub = rospy.Publisher(f"/{self.robot.robot_name}/image_from_ros", Image, queue_size=10)

        rospy.sleep(2)

    def execute(self, userdata=None) -> str:
        img = self.robot.perception.get_image()
        img.header.frame_id = ""

        self._telegram_message_pub.publish(f"Hi {self.neighbor}")
        rospy.sleep(0.2)
        self._telegram_message_pub.publish(f"{self.victim} needs your help. {self.victim_pronoun} is injured")
        rospy.sleep(0.2)

        self._telegram_image_pub.publish(img)
        rospy.sleep(2)

        self._telegram_message_pub.publish(f"Could you please get here ASAP?")
        rospy.sleep(0.2)

        return "done"


class ReceiveTelegram(State):
    def __init__(self, robot, neighbor="Lotte", timeout=20):
        super().__init__(outcomes=["done", "timeout"])

        self.robot = robot
        self.neighbor = neighbor
        self.timeout = timeout

    def execute(self, ud=None):
        try:
            telegram_response = rospy.wait_for_message(
                f"/{self.robot.robot_name}/message_to_ros", String, timeout=self.timeout
            )
        except:
            return "timeout"

        self.robot.speech.speak(f"{self.neighbor} said, {telegram_response.data}", block=True)
        return "done"


class CallNeighbor(StateMachine):
    def __init__(self, robot):
        StateMachine.__init__(self, outcomes=["done", "preempted"])

        with self:
            @cb_interface(outcomes=["done"])
            def _look_down_and_say(_):
                robot.head.look_down()
                robot.head.wait_for_motion_done()
                robot.speech.speak("Tell me where the first aid kit is")
                return "done"

            self.add("LOOK_DOWN_AND_SAY", CBState(_look_down_and_say), transitions={"done": "GET_CUPBOARD_RAY"})
            self.add("GET_CUPBOARD_RAY", GetCupboardRay(robot), transitions={"done": "SAY", "failed": "preempted"})
            self.add("SAY", Say(robot, "I understand that the first aid kit is at the cupboard"),
                     transitions={"spoken": "done"})


if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot_instance = get_robot("hero")
    CallNeighbor(robot_instance).execute()
