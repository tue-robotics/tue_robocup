import cv2
from datetime import datetime
import os.path

from PyKDL import Vector
import rospy
import smach

from robot_smach_states.human_interaction.show_image import ShowImage
from robot_smach_states.util.designators import check_type, is_writeable, VariableDesignator


class GetMap(smach.State):
    def __init__(
        self,
        robot,
        filename_des,
        entity_ids=None,
        mark_ids=None,
        plan_points=None,
        background: str = "white",
        print_labels: bool = True,
        width: int = 0,
        height: int = 0,
    ):
        super().__init__(outcomes=["created", "failed"])
        self.robot = robot

        check_type(filename_des, str)
        is_writeable(filename_des)
        self.filename_des = filename_des

        check_type(entity_ids, [str], type(None))
        self.entity_ids = entity_ids
        check_type(mark_ids, [str], type(None))
        self.mark_ids = mark_ids
        check_type(plan_points, Vector, type(None))
        self.plan_points = plan_points

        check_type(background, str)
        self.background = background
        check_type(print_labels, bool)
        self.print_labels = print_labels
        check_type(width, int)
        self.width = width
        check_type(height, int)
        self.height = height

    def execute(self, ud=None):
        entity_ids = self.entity_ids.resolve() if hasattr(self.entity_ids, "resolve") else self.entity_ids
        background = self.background.resolve() if hasattr(self.background, "resolve") else self.background
        print_labels = self.print_labels.resolve() if hasattr(self.print_labels, "resolve") else self.print_labels
        width = self.width.resolve() if hasattr(self.width, "resolve") else self.width
        height = self.height.resolve() if hasattr(self.height, "resolve") else self.height
        if entity_ids is None:
            entity_ids = []
        for _ in range(3):
            floor_plan = self.robot.ed.get_map(
                entity_ids, background=background, print_labels=print_labels, width=width, height=height
            )
            if floor_plan is not None:
                break
        else:
            rospy.logerr("Could not generate a map image")
            return "failed"

        mark_ids = self.mark_ids.resolve() if hasattr(self.mark_ids, "resolve") else self.mark_ids

        if mark_ids is not None:
            for mark_id in mark_ids:
                e = self.robot.ed.get_entity(uuid=mark_id)
                if e is None:
                    rospy.logdebug(f"Could not get entity for uuid: '{mark_id}'")
                    continue

                vs_image_frame = floor_plan.map_pose.frame.Inverse() * e.pose.frame.p

                px = int(vs_image_frame.x() * floor_plan.pixels_per_meter_width)
                py = int(vs_image_frame.y() * floor_plan.pixels_per_meter_height)

                cv2.circle(floor_plan.map, (px, py), 25, (0, 0, 255), -1)

        plan_points = self.plan_points.resolve() if hasattr(self.plan_points, "resolve") else self.plan_points

        if plan_points is not None:
            for point in plan_points:
                vs_image_frame = floor_plan.map_pose.frame.Inverse() * point

                px = int(vs_image_frame.x() * floor_plan.pixels_per_meter_width)
                py = int(vs_image_frame.y() * floor_plan.pixels_per_meter_height)

                cv2.circle(floor_plan.map, (px, py), 1, (255, 0, 0), -1)

        os.makedirs(os.path.expanduser("~/ed/maps"), exist_ok=True)
        filename = os.path.expanduser(
            "~/ed/maps/floor_plan-{}.png".format(datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
        )
        cv2.imwrite(filename, floor_plan.map)
        self.filename_des.write(filename)
        rospy.loginfo(f"Wrote image to {filename}")

        return "created"


class GetMapAndShow(smach.StateMachine):
    def __init__(
        self,
        robot,
        entity_ids=None,
        mark_ids=None,
        plan_points=None,
        background: str = "white",
        print_labels: bool = True,
        width: int = 0,
        height: int = 0,
        duration=30,
    ):
        super().__init__(outcomes=["done", "failed"])

        filename_des = VariableDesignator(resolve_type=str)

        with self:
            smach.StateMachine.add(
                "GET_MAP",
                GetMap(
                    robot,
                    filename_des.writeable,
                    entity_ids,
                    mark_ids,
                    plan_points,
                    background,
                    print_labels,
                    width,
                    height,
                ),
                transitions={"created": "SHOW_MAP", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SHOW_MAP",
                ShowImage(robot, filename_des, duration),
                transitions={"succeeded": "done", "failed": "failed"},
            )
