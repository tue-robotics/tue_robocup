import cv2
from datetime import datetime
import os.path

from PyKDL import Vector
import rospy
import smach

from robot_smach_states.human_interaction.show_image import ShowImage
from robot_smach_states.util.designators import check_type, is_writeable, VariableDesignator


class GetMap(smach.State):
    def __init__(self, robot, filename_des, entity_ids=None, mark_ids=None, plan_points=None):
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

    def execute(self, ud=None):
        entity_ids = self.entity_ids.resolve() if hasattr(self.entity_ids, "resolve") else self.entity_ids
        if entity_ids is None:
            entity_ids = []
        for _ in range(3):
            floor_plan = self.robot.ed.get_map(entity_ids)
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
    def __init__(self, robot, entity_ids=None, mark_ids=None, plan_points=None, duration=30):
        super().__init__(outcomes=["done", "failed"])

        filename_des = VariableDesignator(resolve_type=str).writeable

        with self:
            smach.StateMachine.add(
                "GET_MAP",
                GetMap(robot, filename_des, entity_ids, mark_ids, plan_points),
                transitions={"created": "SHOW_MAP", "failed": "failed"},
            )

            smach.StateMachine.add(
                "SHOW_MAP",
                ShowImage(robot, filename_des, duration),
                transitions={"succeeded": "done", "failed": "failed"},
            )


if __name__ == "__main__":
    from robot_skills import get_robot

    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    r = get_robot("hero")
    # Works for impuls environment
    entities_des = VariableDesignator(["dining_room"], resolve_type=[str])
    mark_ids_des = VariableDesignator(["initial_pose", "salon_table"], resolve_type=[str])
    plan_points_des = VariableDesignator(resolve_type=Vector)
    sm = GetMapAndShow(r, entities_des, mark_ids_des, plan_points_des)
    sm.execute()
