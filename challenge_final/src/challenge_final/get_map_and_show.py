import cv2
from datetime import datetime
import os.path

import smach

import rospy

from PyKDL import Vector
from robot_smach_states.util.designators import check_type, is_writeable, VariableDesignator


class GetMap(smach.State):
    def __init__(self, robot, filename_des, entity_ids=None, mark_ids=None, plan_points=None):
        smach.State.__init__(self, outcomes=["created", "failed"])
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
            floorplan = robot.ed.get_map(entity_ids)
            if floorplan is not None:
                break
        else:
            rospy.logerr("Shit is broken")
            return "failed"

        mark_ids = self.mark_ids.resolve() if hasattr(self.mark_ids, "resolve") else self.mark_ids

        if mark_ids is not None:
            for id in mark_ids:
                e = self.robot.ed.get_entity(uuid=id)
                if e is None:
                    continue

                vs_image_frame = floorplan.map_pose.frame.Inverse() * e.pose.frame.p

                px = int(vs_image_frame.x() * floorplan.pixels_per_meter_width)
                py = int(vs_image_frame.y() * floorplan.pixels_per_meter_height)

                cv2.circle(floorplan.map, (px, py), 25, (0, 0, 255), -1)

        plan_points = self.plan_points.resolve() if hasattr(self.plan_points, "resolve") else self.plan_points

        if plan_points is not None:
            for point in plan_points:

                vs_image_frame = floorplan.map_pose.frame.Inverse() * point

                px = int(vs_image_frame.x() * floorplan.pixels_per_meter_width)
                py = int(vs_image_frame.y() * floorplan.pixels_per_meter_height)

                cv2.circle(floorplan.map, (px, py), 1, (255, 0, 0), -1)

        os.makedirs(os.path.expanduser('~/final'), exist_ok=True)
        filename = os.path.expanduser(
            '~/final/floorplan-{}.png'.format(datetime.now().strftime("%Y-%m-%d-%H-%M-%S")))
        cv2.imwrite(filename, floorplan.map)
        self.filename_des.write(filename)
        rospy.loginfo(f"Wrote image to {filename}")

        return "created"


class ShowImage(smach.State):
    def __init__(self, robot, filename_des, timeout=30):
        smach.State.__init__(self, outcomes=["shown", "failed"])

        self.robot = robot
        check_type(filename_des, str)
        self.filename_des = filename_des
        self.timeout = timeout

    def execute(self, ud=None):
        filename = self.filename_des.resolve()
        if filename is None:
            rospy.logerr(f"Could not resolve a filename from {self.filename_des}")
            return "failed"

        self.robot.hmi.show_image(filename, self.timeout)
        return "shown"


class GetMapAndShow(smach.StateMachine):
    def __init__(self, robot, entity_ids=None, mark_ids=None, plan_points=None, timeout=30):
        smach.StateMachine.__init__(self, outcomes=["done", "failed"])

        filename_des = VariableDesignator(resolve_type=str).writeable

        with self:
            smach.StateMachine.add("GET_MAP", GetMap(robot, filename_des, entity_ids, mark_ids, plan_points),
                                   transitions={"created": "SHOW_MAP",
                                                "failed": "failed"})

            smach.StateMachine.add("SHOW_MAP", ShowImage(robot, filename_des, timeout),
                                   transitions={"shown": "done",
                                                "failed": "failed"})


if __name__ == "__main__":
    import os.path
    from robot_skills import get_robot
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    robot = get_robot("hero")
    entities_des = VariableDesignator(['final'], resolve_type=[str])
    mark_ids_des = VariableDesignator(['victim', 'cupboard'], resolve_type=[str])
    plan_points_des = VariableDesignator(resolve_type=Vector)
    sm = GetMapAndShow(robot, entities_des, mark_ids_des)
    sm.execute()
