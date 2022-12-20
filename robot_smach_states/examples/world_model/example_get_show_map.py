import os.path

from PyKDL import Vector
import rospy

from robot_skills import get_robot
from robot_smach_states.util.designators import VariableDesignator

from robot_smach_states.world_model.get_show_map import GetMapAndShow

if __name__ == "__main__":
    rospy.init_node(os.path.splitext("test_" + os.path.basename(__file__))[0])
    r = get_robot("hero")
    # Works for impuls environment
    entities_des = VariableDesignator(["dining_room"], resolve_type=[str])
    mark_ids_des = VariableDesignator(["initial_pose", "salon_table"], resolve_type=[str])
    plan_points_des = VariableDesignator(resolve_type=Vector)
    sm = GetMapAndShow(r, entities_des, mark_ids_des, plan_points_des)
    sm.execute()
