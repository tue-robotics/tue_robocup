# ROS
import PyKDL as kdl
import rospy
import math
from std_msgs.msg import Bool
# from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

# TU/e Robotics
from robot_skills.get_robot import get_robot


class Lethal_Zone:
    def __init__(self, robot):
        self.robot = robot
        self.front_bumper_sub = rospy.Subscriber("hero/base_f_bumper_sensor", Bool, self.front_callback)
        self.front_bumper_active = False
        self.back_bumper_sub = rospy.Subscriber("hero/base_b_bumper_sensor", Bool, self.back_callback)
        self.back_bumper_active = False
#        self._cmd_vel = rospy.Publisher("hero/base/references", Twist)
        self.global_costmap_sub = rospy.Subscriber("/hero/global_planner/global_costmap/costmap", OccupancyGrid, self.global_costmap_callback)
        self.costmap_info = None
        self.costmap_data = None

    def front_callback(self, msg):
#        rospy.loginfo(msg)
        self.front_bumper_active = msg.data

    def back_callback(self, msg):
#        rospy.loginfo(msg)
        self.back_bumper_active = msg.data

    def global_costmap_callback(self, msg):
#        rospy.loginfo(msg)
        self.costmap_info = msg.info
        self.costmap_data = msg.data

    def start_value(self, x, y):
        x_grid = (x - self.costmap_info.origin.position.x) / self.costmap_info.resolution
        y_grid = (y - self.costmap_info.origin.position.y) / self.costmap_info.resolution
        return x_grid, y_grid

    def free_space_finder(self, x, y):
        search_range = 0.24 + 0.2 - 0.5 * 0.05
        d_max_grid = (x+4)^2 + (y+4)^2
        x_free_grid = None
        y_free_grid = None
        for i in range(x-3, x+4):
            for j in range(y-3, y+4):
                if self.costmap_data[i, j] < 253:
                    d = (i-x) ^ 2 + (j-y) ^ 2
                    if d < d_max_grid:
                        d_max_grid = d
                        x_free_grid = i
                        y_free_grid = j
        return x_free_grid, y_free_grid, d_max_grid


    def motion(self):
        robot_frame = self.robot.base.get_location()
        x = robot_frame.frame.p.x()
        y = robot_frame.frame.p.y()
        # Get coordinates from HERO's current location
        rospy.loginfo("x: {}, y: {}".format(x, y))
        x_grid, y_grid = self.start_value(x, y)
        # Grid coordinates of HERO's start position
        rospy.loginfo("x_grid: {}, y_grid: {}".format(x_grid, y_grid))
        if self.costmap_data[x_grid, y_grid] == 253:

            x_free_grid, y_free_grid, d_max_grid = self.free_space_finder(x_grid, y_grid)
            # Get grid coordinates of the free space from the free_space_finder function
            if x_free_grid is not None:

                x_free = (x_free_grid * self.costmap_info.resolution) + self.costmap_info.origin.position.x
                y_free = (y_free_grid * self.costmap_info.resolution) + self.costmap_info.origin.position.y
                # Convert grid coordinates to regular coordinates

                d_max = (d_max_grid ^ 0.5) * self.costmap_info.resolution
                # Convert the value of d_max_grid to the actual distance to the free space in meters

                _, _, theta_H = robot_frame.frame.M.GetRPY()
                # Get rotation of HERO with respect to the world coordinate system

                vx = (math.cos(theta_H)*(x_free+x)+math.sin(theta_H)*(y_free+y))*0.1/d_max
                vy = (math.cos(theta_H)*(y_free+y)-math.sin(theta_H)*(x_free+x))*0.1/d_max
                vth = 0
                # Calculate the velocities in the x and y with respect to HERO's coordinate system

                duration = d_max / 0.1
                self.robot.base.force_drive(vx, vy, vth, duration)
                # Use force_drive to move HERO towards the free space
            else:

                rospy.loginfo("No free space found")
        else:

            rospy.loginfo("HERO already in free space")


if __name__ == "__main__":
    rospy.init_node("test_grasping")
    robot = get_robot("hero")
    bt = Lethal_Zone(robot)
    bt.motion()

