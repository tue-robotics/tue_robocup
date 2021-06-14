# ROS
import PyKDL as kdl
import rospy
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
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
        self._cmd_vel = rospy.Publisher("hero/base/references", Twist)
        self.global_costmap_sub = rospy.Subscriber("/hero/global_planner/global_costmap/costmap", OccupancyGrid, self.global_costmap_callback)
        self.costmap_info = None
        self.costmap_data = None

    def front_callback(self, msg):
        rospy.loginfo(msg)
        self.front_bumper_active = msg.data

    def back_callback(self, msg):
        rospy.loginfo(msg)
        self.back_bumper_active = msg.data

    def global_costmap_callback(self, msg):
        rospy.loginfo(msg)
        self.costmap_info = msg.info
        self.costmap_data = msg.data

    def start_value(self, x, y):
        x = (x - self.costmap_info.origin.position.x) / self.costmap_info.resolution
        y = (y - self.costmap_info.origin.position.y) / self.costmap_info.resolution
        start = self.costmap_data[x, y]
        return start

    def free_space_finder(self, x, y):
        d_max = (x+4)^2 + (y+4)^2
        x_free = None
        y_free = None
        for i in range(x-3, x+4):
            for j in range(y-3, y+4):
                if self.costmap_data[i, j] < 253:
                    d = (i-x) ^ 2 + (j-y) ^ 2
                    if d < d_max:
                        d_max = d
                        x_free = i
                        y_free = j
        return x_free, y_free, d_max


    def motion(self):
        robot_frame = self.robot.base.get_location()
        x = robot_frame.frame.p.x
        y = robot_frame.frame.p.y
        theta_H = robot_frame.frame.orientation
        x_free, y_free, d_max = self.free_space_finder(x, y)
        vx = math.cos(theta_H)*(x_free+x)+math.sin(theta_H)*(y_free+y)
        vy = math.cos(theta_H)*(y_free+y)-math.sin(theta_H)*(x_free+x)
        vth = 0
        duration = d_max / 0.1
        self.robot.base.force_drive(vx, vy, vth, duration)



if __name__ == "__main__":
    rospy.init_node("test_grasping")
    robot = get_robot("hero")
    bt = Lethal_Zone(robot)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        bt.motion()
        r.sleep()

