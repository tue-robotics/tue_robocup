import rospy

from sensor_msgs.msg import BatteryState
from robot_skills.robot_part import RobotPart


class Battery(RobotPart):
    def __init__(self, robot_name, tf_listener, location):
        super(Battery, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        self.location = location
        self.battery_sub = rospy.Subscriber("battery", BatteryState, self.battery_callback)
        self.recent_msg = None

    def battery_callback(self, battery_state_msg):
        if battery_state_msg.location == self.location:
            self.recent_msg = battery_state_msg

    @property
    def percentage(self):
        if self.recent_msg is None:
            return None
        return self.recent_msg.percentage

    @property
    def is_charging(self):
        if self.recent_msg is None:
            return None
        return self.recent_msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
