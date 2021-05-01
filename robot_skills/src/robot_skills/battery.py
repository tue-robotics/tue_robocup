from sensor_msgs.msg import BatteryState

from robot_skills.robot_part import RobotPart


class Battery(RobotPart):
    """
    Interface to a battery. This implementation assumes a recent measurement to be available
    """
    def __init__(self, robot_name, tf_buffer, location):
        """
        @param robot_name: name of the robot
        @param tf_buffer: tf_buffer of the robot (needed because it is a robot part)
        @param location: string indicating the location of the battery
        """
        super(Battery, self).__init__(robot_name=robot_name, tf_buffer=tf_buffer)
        self.location = location
        self.battery_sub = self.create_subscriber("battery", BatteryState, self.battery_callback)
        self.recent_msg = None

    def battery_callback(self, battery_state_msg):
        if battery_state_msg.location == self.location:
            self.recent_msg = battery_state_msg

    @property
    def percentage(self):
        """"
        Percentage of the battery expressed between 1 (full) and 0 (empty)
        """
        if self.recent_msg is None:
            return None
        return self.recent_msg.percentage

    @property
    def is_charging(self):
        """
        boolean indicating whether or not the battery is charging
        """
        if self.recent_msg is None:
            return None
        return self.recent_msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
