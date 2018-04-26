import robot


class Sergio(robot.Robot):
    """docstring for Sergio"""
    def __init__(self, wait_services=False):
        super(Sergio, self).__init__(robot_name="sergio", wait_services=False)

        self._ignored_parts = ["left_arm", "right_arm", "spindle"]
