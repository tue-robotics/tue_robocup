from robot_skills.robot_part import RobotPart


class GripperMeasurement(object):
    """
    Class holding measurements from the distance sensor on the grippers
    """
    EMPTY = -1
    UNKNOWN = 0
    HOLDING = 1

    def __init__(self, distance):
        """
        Constructor

        :param distance: float with measured distance
        """
        self._distance = distance
        self._stamp = rospy.Time.now()

        # If the grasp sensor distance is smaller than this value, the gripper is holding an object
        self.GRASP_SENSOR_THRESHOLD = rospy.get_param("skills/arm/grasp_sensor/threshold", 0.1)
        self.GRASP_SENSOR_TIMEOUT = rospy.get_param("skills/arm/grasp_sensor/timeout", 0.5)
        self.GRASP_SENSOR_LIMITS = tuple(rospy.get_param("skills/arm/grasp_sensor/limits", [0.0025, 0.18]))

    def _is_recent(self):
        """
        Checks if the sensor data is recent

        :return: bool True if recent, i.e., measurement is less than GRASP_SENSOR_TIMEOUT old, False otherwise
        """
        return (rospy.Time.now() - self._stamp).to_sec() < self.GRASP_SENSOR_TIMEOUT

    @property
    def distance(self):
        """
        Returns the measured distance. If the measurement is too old or the distance is outside of of the provided
        limits, NaN is returned

        :return: float with distance if valid, NaN otherwise
        """
        # Check if data is recent
        if not self._is_recent():
            return float('nan')
        elif not self.GRASP_SENSOR_LIMITS[0] < self._distance < self.GRASP_SENSOR_LIMITS[1]:
            return float('nan')
        else:
            return self._distance

    @property
    def is_holding(self):
        """
        Returns if the gripper is holding anything based on the measurement, i.e., if the measurement is recent and
        the value is between the lower limit and the sensor threshold

        :return: bool if holding
        """
        return self._is_recent() and self.GRASP_SENSOR_LIMITS[0] < self._distance < self.GRASP_SENSOR_THRESHOLD

    @property
    def is_unknown(self):
        """
        Returns if the state is unknown, i.e., either the measurement is outdated or the distance is less than the
        limit

        :return: bool if unknown
        """
        return not self._is_recent() or self._distance < self.GRASP_SENSOR_LIMITS[0]

    @property
    def is_empty(self):
        """
        Returns if the gripper is empty, i.e., the measurement is recent and the value is greater than the threshold

        :return: bool if holding
        """
        return self._is_recent() and self._distance > self.GRASP_SENSOR_THRESHOLD

    def __repr__(self):
        return "Distance: {}, is_holding: {}, is_unknown: {}, " \
               "is_empty: {}".format(self.distance, self.is_holding, self.is_unknown, self.is_empty)


class GraspSensor(RobotPart):
    """
    Sensor to detect whether or not the robot is holding an object.
    """

    def __init__(self, robot_name, tf_listener, side):
        """
        constructor

        :param robot_name: robot_name
        :param tf_listener: tf_server.TFClient()
        :param side: left or right arm of the robot.
        """
        super(GraspSensor, self).__init__(robot_name=robot_name, tf_listener=tf_listener)
        # Init grasp sensor subscriber
        self._grasp_sensor_state = GripperMeasurement(0.0)
        rospy.Subscriber("/" + robot_name + "/" + side + "_arm/proximity_sensor",
                         std_msgs.msg.Float32MultiArray, self._grasp_sensor_callback)

    @property
    def object_in_gripper_measurement(self):
        """
        Returns whether the gripper is empty, holding an object or if this is unknown

        :return: latest GripperMeasurement
        """
        return self._grasp_sensor_state

    @property
    def grasp_sensor_distance(self):
        """
        Returns the sensor distance. If no recent measurement is available or the measurement is outside bounds
        and hence unreliable, NaN is returned

        :return: float with distance
        """
        return self._grasp_sensor_state.distance

    def _grasp_sensor_callback(self, msg):
        """
        Callback function for grasp sensor messages

        :param msg: std_msgs.msg.Float32MultiArray
        """
        self._grasp_sensor_state = GripperMeasurement(msg.data[0])
