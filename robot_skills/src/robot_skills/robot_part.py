# Maintainer: Janno Lunenburg <jannolunenburg@gmail.com>

# System
import os

# ROS
import rospy
import actionlib

# Determine simulation mode
SIM_MODE = os.environ.get("ROBOT_REAL", "false").lower() != "true"


class RobotPart(object):
    """ Base class for robot parts """

    def __init__(self, robot_name, tf_listener):
        """
        Constructor

        :param robot_name: string with robot name
        :param tf_listener: tf listener object
        """
        self.robot_name = robot_name
        self.tf_listener = tf_listener

        self.__ros_connections = {}

        self.__diagnostics_name = ""

        # This is set to False by subscribe_hardware_status, because then apparently there is a meaningful check
        # If no such check exists, then assume it's operational unless overridden in subclass
        self._operational = True

    def load_param(self, param_name, default=None):
        """
        Loads a parameter from the parameter server, namespaced by robot name

        :param param_name: parameter name
        :param default: default value for when parameter unavailable
        :return: loaded parameters
        """
        if default is None:
            return rospy.get_param('/' + self.robot_name + '/' + param_name)
        else:
            return rospy.get_param('/' + self.robot_name + '/' + param_name, default)

    def wait_for_connections(self, timeout, log_failing_connections=True):
        """
        Waits for the connections until they are connected

        :param timeout: timeout in seconds
        :param log_failing_connections: (bool) whether to log errors if not connected. This is useful when checking
            multiple robot parts in a loop
        :return: bool indicating whether all connections are connected
        """
        start = rospy.Time.now()
        t = rospy.Duration(timeout)
        r = rospy.Rate(20)

        # Loop until the timeout
        while (rospy.Time.now() - start) < t:

            # If shutdown: return immediately without any further checks or prints
            if rospy.is_shutdown():
                return False

            # If everything is connected: return True
            if len(self.__ros_connections) == 0:
                return True
            # Check all connections
            new_connections = {}
            for name, connection in self.__ros_connections.items():
                rospy.logdebug("Checking {}".format(name))
                connected = False
                # Check actionlib connection
                if isinstance(connection, actionlib.SimpleActionClient):
                    connected = connection.wait_for_server(rospy.Duration(0.01))
                elif isinstance(connection, rospy.ServiceProxy):
                    # Check service connection
                    # Need to use try-except in case of service since this throws an exception if not connected.
                    try:
                        connection.wait_for_service(timeout=0.01)
                        connected = True
                    except Exception:
                        connected = False
                elif isinstance(connection, rospy.Subscriber):
                    connected = connection.get_num_connections() >= 1
                else:
                    rospy.logerr("Don't know what to do with a {}".format(type(connection)))
                # If connected, remove from the list
                if connected:
                    rospy.logdebug("Connected to {}".format(name))
                    # self.__ros_connections = {name: connection
                    #                           for name, connection in self.__ros_connections.items() if name != k}
                else:
                    new_connections[name] = connection

            self.__ros_connections = new_connections
            r.sleep()

        if log_failing_connections:
            for name, connection in self.__ros_connections.items():
                rospy.logerr("{} not connected timely".format(name))
        return False

    def create_simple_action_client(self, name, action_type):
        """
        Creates a simple actionlib client and waits for the action server

        :param name: string with the name of the action in the correct namespace
        :param action_type: action type of this action
        :return: the action client
        """
        ac = actionlib.SimpleActionClient(name, action_type)
        self._add_connection(name, ac)
        return ac

    def create_service_client(self, name, srv_type):
        """
        Creates a service client and waits for the server
        :param name: string with the name of the service in the correct namespace
        :param srv_type: service type
        :return: the service client
        """
        srv = rospy.ServiceProxy(name, srv_type)
        self._add_connection(name, srv)
        return srv

    def create_subscriber(self, name, *args, **kwargs):
        """
        Creates a Subscriber and add to the connections to check

        :param name: string with the name topic to subscribe
        :param args: other args passed to rospy.Subscriber
        :param kwargs: other keyword args passed to rospy.Subscriber
        :return: the Subscriber
        """
        sub = rospy.Subscriber(name, *args, **kwargs)
        self._add_connection(name, sub)
        return sub

    def _add_connection(self, name, connection):
        """
        Adds a connection to the internal dict with connections that is used when initializing the robot object.

        :param name: name of the connection
        :param connection: connection to add. This might be a ServiceProxy, ActionClient or Subscriber
        """
        self.__ros_connections[name] = connection

    @property
    def operational(self):
        """
        Check whether this bodypart's hardware is operational
        If the associated hardware is not yet up, has an error etc, the bodypart is not operational. Note: in
        in simulation: operational is always true

        :rtype: bool True is part is ready to do work, False otherwise
        """
        if SIM_MODE:
            return True
        else:
            return self._operational

    def subscribe_hardware_status(self, name):
        """
        Start to check if the bodypart is operational. To do so, subscribe to the hardware status/diagnostics

        :param name: check on the level of the diagnostic_msgs/DiagnosticStatus with this name
        """

        self._operational = False

        self.__diagnostics_name = name

    def unsubscribe_hardware_status(self):
        """
        When process_hardware_status sees that self.__diagnostics_name evaluates to False,
            it will not process the diagnostics and ignore them
        """
        self.__diagnostics_name = None

    def process_hardware_status(self, diagnostic_dict):
        """
        hardware_status callback to determine if the bodypart is operational

        :param diagnostic_dict: dictionary[str, diagnostic_msgs.msg.DiagnosticStatus]
        :return: no return
        """
        if not self.__diagnostics_name:
            return

        diag = diagnostic_dict.get(self.__diagnostics_name, None)

        if not diag:
            # (Logwarn throttle: show this message once every day (24 * 3600 seconds)
            rospy.logwarn_throttle(86400.0, 'no diagnostic msg received for the %s' % self.__diagnostics_name)
        else:
            # TODO: diagnostic_msgs.DiagnosticStatus.OK instead of our own enumeration.
            # 0. Stale
            # 1. Idle
            # 2. Operational
            # 3. Homing
            # 4. Error
            if diag.level != 2:
                self._operational = False
            else:
                self._operational = True

    def reset(self):
        """
        Reset body part. This should be implemented in the subclass. Should always return a bool.

        :return: Succes
        :rtype: bool
        """
        return True
