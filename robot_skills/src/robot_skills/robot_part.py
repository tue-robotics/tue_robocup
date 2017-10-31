# Maintainer: Janno Lunenburg <jannolunenburg@gmail.com>

# ROS
import rospy
import actionlib


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

    def wait_for_connections(self, timeout):
        """
        Waits for the connections until they are connected
        :param timeout: timeout in seconds
        :return: bool indicating whether all connections are connected
        """
        start = rospy.Time.now()
        t = rospy.Duration(timeout)
        r = rospy.Rate(20)
        # Loop until the timeout
        while (rospy.Time.now() - start) < t:
            # If everything is connected: return True
            if len(self.__ros_connections) == 0:
                return True
            # Check all connections
            new_connections = {}
            for k, v in self.__ros_connections.iteritems():
                # print "Checking {}".format(k)
                connected = False
                # Check actionlib connection
                if isinstance(v, actionlib.SimpleActionClient):
                    connected = v.wait_for_server(rospy.Duration(0.01))
                elif isinstance(v, rospy.ServiceProxy):
                    # Check service connection
                    # Need to use try-except in case of service since this throws an exception if not connected.
                    try:
                        v.wait_for_service(timeout=0.01)
                        connected = True
                    except:
                        connected = False
                else:
                    rospy.logerr("Don't know what to do with a {}".format(type(v)))
                # If connected, remove from the list
                if connected:
                    rospy.logdebug("Connected to {}".format(k))
                    # self.__ros_connections = {name: connection
                    #                           for name, connection in self.__ros_connections.iteritems() if name != k}
                else:
                    new_connections[k] = v

            self.__ros_connections = new_connections
            r.sleep()

        for k, v in self.__ros_connections.iteritems():
            rospy.logerr("{} not connected timely".format(k))
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

    def _add_connection(self, name, connection):
        """
        Adds a connection to the internal dict with connections that is used when initializing the robot object.
        :param name: name of the connection
        :param connection: connection to add
        """
        self.__ros_connections[name] = connection
