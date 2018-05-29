# ROS
import rospy
from std_srvs.srv import Trigger, TriggerRequest


def start_robocup_recorder(robot_name):
    service_name = '%s/robocup_recorder_toggle'%robot_name
    try:
        rospy.wait_for_service(service_name, timeout=1.0)
    except rospy.ROSException as e:
        rospy.logwarn("Service 'robocup_recorder_toggle' unavailable, unable to start recording.")
        return

    try:
        s = rospy.ServiceProxy(service_name, Trigger)
        s()
    except rospy.ServiceException as e:
        rospy.logwarn("Unable to start recording: %s" % e)

