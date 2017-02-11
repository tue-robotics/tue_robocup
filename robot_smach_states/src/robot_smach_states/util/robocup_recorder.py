import roslaunch, rospkg
import os.path
import rospy


def start_robocup_recorder(robot_name):

    try:
        package_path = rospkg.RosPack().get_path('%s_bringup' % robot_name)
    except rospkg.ResourceNotFound as e:
        rospy.logerr("Could not find bringup package of robot %s" % robot_name)
        return

    file_path = package_path + "/launch/record_robocup.launch"
    if not os.path.isfile(file_path):
        rospy.logerr("Could not find record_robocup.launch in bringup package of robot %s" % robot_name)
        return

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [file_path])
    launch.start()
