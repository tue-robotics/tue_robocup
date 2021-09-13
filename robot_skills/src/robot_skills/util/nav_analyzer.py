# System
import datetime
import os
import signal
import subprocess
import xml.etree.ElementTree as ET

import PyKDL as kdl
# ROS
import nav_msgs.msg
import rospy

import tf2_ros
# noinspection PyUnresolvedReferences
import tf2_kdl


class NavAnalyzer:

    def __init__(self, robot_name):

        self._robot_name = robot_name
        self.rosbag = False
        rospy.logdebug("Nav_analyser: Bagging = {0}".format(self.rosbag))

        # Path
        # ToDo: make easier
        date = datetime.datetime.now()
        datestr = "{0}".format(date.year)
        if date.month < 10:
            datestr += "0"
        datestr += "{0}".format(date.month)
        if date.day < 10:
            datestr += "0"
        datestr += "{0}".format(date.day)

        self.path = os.path.join(os.environ["HOME"], "ros/data/private/recorded/rosbags/nav_data", datestr)
        if not os.path.isdir(self.path):
            os.makedirs(self.path)

        self.filename = os.path.join(self.path, "Summary.xml")

        # Initialize XML element
        self.logitem = ET.Element("item")

        # Subprocess
        self.pro = None

        # Odometry subscriber
        self.odom_sub = rospy.Subscriber("/{}/base/measurements".format(self._robot_name), nav_msgs.msg.Odometry,
                                         self.odomCallback)

        # Indicates whether measuring or not
        self.active = False

        # Bag topics
        if self.rosbag:
            topics = ["/tf",
                      "/tf_static"
                      "/{}/base/references",
                      "/{}/base/measurements",
                      "/{}/initialpose",
                      "/{}/joint_states",
                      "/{}/base_laser/scan",
                      "/{}/base_laser/scan_raw",
                      "/{}/torso_laser/scan",
                      "/{}/torso_laser/scan_raw",
                      "/{}/top_kinect/rgbd",
                      "/{}/ed/gui/entities",
                      "/{}/ed/profile/ed",
                      "/{}/local_planner/visualization/markers/goal_pose_marker",
                      "/{}/global_planner/visualization/markers/global_plan",
                      "/{}/local_planner/DWAPlannerROS/local_traj",
                      "/{}/local_planner/dwa_planner/cost_cloud",
                      "/{}/local_planner/action_server/goal"]

            self.base_cmd = ["rosbag", "record"]
            for topic in topics:
                self.base_cmd.append(topic.format(self._robot_name))

        # Initialize variables
        self.previous_position = kdl.Vector(0.0, 0.0, 0.0)
        self.distance_traveled = 0.0
        self.nr_plan = 0
        self.nr_clear_costmap = 0
        self.nr_reset_costmap = 0
        self.starttime = rospy.Time.now()

    def start_measurement(self, startpose):
        # The distance traveled concerns this specific goal
        self.distance_traveled = 0.0
        self.nr_plan = 0
        self.nr_clear_costmap = 0
        self.nr_reset_costmap = 0

        # Set time stamp
        self.starttime = rospy.Time.now()
        stamp = self.getTimeStamp()

        # Fill XML element
        self.logitem.set("stamp", stamp)
        ET.SubElement(self.logitem, "plans")
        ET.SubElement(self.logitem, "clears")
        ET.SubElement(self.logitem, "resets")

        # Log startpose
        startposeitem = ET.SubElement(self.logitem, "startpose")
        self.kdl_frame_to_sub_element(startpose, startposeitem)

        # Start bagging
        # The os.setsid() is passed in the argument preexec_fn so
        # it's run after the fork() and before  exec() to run the shell.
        # cmd = self.base_cmd
        # cmd.extend(["-O", "~/ros/data/recorded/rosbags/nav_data/{0}".format(stamp)])
        if self.rosbag:
            cmd = self.base_cmd
            cmd.extend(["-O", self.path + "/" + stamp])
            self.pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            rospy.logwarn("Logging with PID {0}".format(self.pro.pid))

        # Make active
        self.active = True

    def stop_measurement(self, endpose, result):

        # Stop rosbagging
        if self.rosbag:
            os.killpg(self.pro.pid, signal.SIGINT)  # Send the signal to all the process groups

        # Compute duration
        endtime = rospy.Time.now()
        duration = endtime.to_sec() - self.starttime.to_sec()

        # Log endpose
        endposeitem = ET.SubElement(self.logitem, "endpose")
        self.kdl_frame_to_sub_element(endpose, endposeitem)

        # Make inactive
        self.active = False

        self.logitem.set("duration", "{0}".format(duration))

        self.logitem.set("distance", "{0}".format(self.distance_traveled))

        self.logitem.set("result", "{0}".format(result))

        self.indent(self.logitem)

        # Write data to file
        with open(self.filename, 'a') as datafile:
            # Writing is not done using tree.write: this will overwrite this. It is also possible to
            # load the tree from the datafile and put this item in the tree, but this will probably become inefficient
            # if the file grows
            datafile.write(ET.tostring(self.logitem, encoding='unicode', method="xml"))

        rospy.logdebug(
            "\n\nNavigation summary:\nCovered {0} meters in {1} seconds ({2}) m/s avg.\n"
            "Result = {3} with {4} plans, {5} clears and {6} resets\n\n".format(
                self.distance_traveled,
                duration,
                self.distance_traveled/duration,
                result,
                self.nr_plan,
                self.nr_clear_costmap,
                self.nr_reset_costmap))

    def abort_measurement(self):
        self.active = False

    def odomCallback(self, odom_msg):
        pose = odom_msg.pose.pose.position
        current_position = kdl.Vector(pose.x, pose.y, pose.z)
        if self.active:
            self.distance_traveled += kdl.diff(current_position, self.previous_position).Norm()

        self.previous_position = current_position

    def kdl_frame_to_sub_element(self, kdl_frame, element):
        x = kdl_frame.p.x()
        y = kdl_frame.p.y()
        phi = kdl_frame.M.GetRPY()[2]  # Get the yaw
        element.set("x", "{0}".format(x))
        element.set("y", "{0}".format(y))
        element.set("phi", "{0}".format(phi))

    def getTimeStamp(self):
        stamp = datetime.datetime.now()
        datestr = "{0}".format(stamp.year)
        if stamp.month < 10:
            datestr += "0"
        datestr += "{0}".format(stamp.month)
        if stamp.day < 10:
            datestr += "0"
        datestr += "{0}_".format(stamp.day)
        if stamp.hour < 10:
            datestr += "0"
        datestr += "{0}".format(stamp.hour)
        if stamp.minute < 10:
            datestr += "0"
        datestr += "{0}".format(stamp.minute)
        if stamp.second < 10:
            datestr += "0"
        datestr += "{0}_{1}".format(stamp.second, stamp.microsecond)
        return datestr

    def indent(self, elem, level=0):
        i = "\n" + level*"  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "  "
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                self.indent(elem, level+1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i
