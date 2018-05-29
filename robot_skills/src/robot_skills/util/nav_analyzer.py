# System
import datetime
import os
import signal
import subprocess
import xml.etree.ElementTree as ET

# ROS
import nav_msgs.msg
import PyKDL as kdl
import rospy

# TU/e Robotics
from robot_skills.util.kdl_conversions import point_msg_to_kdl_vector


class NavAnalyzer:

    def __init__(self, robot_name):

        self._robot_name = robot_name
        self.rosbag = False
        rospy.logdebug("Nav_analyser: Bagging = {0}".format(self.rosbag))

        ''' Path '''
        # ToDo: make easier
        date = datetime.datetime.now()
        datestr = "{0}".format(date.year)
        if date.month < 10:
            datestr += "0"
        datestr += "{0}".format(date.month)
        if date.day < 10:
            datestr += "0"
        datestr += "{0}".format(date.day)

        self.path = os.environ["HOME"]+"/ros/data/private/recorded/rosbags/nav_data/"+datestr
        if not os.path.isdir(self.path):
            os.makedirs(self.path)

        self.filename = self.path+'/Summary.xml'

        ''' Odometry subscriber '''
        self.odom_sub = rospy.Subscriber("/"+self._robot_name+"/base/measurements", nav_msgs.msg.Odometry, self.odomCallback)

        ''' Indicates whether measuring or not '''
        self.active = False

        ''' Bag topics '''
        if self.rosbag:
            topics = ["/tf",
                      "/"+self._robot_name+"/base/references",
                      "/"+self._robot_name+"/base/measurements",
                      "/"+self._robot_name+"/initialpose",
                      "/"+self._robot_name+"/joint_states",
                      "/amcl_pose",
                      "/"+self._robot_name+"/base_laser/scan",
                      "/"+self._robot_name+"/base_laser/scan_raw",
                      "/"+self._robot_name+"/torso_laser/scan",
                      "/"+self._robot_name+"/torso_laser/scan_raw",
                      "/"+self._robot_name+"/top_kinect/rgbd",
                      "/ed/gui/entities",
                      "/ed/profile/ed",
                      "/cb_base_navigation/local_planner_interface/visualization/markers/goal_pose_marker",
                      "/cb_base_navigation/global_planner_interface/visualization/markers/global_plan",
                      "/cb_base_navigation/local_planner_interface/DWAPlannerROS/local_traj",
                      "/cb_base_navigation/local_planner_interface/dwa_planner/cost_cloud",
                      "/cb_base_navigation/local_planner_interface/action_server/goal"]

            #plantopic = "/move_base"
            #if os.environ["AMIGO_NAV"] == "3d":
            #    plantopic += "_3d"
            #plantopic += "/AStarPlannerROS/plan"
            #topics.append(plantopic)

            self.base_cmd = "rosbag record "
            for topic in topics:
                self.base_cmd += (topic + " ")

        ''' Initialize variables '''
        self.previous_position = kdl.Vector(0.0, 0.0, 0.0)
        self.distance_traveled   = 0.0
        self.nr_plan             = 0
        self.nr_clear_costmap    = 0
        self.nr_reset_costmap    = 0
        self.starttime = rospy.Time.now()

    def start_measurement(self, startpose):

        ''' The distance traveled concerns this specific goal '''
        self.distance_traveled = 0.0
        self.nr_plan             = 0
        self.nr_clear_costmap    = 0
        self.nr_reset_costmap    = 0

        ''' Set time stamp '''
        self.starttime = rospy.Time.now()
        stamp          = self.getTimeStamp()

        ''' Initialize XML element '''
        self.logitem = ET.Element("item")
        self.logitem.set("stamp", stamp)
        self.planitems = ET.SubElement(self.logitem, "plans")
        self.clearitems= ET.SubElement(self.logitem, "clears")
        self.resetitems= ET.SubElement(self.logitem, "resets")

        ''' Log startpose '''
        startposeitem = ET.SubElement(self.logitem, "startpose")
        self.kdl_frame_to_sub_element(startpose, startposeitem)

        ''' Start bagging '''
        # The os.setsid() is passed in the argument preexec_fn so
        # it's run after the fork() and before  exec() to run the shell.
        #cmd = self.base_cmd + "-O ~/ros/data/recorded/rosbags/nav_data/" + "{0}".format(stamp)
        if self.rosbag:
            cmd = self.base_cmd + "-O " + self.path + "/" + stamp
            self.pro = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
            rospy.logwarn("Logging with PID {0}".format(self.pro.pid))

        ''' Make active '''
        self.active = True

    def stop_measurement(self, endpose, result):

        ''' Stop rosbagging '''
        if self.rosbag:
            os.killpg(self.pro.pid, signal.SIGINT)  # Send the signal to all the process groups

        ''' Compute duration '''
        endtime = rospy.Time.now()
        duration = endtime.to_sec() - self.starttime.to_sec()

        ''' Log endpose '''
        endposeitem = ET.SubElement(self.logitem, "endpose")
        self.kdl_frame_to_sub_element(endpose, endposeitem)

        ''' Make inactive '''
        self.active = False

        ''' Write data to file '''
        datafile = open(self.filename, 'a')

        ''' With data string '''
        #datastring = "{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\n".format(self.stamp,
        #                                                       duration,
        #                                                        self.distance_traveled,
        #                                                        result,
        #                                                        self.nr_plan,
        #                                                        self.nr_clear_costmap,
        #                                                        self.nr_reset_costmap)
        #datafile.write(datastring)

        ''' Alternative: xml '''
        self.logitem.set("duration", "{0}".format(duration))
        #duritem = ET.SubElement(self.logitem, "duration")
        #duritem.text = "{0}".format(duration)

        self.logitem.set("distance", "{0}".format(self.distance_traveled))
        #distitem = ET.SubElement(self.logitem, "distance")
        #distitem.text = "{0}".format(self.distance_traveled)

        self.logitem.set("result", "{0}".format(result))
        #resultitem = ET.SubElement(self.logitem, "result")
        #resultitem.text = "{0}".format(result)

        self.indent(self.logitem)
        #tree = ET.ElementTree(self.logitem)
        #tree.write(self.filename)

        ''' Writing is not done using tree.write: this will overwrite this. It is also possible to
        load the tree from the datafile and put this item in the tree, but this will probably become inefficient
        if the file grows '''
        datafile.write(ET.tostring(self.logitem, 'utf-8', method="xml"))
        #open(self.filename, 'a') as output
        #output.write(tree)

        ''' Display results '''
        rospy.logdebug("\n\nNavigation summary:\nCovered {0} meters in {1} seconds ({2}) m/s avg.\nResult = {3} with {4} plans, {5} clears and {6} resets\n\n".format(self.distance_traveled,
        duration,
        self.distance_traveled/duration,
        result,
        self.nr_plan,
        self.nr_clear_costmap,
        self.nr_reset_costmap))

        datafile.close()

    def abort_measurement(self):
        self.active = False

    def odomCallback(self, odom_msg):
        current_position = point_msg_to_kdl_vector(odom_msg.pose.pose.position)
        if self.active:
            self.distance_traveled += kdl.diff(current_position, self.previous_position).Norm()

        self.previous_position = current_position

    def kdl_frame_to_sub_element(self, kdl_frame, element):
        x   = kdl_frame.p.x()
        y   = kdl_frame.p.y()
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

