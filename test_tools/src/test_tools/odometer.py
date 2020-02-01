#! /usr/bin/python

from __future__ import print_function

import os
import socket
import time
import csv
from fnmatch import fnmatch
import rospy
from math import sqrt, pow, pi
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry

DEFAULT_PATH = "~/odometer"
DEFAULT_FILENAME = 'odometer'
EXT = '.csv'
ROUND_LEVEL = 5


class Odometer:
    """
    Odometer logs odometry. The odometry is measured in a callback function and sampled to a data storage in sample().
    The data is written to a csv file, which is appended with a date. This is done is write(). In shutdown(), sample() and
    write() are called to prevent data lost. This function is called on rospy shutdown. You can activate periodic
    writing by calling active_write() in a loop, which the maximum length of your data storage as argument.
    """
    def __init__(self, path=DEFAULT_PATH, filename=DEFAULT_FILENAME):
        """
        Constructor
        In the constructor old data is retrieved, if possible. Otherwise it starts from zero.

        :param path: Path to store the data. Path is expanded by hostname(lowercase). Can be relative or in home folder
        :type path: str
        :param filename: Filename of data file. Filenames are appended with date and extension.
        :type filename: str
        """
        if fnmatch(path, "~*"):  # if path is in home folder
            path = os.path.expanduser(path)
        path = os.path.abspath(path)  # returns abs path, also when path is already abs.

        hostname = socket.gethostname()
        date = time.strftime("%Y_%m_%d")

        hostfolderpath = os.path.join(os.path.expanduser(path), hostname.lower())
        self.newfilepath = os.path.join(hostfolderpath, filename + "_" + date + EXT)
        lastfilepath = ""

        self.file_has_header = False

        self.total_time = 0
        self.total_distance = 0
        self.total_rotation = 0

        self.data = []  # list of dicts with data
        self.last_pose = None
        self.last_time = rospy.Time.now().secs

        # Check if there exist a previous file to read from
        if os.path.exists(hostfolderpath):
            if os.path.exists(self.newfilepath):
                lastfilepath = self.newfilepath
                rospy.logdebug("Last data file is today's file")
            else:
                files = [item for item in sorted(os.listdir(hostfolderpath), reverse=True) if
                         os.path.isfile(os.path.join(hostfolderpath, item))]
                if files:
                    for file in files:
                        if fnmatch(file, filename+"_*"+EXT):
                            filepath = os.path.join(hostfolderpath, file)
                            lastfilepath = filepath
                            rospy.logdebug("Found last file: {}".format(lastfilepath))
                            break

                    if not lastfilepath:
                        rospy.logdebug("Not found a correct data file in the folder: {}".format(hostfolderpath))
                else:
                    rospy.logdebug("No data files there yet in: {}".format(hostfolderpath))
        else:
            os.makedirs(hostfolderpath)
            rospy.logdebug("No folder for hostname: '{}' found".format(hostname))

        # look for data in last data file, if found
        if not lastfilepath:
            rospy.logdebug("No previous data file found. Starting from zero")
        else:
            rospy.logdebug("Reading from last data file: {}".format(lastfilepath))
            with open(lastfilepath, "r") as f:
                reader = csv.reader(f)
                # in case an empty file is there. The first line will stay empty. Therefore the header needs to be found
                found_header = False
                for header in reader:
                    if header == ['timestamp', 'distance', 'rotation', 'time']:
                        found_header = True
                        break

                if found_header:
                    # Iterate over all lines and get the last valid line. So it continues after an invalid line.
                    last_row = None
                    while True:
                        try:
                            last_row = next(reader)
                        except csv.Error:
                            pass
                        except StopIteration:
                            break
                    if last_row:
                        last_row = dict(zip(header, last_row))
                        try:
                            self.total_distance = float(last_row['distance'])
                            self.total_rotation = float(last_row['rotation'])
                            self.total_time = int(float(last_row['time']))
                            if lastfilepath == self.newfilepath:
                                self.file_has_header = True
                            rospy.logdebug("Loaded data from file: {}".format(lastfilepath))
                        except Exception as e:
                            rospy.logerr(e)
                            rospy.signal_shutdown("Unable to read last data. Last data is corrupt")
                else:
                    rospy.logerr("No header found in file: {}".format(lastfilepath))
                    rospy.signal_shutdown("Shutdown, because last data file has no header")

        rospy.loginfo("Logging odometry to file: {}".format(self.newfilepath))

        rospy.Subscriber("odom", Odometry, self.callback)
        rospy.on_shutdown(lambda: self.shutdown())

    def sample(self):
        """
        Current measurements are stored in self.data

        :return: no return
        """
        new_time = rospy.Time.now().secs
        time_delta = new_time - self.last_time
        self.total_time += time_delta
        self.last_time = new_time

        timestamp = time.strftime("%Y_%m_%d_%H_%M_%S")
        dist = round(self.total_distance, ROUND_LEVEL)
        rot = round(self.total_rotation, ROUND_LEVEL)
        t = self.total_time
        self.data.append({'timestamp': timestamp, 'distance': dist, 'rotation': rot, 'time': t})

    def write(self):
        """
        Writing all data in self.data to the data file and closing it again. This should prevent file corruption.

        :return: no return
        """
        # Create today's file if not already there
        if os.path.exists(self.newfilepath):
            rospy.logdebug("Today's file already exists")
        else:
            rospy.logdebug("First time writing in today's file")

        with open(self.newfilepath, "a", 1) as new_file:  # 1=line-buffered
            try:
                writer = csv.DictWriter(new_file, fieldnames=['timestamp', 'distance', 'rotation', 'time'])
                if not self.file_has_header:
                    rospy.logdebug("Printing header of csv file")
                    writer.writeheader()
                    self.file_has_header = True
                if self.data:
                    rospy.logdebug("Writing data to csv file")
                    writer.writerows(self.data)
                    self.data = []

            except Exception as e:
                rospy.logerr(e)

    def callback(self, msg):
        """
        Measuring the displacement based on the new position

        :param msg: Odometry msg with position and rotation information
        :type msg: nag_msgs.msg.Odometry
        :return: no return
        """
        if self.last_pose:
            new_pose = msg.pose.pose
            pos = new_pose.position
            pos_old = self.last_pose.position
            distance_delta = sqrt(pow(pos.x-pos_old.x, 2) + pow(pos.y-pos_old.y, 2))
            if distance_delta < 0.5:  # If delta is too big, it is incorrect. Not doing anything with this data
                self.total_distance += distance_delta

                new_orientation = new_pose.orientation
                old_orientation = self.last_pose.orientation
                new_rotation = euler_from_quaternion([new_orientation.x,
                                                      new_orientation.y,
                                                      new_orientation.z,
                                                      new_orientation.w])
                old_rotation = euler_from_quaternion([old_orientation.x,
                                                      old_orientation.y,
                                                      old_orientation.z,
                                                      old_orientation.w])
                rotation_delta = new_rotation[2] - old_rotation[2]
                if rotation_delta >= pi:
                    rotation_delta -= 2*pi
                elif rotation_delta <= -pi:
                    rotation_delta += 2*pi
                self.total_rotation += abs(rotation_delta)
            else:
                rospy.logerr("Distance delta too big (%f m), ignoring this step" % distance_delta)

        self.last_pose = msg.pose.pose

    def activate_write(self, length):
        """
        If self.data contains more than X samples, all the data is written to the file

        :param length:
        :type length: int
        :return: no return
        """
        if len(self.data) >= length:
            self.write()

    def shutdown(self):
        """
        To prevent data loss between last sample and shutdown, this function is called to sample and write all the data.

        :return: no return
        """
        self.sample()
        self.write()
