#!/usr/bin/python
import rospy

from std_msgs.msg import String
from std_srvs.srv import Empty
from datetime import datetime, date, timedelta

from challenge_final.srv import *

class Time:
    def __init__(self, tzdiff):
        self.timezone_difference = timedelta(hours=tzdiff)
        self.start_time = None
        self.countdown_time = None

    def start_clock(self, req):
        self.start_time = datetime.now() + self.timezone_difference

        line = "Starting clock at " + str(self.start_time.time().hour) + " " + str(self.start_time.time().minute)
        print line
        return True

    def start_countdown(self, req):
        self.countdown_time = timedelta(minutes=req.mins, seconds=req.secs)
        self.start_time = datetime.now() + self.timezone_difference

        line = "Starting countdown from " + str(self.countdown_time)
        print line
        return True

    def get_time(self, req):
        current_time = datetime.now() + self.timezone_difference

        # If the timer is counting down
        if self.countdown_time:
            time_left = self.countdown_time - (current_time - self.start_time)
            time_left_hrs = time_left.seconds / 3600
            time_left_min = ( time_left.seconds % 3600 ) / 60
            time_left_sec = time_left.seconds % 60

            line = " " + str(time_left_min) + " minutes and " + str(time_left_sec) + " seconds "
            print line
            return line

        # If the timer is counting up
        elif self.start_time:
            duration_dt  = current_time - self.start_time
            duration_hrs = duration_dt.seconds / 3600
            duration_min = ( duration_dt.seconds % 3600 ) / 60
            duration_sec = duration_dt.seconds % 60

            line = "Time since start: " + str(duration_hrs) + " hrs " + str(duration_min) + " min and " + str(duration_sec) + " sec."
            print line
            return line

        # If the timer is not counting at all
        else:
            line = str(current_time.time().hour) + " " + str(current_time.time().minute)
            print line
            return line

    def is_running(self, req):
        if self.start_time:
            return True


if __name__ == "__main__":

    rospy.init_node('time_server')

    timezone_difference = 0

    time = Time(tzdiff=timezone_difference)

    set_clock_service       = rospy.Service('finals/start_clock', EmptyBool, time.start_clock)
    set_countdown_service   = rospy.Service('finals/start_countdown', StartCountdown, time.start_countdown)
    give_time_service       = rospy.Service('finals/get_time', EmptyString, time.get_time)
    is_running_service      = rospy.Service('finals/timer_running', EmptyBool, time.is_running)

    while not rospy.is_shutdown():
        rospy.spin()