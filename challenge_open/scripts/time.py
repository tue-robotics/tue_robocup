#!/usr/bin/python
import rospy

from std_msgs.msg import String
from std_srvs.srv import Empty
from datetime import datetime, timedelta

from challenge_open.srv import *

class Time:
    def __init__(self, tzdiff):
        self.timezone_difference = timedelta(hours=tzdiff)
        # self.start_time = None
        # self.countdown_time = None

        self.countdown_time = timedelta(minutes=10, seconds=0)
        self.start_time = datetime.now() + self.timezone_difference

        line = "Starting countdown from " + str(self.countdown_time)
        print line
        

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

            if time_left.days < 0:
                line = " You better hurry up! "
                # over_time = timedelta(0) - time_left
                # over_time_min = ( over_time.seconds % 3600 ) / 60
                # over_time_sec = over_time.seconds % 60
                
                # if over_time_min == 0 and over_time.seconds > 0:
                #     line = " Our time was already up " + str(over_time.seconds) + " seconds ago. I'll stop the timer now. "
                # else:
                #     line = " Our time was already up " + str(over_time_min) + " minutes ago. I'll stop the timer now. "
                self.countdown_time = None
                self.start_time = None
            else:
                line = "We still have " + str(time_left_min) + " minutes and " + str(time_left_sec) + " seconds left. "
            print line
            return line

        # If the timer is counting up
        elif self.start_time:
            duration_dt  = current_time - self.start_time
            duration_hrs = duration_dt.seconds / 3600
            duration_min = ( duration_dt.seconds % 3600 ) / 60
            duration_sec = duration_dt.seconds % 60

            line = "You started the timer " + str(duration_hrs) + " hours " + str(duration_min) + " minutes and " + str(duration_sec) + " seconds ago."
            print line
            return line

        # If the timer is not counting at all
        else:
            print current_time
            # line = "The current time is " + str(current_time.time().hour) + ":" + str(current_time.time().minute)
            line = "I don't know. You should have set a timer, pal. But it's " + str(current_time.time().hour) + ":" + str(current_time.time().minute) + " now."
            print line
            return line

    def is_running(self, req):
        if self.start_time:
            return True


if __name__ == "__main__":

    rospy.init_node('time_server')

    # HACK!!!
    timezone_difference = 6
    # !!!

    time = Time(tzdiff=timezone_difference)

    set_clock_service       = rospy.Service('timer/start_clock', EmptyBool, time.start_clock)
    set_countdown_service   = rospy.Service('timer/start_countdown', StartCountdown, time.start_countdown)
    give_time_service       = rospy.Service('timer/get_time', EmptyString, time.get_time)
    is_running_service      = rospy.Service('timer/timer_running', EmptyBool, time.is_running)

    while not rospy.is_shutdown():
        rospy.spin()