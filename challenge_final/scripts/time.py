#!/usr/bin/python

from datetime import datetime, timedelta

from challenge_final.srv import *

import time


class StopWatch:
    def __init__(self, start_time=datetime.now()):
        self._start_time = start_time

    def getElapsedTime(self):
        return datetime.now() - self._start_time

    def reset(self):
        self._start_time = datetime.now()


class Clock:
    def __init__(self, robot, tzdiff=0):
        self._robot = robot
        self._timezone_difference = timedelta(hours=tzdiff)
        self._stopwatches = []
        self._timers = {}

    def start_stopwatch(self):
        stopwatch_number = len(self._stopwatches)
        self._stopwatches.append(StopWatch())

        print "Starting stopwatch %i" % stopwatch_number

        return stopwatch_number

    def get_stopwatch_time(self, stopwatch_number=0):
        if stopwatch_number >= len(self._stopwatches) or stopwatch_number < 0:
            now = datetime.now()
            return now - now

        return self._stopwatches[stopwatch_number].getElapsedTime()

    def get_stopwatch_hrs(self, stopwatch_number=0):
        return get_stopwatch_time(stopwatch_number).seconds / 3600

    def get_stopwatch_mins(self, stopwatch_number=0):
        return (get_stopwatch_time(stopwatch_number).seconds % 3600) / 60

    def get_stopwatch_secs(self, stopwatch_number=0):
        return get_stopwatch_time(stopwatch_number).seconds % 60

    def count_up_out_loud(self, secs):
        count_seconds = secs
        current_seconds = 1

        while current_seconds < count_seconds:
            self._robot.speech.speak(str(current_seconds), block=False)
            current_seconds += 1
            time.sleep(1.0)

        self._robot.speech.speak(str(current_seconds), block=False)

    def count_down_out_loud(self, secs):
        while secs > 0:
            secs -= 1
            self._robot.speech.speak(str(current_seconds), block=False)
            time.sleep(1.0)

    def tell_time(self):
        current_real_time = datetime.now() + self._timezone_difference
        self._robot.speech.speak(
            "It is %s:%s" % (str(current_real_time.time().hour), str(current_real_time.time().minute)))

    def start_timer(self, hrs, mins, secs):
        countdown_time = timedelta(hours=hrs, minutes=mins, seconds=secs)
        end_time = datetime.now() + countdown_time
        stopwatch_number = start_stopwatch()
        self._timers[stopwatch_number] = end_time

        print "Starting timer for %s using stopwatch %i" % (str(self.countdown_time), stopwatch_number)
        return stopwatch_number

    def tell_remaining_timer_time(self, timer_number):
        if timer_number >= len(self._stopwatches) or timer_number < 0:
            return False

        time_left = self._timers[timer_number] - datetime.now()

        time_left_hrs = time_left.seconds / 3600
        time_left_min = (time_left.seconds % 3600) / 60
        time_left_sec = time_left.seconds % 60

        if time_left.days < 0:
            line = "You better hurry up!"
            return True
        else:
            if time_left_hrs > 0:
                line = "We still have %i hours, %i minutes and %i seconds left. " % (
                    time_left_hrs, time_left_min, time_left_sec)
            else if time_left_min > 0:
                line = "We still have %i minutes and %i seconds left." % (time_left_min, time_left_sec)
            else if time_left_sec < 10:
                line = "Your time is almost up!"

        self._robot.speech.speak(line)

        return True
