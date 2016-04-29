#!/usr/bin/python

"""Data collector

Usage:
  data_collector.py <name> <start_time> <end_time>
  data_collector.py <name> <start_time> <start_date> <end_time> <end_date>
  data_collector.py (-h | --help)
  data_collector.py --version

Examples:
  data_collector.py reo2016_challenge_navigation 15:30 15:40
  data_collector.py rwc_2016_challenge_restaurant 15:30 now
  data_collector.py rwc_2016_challenge_speech_recognition 15:30 2015-03-02 17:40 2015-04-08

Options:
  -h --help     Show this screen.

"""
from docopt import docopt
from datetime import datetime
import sys
import os
from glob import glob
import shutil

GLOBS = {
    "faces": "/tmp/faces/*.jpg",
    "speech": "/tmp/*.wav"
}


def _get_modification_date(file_name):
    try:
        mtime = os.path.getmtime(file_name)
    except OSError:
        print '\033[91mCannot get modification date from %s \033[0m' % file_name
        return None

    return datetime.fromtimestamp(mtime)


def _parse_start_end(args, current):
    try:
        start_time = datetime.strptime(args["<start_time>"], "%H:%M")
        if args["<end_time>"] == "now":
            end_time = current
        else:
            end_time = datetime.strptime(args["<end_time>"], "%H:%M")

        if args["<start_date>"]:
            start_date = datetime.strptime(args["<start_date>"], "%Y-%m-%d")
            end_date = datetime.strptime(args["<end_date>"], "%Y-%m-%d")
        else:
            start_date = current
            end_date = current

        start_combined = datetime.combine(start_date.date(), start_time.time())
        end_combined = datetime.combine(end_date.date(), end_time.time())

        if start_combined > end_combined:
            raise ValueError("Specified start is larger than end!")

    except ValueError as e:
        print '\033[91mInvalid input: %s \033[0m' % e
        sys.exit(1)

    return start_combined, end_combined

if __name__ == '__main__':
    now = datetime.now()

    arguments = docopt(__doc__, version='Data Collector 1.0')

    start, end = _parse_start_end(arguments, now)
    name = arguments["<name>"]

    print "I am going to collect data for '%s'" % name
    print "Datetime: [%s <--> %s]" % (start.strftime("%Y-%m-%d %H:%M"), end.strftime("%Y-%m-%d %H:%M"))

    raw_input("Press Enter to continue...")

    # Create dir if exist
    if not os.path.exists(name):
        os.makedirs(name)

    # Copy files
    for dir_name, glob_entry in GLOBS.iteritems():
        dir_name = name + "/" + dir_name
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)

        for file_name in glob(glob_entry):
            # Check if modification date is between bounds (or unknown)
            mod_date = _get_modification_date(file_name)
            if not mod_date or start < mod_date < end:
                print "Writing file '%s' to '%s'" % (file_name, dir_name)
                shutil.copy(file_name, dir_name)
