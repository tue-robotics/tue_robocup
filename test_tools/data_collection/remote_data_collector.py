#!/usr/bin/python

"""Remote Data collector

Usage:
  remote_data_collector.py <name> <start_time> <end_time> <remotes>...
  remote_data_collector.py (-h | --help)
  remote_data_collector.py --version

Examples:
  remote_data_collector.py reo2016_challenge_navigation 15:30 15:40 amigo1 amigo2 amigo3
  remote_data_collector.py rwc_2016_challenge_restaurant 15:30 now sergio1 sergio2 sergi3
  remote_data_collector.py rwc_2016_challenge_speech_recognition 15:30 17:40 localhost

Options:
  -h --help     Show this screen.

"""
from docopt import docopt
from datetime import datetime
import sys
import os
from glob import glob
import shutil
from util import parse_start_end


if __name__ == '__main__':
    now = datetime.now()
    now_str = now.strftime("%Y-%m-%d_%H-%M-%S")

    arguments = docopt(__doc__, version='Data Collector 1.0')

    start, end = parse_start_end(arguments, now)
    name = arguments["<name>"]
    remotes = arguments["<remotes>"]

    print "I am going to collect data for '%s' on '%s'" % (name, remotes)
    print "Datetime: [%s <--> %s]" % (start.strftime("%Y-%m-%d %H:%M"), end.strftime("%Y-%m-%d %H:%M"))

    # Create dir if exist
    if not os.path.exists(name):
        os.makedirs(name)

    for remote in remotes:
        dir_name = name + "/" + remote

        # Create dir if exist
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)

        remote_dir_name = "/tmp/%s" % now_str

        cmd = "rosrun test_tools data_collector.py %s %s %s --go" % (remote_dir_name, start.strftime("%H:%M"), end.strftime("%H:%M"))
        print ">> %s" % cmd

        os.system("ssh %s 'source ~/.tue/setup.bash && %s'" % (remote, cmd))

        cp_cmd = "scp -r %s:%s/* %s" % (remote, remote_dir_name, dir_name)
        print ">> %s" % cp_cmd

        os.system(cp_cmd)
