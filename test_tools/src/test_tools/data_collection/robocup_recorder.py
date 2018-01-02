#!/usr/bin/python

"""Data collector

"""
import rospy
import subprocess, os
from std_srvs.srv import Trigger, TriggerResponse

class Recorder():
    def __init__(self):
        self._recording_process = None
        self._s = rospy.Service('robocup_recorder_toggle', Trigger, self._toggle_recording)
        self._recorder_name = rospy.get_param('recorder_name', 'robocup_recorder')

        if self._recorder_name == rospy.get_name():
            rospy.logerr("Parameter recorder_name should be different from the robocup recorder server node name.")
            exit(1)

        self._filename_prefix = rospy.get_param('filename_prefix', '/tmp/robocup_record')
        self._topic_regex = rospy.get_param('topic_regex', "'.*record/.*'")

    def _toggle_recording(self, req):
        if not self._recording_process:
            FNULL = open(os.devnull, 'w')
            self._recording_process = subprocess.Popen(['rosbag', 'record',
                                                        '--regex', self._topic_regex,
                                                        '-o', self._filename_prefix,
                                                        '__name:=%s' % self._recorder_name],
                                                       stdout=FNULL,
                                                       stderr=subprocess.STDOUT)
            rospy.loginfo("Started recording")
            return TriggerResponse(True, 'started recording')
        else:
            self.clean_up()
            return TriggerResponse(False, 'stopped recording')

    def clean_up(self):
        FNULL = open(os.devnull, 'w')
        subprocess.Popen(['rosnode', 'kill', self._recorder_name],
                         stdout=FNULL,
                         stderr=subprocess.STDOUT)
        self._recording_process.wait()
        self._recording_process = None
        rospy.loginfo("Killed the recording process.")

if __name__ == "__main__":
    rospy.init_node('robocup_recorder_server')
    recorder = Recorder()
    rospy.spin()
    recorder.clean_up()
