#!/usr/bin/python

import rospy
from hmi_msgs.msg import QueryActionGoal, QueryActionResult

from datetime import datetime
import os

# import pyaudio
# import wave
import json

# CHUNK = 1024
# FORMAT = pyaudio.paInt16
# CHANNELS = 2
# RATE = 44100

# P = pyaudio.PyAudio()

RECORDING = False
RESULT = None
# STREAM = P.open(format=FORMAT,
#                 channels=CHANNELS,
#                 rate=RATE,
#                 input=True,
#                 frames_per_buffer=CHUNK)


def start(msg):
    global RECORDING

    if RECORDING:
        return

    # frames = []

    rospy.loginfo("Starting HMI recording")
    RECORDING = True

    # last_print = rospy.Time.now()
    while RECORDING:
        rospy.sleep(0.1)
    #     data = STREAM.read(CHUNK)
    #     frames.append(data)
    #     if rospy.Time.now() - last_print > rospy.Duration(1):
    #         rospy.loginfo(".. recording")
    #         last_print = rospy.Time.now()

    now = datetime.now()
    # wav_filename = "%s/%s.wav" % (STORAGE_FOLDER, now.strftime("%Y-%m-%d-%H-%M-%d-%f"))
    # wf = wave.open(wav_filename, 'wb')
    # wf.setnchannels(CHANNELS)
    # wf.setsampwidth(P.get_sample_size(FORMAT))
    # wf.setframerate(RATE)
    # wf.writeframes(b''.join(frames))
    # wf.close()

    data = {
        # "wav_filename": wav_filename,
        "request": msg.__str__(),
        "result": RESULT.__str__()
    }

    if RESULT.status.status == 3:
        json_filename = "%s/%s.json" % (STORAGE_FOLDER, now.strftime("%Y-%m-%d-%H-%M-%d-%f"))
        with open(json_filename, 'w') as outfile:
            json.dump(data, outfile)

        rospy.loginfo("Writing HMI log to %s" % json_filename)


def end(msg):
    global RECORDING, RESULT
    RESULT = msg
    rospy.loginfo("End callback")
    RECORDING = False


if __name__ == '__main__':
    rospy.init_node("hmi_logger")

    start_sub = rospy.Subscriber("goal", QueryActionGoal, start)
    end_sub = rospy.Subscriber("result", QueryActionResult, end)
    rospy.loginfo("Start topic: %s", start_sub.name)
    rospy.loginfo("End   topic: %s", end_sub.name)
    STORAGE_FOLDER = rospy.get_param('~storage_folder', os.path.expanduser('/tmp/hmi'))

    if not os.path.exists(STORAGE_FOLDER):
        os.makedirs(STORAGE_FOLDER)

    rospy.spin()

    # STREAM.stop_stream()
    # STREAM.close()
    # P.terminate()
