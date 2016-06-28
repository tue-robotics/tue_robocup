#!/usr/bin/python

import rospy

from datetime import datetime
import os

import pyaudio
import wave
import json
import time

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100

P = pyaudio.PyAudio()

RECORDING = False
STREAM = P.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)


def start():
    global RECORDING

    if RECORDING:
        return

    frames = []

    print("Starting HMI recording")
    RECORDING = True

    last_print = time.time()
    while RECORDING:
        data = STREAM.read(CHUNK)
        frames.append(data)
        if time.time() - last_print > 1.0:
            print(".. recording")
            last_print = time.time()

    now = datetime.now()
    wav_filename = "%s/%s.wav" % (STORAGE_FOLDER, now.strftime("%Y-%m-%d-%H-%M-%d-%f"))
    wf = wave.open(wav_filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(P.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    data = {
        "wav_filename": wav_filename,
    }

    json_filename = "%s/%s.json" % (STORAGE_FOLDER, now.strftime("%Y-%m-%d-%H-%M-%d-%f"))
    with open(json_filename, 'w') as outfile:
        json.dump(data, outfile)

    print("Writing HMI log to %s" % json_filename)


def end():
    global RECORDING
    RECORDING = False


if __name__ == '__main__':
    STORAGE_FOLDER = os.path.expanduser('/tmp/hmi')

    if not os.path.exists(STORAGE_FOLDER):
        os.makedirs(STORAGE_FOLDER)

    start()
    time.sleep(10)
    end()

    STREAM.stop_stream()
    STREAM.close()
    P.terminate()
