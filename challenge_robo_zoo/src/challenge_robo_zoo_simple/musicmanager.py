from contextlib import contextmanager
import subprocess
import os
import signal

def start_music(filepath, volume=100):
    print "Playing music: {0}".format(filepath)
    # The os.setsid() is passed in the argument preexec_fn so
    # it's run after the fork() and before  exec() to run the shell.
    max_volume = 32768
    current_volume = int((max_volume / 100) * volume)

    music_process = subprocess.Popen("mpg123 --scale {1} '{0}'".format(filepath, current_volume), stdout=subprocess.PIPE, 
                           shell=True, preexec_fn=os.setsid) 
    print "If the music keeps going somehow, its PID is: {0}".format(music_process.pid)
    return music_process

def stop_music(music_process):
    os.killpg(music_process.pid, signal.SIGTERM)  # Send the signal to all the process groups

@contextmanager
def music(filename, volume=100):
    pid = start_music(filename, volume)
    yield
    stop_music(pid)

if __name__ == "__main__":
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    musicfile = "macarena.mp3"
    musicfile = os.path.join(dname, musicfile)

    with music(musicfile):
        import time
        time.sleep(10)