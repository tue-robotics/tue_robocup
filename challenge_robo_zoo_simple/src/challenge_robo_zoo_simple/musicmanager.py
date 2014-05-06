from contextlib import contextmanager
import subprocess
import os
import signal

def start_music(filepath):
    print "Playing music: {0}".format(filepath)
    # The os.setsid() is passed in the argument preexec_fn so
    # it's run after the fork() and before  exec() to run the shell.
    music_process = subprocess.Popen("mpg123 '{0}'".format(filepath), stdout=subprocess.PIPE, 
                           shell=True, preexec_fn=os.setsid) 
    print "If the music keeps going somehow, its PID is: {0}".format(music_process.pid)
    return music_process

def stop_music(music_process):
    os.killpg(music_process.pid, signal.SIGTERM)  # Send the signal to all the process groups

@contextmanager
def music(filename):
    pid = start_music(filename)
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