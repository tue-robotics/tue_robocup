from contextlib import contextmanager
import threading

@contextmanager
def iterate_in_background(func, *funcarg, **funckwargs):
    stopEvent = threading.Event()
    def iterate():
        while not stopEvent.is_set():
            func(*funcarg, **funckwargs)
    iterate_thread = threading.Thread(target=iterate)
    iterate_thread.start()
    yield
    stopEvent.set()
    iterate_thread.join()


# def spindle_up_down(robot, lower, upper, stopEvent):
#     """Loop the robot's spindle between the lower and upper heights given here"""
#     while not rospy.is_shutdown() and not stopEvent.is_set():
#         robot.spindle.send_goal(lower, timeout=4.0)
#         robot.spindle.send_goal(upper, timeout=4.0)

# stopEvent = threading.Event()

# up_and_down_spindle = threading.Thread(target=spindle_up_down, args=(robot, 0.3, 0.4, stopEvent))

# stopEvent.set()
# up_and_down_spindle.join()
if __name__ == "__main__":
    import time

    def do_in_background():
        print "Hello"

    with iterate_in_background(do_in_background):
        time.sleep(0.1)