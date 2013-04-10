#! /usr/bin/env python
import roslib; roslib.load_manifest('tue_execution_pack')
import rospy
import multiprocessing


class Worker(multiprocessing.Process):
 
    def __init__(self, func, **kwargs):
        
        # job management stuff
        self.kill_received = False
        self.func = func
        self.kwargs = kwargs
        self.queue = multiprocessing.Queue()
        # base class initialization
        multiprocessing.Process.__init__(self)
 
    def run(self):
        rospy.loginfo("Starting job")
        if self.kwargs == {}:
            self.queue.put(self.func())
        else:
            self.queue.put(self.func(**self.kwargs))
        rospy.loginfo("Finished")
        
    def get_result(self):
        if not self.empty():
            return self.queue.get()
        else:
            return None
    
    def empty(self):
        return self.queue.empty()
    
if __name__ == "__main__":
    print("Start worker test")
    
    
def a():
    return 1 + 1