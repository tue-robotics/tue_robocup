#! /usr/bin/env python
import roslib; roslib.load_manifest('tue_execution_pack')
from collections import deque
import rospy
import amigo_msgs.msg
import actionlib
from std_msgs.msg import String

import threading #used for locking fields edited by the subscriber of Ears

#maybe put this in utils
def synchronized(L): 
    def lock_around(f): 
        def locked(*a, **k): 
            with L: 
                return f(*a, **k) 
        locked.__name__ = f.__name__ 
        locked.__doc__ = f.__doc__ 
        return locked 
    return lock_around

## TODO: some words are appended like 'applejuice', this should be handled gracefully in some way
## by the executive
class Ears:
    """
    Interface to amigo Ears. Listens to /speech/output topic what it has heard the in the last time.
    Works as a Queue (LIFO), that can hold up to 30 entries.
    >>> entries = 50
    >>> ears = Ears(50) #queue up to 50 elements
    """
    _lock = threading.RLock() #Reentrant lock. Only the thread that acquired the lock can unlock it again.
    
    def __init__(self,entries=30):
        
        self.queue = deque([],entries)
        self.__entries = entries
        self._topic = None
        self.is_listening = False
        self._topic = rospy.Subscriber("/speech/output", 
                                         String, 
                                         self._listen)

    def close(self):
        pass
    
    @synchronized(_lock)    
    def _listen(self, s):
        """
        Callback methods that listens to STT
        """
        rospy.loginfo("Heard: '{0}'".format(s.data))
        #import ipdb; ipdb.set_trace()

        if self.is_listening:
            if s.data == '': return
            s.data.strip()
            wordlist = s.data.split(' ')
            wordlist.reverse()
            for string in wordlist:
                self.queue.appendleft((string, rospy.get_time()))
            rospy.loginfo("{0}".format(self.queue))
            #rospy.loginfo("[Ears] Heard {0}".format(s.data.lower()))
        
    def start_listening(self):
        '''Toggle recording of recognized words ON'''
        rospy.loginfo("[Ears] Starting to listen...")
        self.is_listening = True
        
    def stop_listening(self):
        '''Toggle recording of recognized words OFF'''
        self.is_listening = False
        self.forget()
        rospy.loginfo("[Ears] listening stopped...")
    
    def heard(self, query, duration=0):
        """
        Checks if a query is heard in a given duration.
        >>> ears.heard('Bla')
        True
        >>> ears.heard('Bla',1)
        False 
        """
        words = self.get_words_for_duration(duration)
        
        return query.lower() in [x for x,_ in words]
    
    def last_heard_words(self, words, duration):
        wordlist = self.get_words_for_duration(duration)
        rospy.logdebug("len(         words_for_duration({0}) = {1}".format(duration, len(wordlist)))
        filtered_wordlist = [x for x in wordlist if x[0] in words]
        rospy.logdebug("len(filtered words_for_duration({0}) = {1}".format(duration, len(filtered_wordlist)))
        if len(filtered_wordlist) > 0:
            filtered_wordlist.sort(cmp= lambda a,b: cmp(a[1], b[1]))
            return filtered_wordlist[0][0]
        elif len(filtered_wordlist) == 0 and len(wordlist) != 0:
            rospy.logerr("Some words were heard, but were not an option: {0}".format(wordlist))
        else:
            return None
        
    def get_words_for_duration(self, duration):
        """
        Get words for a specific duration. If duration is 0 the entire queue is used
        >>> ears.get_words_for_duration(100)
        [('Bla2', 1335207493.973912), ('Bla', 1335207491.5919981)]

        """
        #print self.queue
        if duration == 0:
            return self.queue
        else:
            return [item for item in self.queue if(self._less_than(item,duration)) ]
        
    def _less_than(self, item, duration):
        """
        Checks if a list item is less than a specific duration. 
        Items are saved as tuples ('Word', 1234.343) #word and rospy.get_time()
        """ 
        _, time = item
        if (rospy.get_time() - time) < duration:
            return True
        else:
            return False
        
    def __repr__(self):
        """
        Simple representation
        """
        return "%r" % self.queue
    
    @synchronized(_lock)
    def pop(self):
        """
        Pop top item.
        """
        return self.queue.popleft()
    
    def peek(self):
        """
        Peek at top item.
        """
        if len(self.queue) > 0:
            return self.queue[0]
        else:
            return None
    
    def last_heard(self):
        """
        When was the last time a word was heard. None is never.
        """
        if len(self.queue) > 0:
            return rospy.get_time() - self.queue[0][1]
        else:
            return None
    
    @synchronized(_lock)     
    def forget(self, upto=0):
        """
        Forget words heard up to a certain number of seconds.
        upto is the number of seconds that the system should forget.
        >>> ears.forget(10) #remember everything from 10 seconds ago and later
        """
        rospy.loginfo("Forgetting words I heard upto {0} (0 means everything)".format(upto))
        if upto == 0:
            self.queue = deque([], self.__entries)
        else:
            self.queue = deque(self.get_words_for_duration(upto), self.__entries)
    
    @synchronized(_lock)         
    def forget_since(self, time):
        """
        See forget. Is the same except a rospy.get_time() may be given instead of a relative time.
        >>> time = rospy.get_time()
        #Do some stuff 
        >>> ears.forgetSince(time)
        """
        self.forget(rospy.get_time() - time)
         
if __name__ == "__main__":
    rospy.init_node('amigo_head_executioner', anonymous=True)
    ears = Ears()