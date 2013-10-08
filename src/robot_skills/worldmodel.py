#! /usr/bin/env python

import roslib; roslib.load_manifest('robot_skills')
import rospy
import threading

import util
from std_srvs.srv import Empty 
import geometry_msgs.msg

def synchronized(L): 
         def lock_around(f): 
             def locked(*a, **k): 
                 with L: 
                     return f(*a, **k) 
             locked.__name__ = f.__name__ 
             locked.__doc__ = f.__doc__ 
             return locked 
         return lock_around

@util.deprecated
class WorldModel(object):
    _lock = threading.RLock() #Reentrant lock. Only the thread that acquired the lock can unlock it again.
    
    
    def __init__(self, tf_listener, debug=False):    

        self.tf_listener = tf_listener
        
        self.sv_world_model_reset = rospy.ServiceProxy("/wire/reset",Empty)
        
        self._logging_ID = None
        
        self.debug = debug
        self._known_IDs = set()

    def close(self):
        pass
            
    def reset(self):
        try:
            self.sv_world_model_reset()
            return True
        except rospy.ServiceException, e:
            rospy.logerr(e)
            return False
        
  

    def tf_transform(self, coordinates, inputframe, outputframe):
        ps = geometry_msgs.msg.PointStamped(point= coordinates) 
        ps.header.frame_id = inputframe
        ps.header.stamp = rospy.Time()
        output_coordinates = self.tf_listener.transformPoint(outputframe, ps)
        return output_coordinates.point
    
    #TODO FIX IF POSITION IS NONE
    def repr_obj(self, obj):
        try:
            return "ID: {id}, xyz=({x:.2f}, {y:.2f}, {z:.2f}), name: {name}, class: {clas}".format(
                                                       id=obj.ID, #:.2f makes it truncate to 2 decimal places
                                                       x=obj.pose.position.x,
                                                       y=obj.pose.position.y,
                                                       z=obj.pose.position.z,
                                                       name=obj.name,
                                                       clas=obj.class_label)
        except AttributeError: #in case some attribute does not exist:
            return str(obj)


if __name__ == "__main__":
    rospy.init_node("amigo_worldmodel_executioner", anonymous=True)
    
    import tf
    tf_listener = tf.TransformListener()
    
    model = WorldModel(tf_listener, debug=True)
    
    print "WorldModel created, this will update as new information comes in."
    
    #rospy.spin()
