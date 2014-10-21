#! /usr/bin/env python
import perception

class PerceptionED(arms.Arms):
    def __init__(self):
        super(PerceptionED, self).__init__()

    def close(self):
        pass

    def toggle(self, modules):
    	rospy.logwarn("Assuming we're using ED and not toggling anything")
    	return True

    def toggle_always_on(self, modules):
    	rospy.logwarn("Assuming we're using ED and not toggling anything")
        return True

    def toggle_always_off(self, modules):
    	rospy.logwarn("Assuming we're using ED and not toggling anything")
        return True

    def toggle_everything_off(self):
    	rospy.logwarn("Assuming we're using ED and not toggling anything")
        return True

    def toggle_recognition(self, faces=False, objects=False, people=False):
    	rospy.logwarn("Assuming we're using ED and not toggling anything")
        return True

    def toggle_perception_2d(self, pointstamped, length_x=0.5, length_y=0.5, length_z=0.5):
    	rospy.logwarn("Assuming we're using ED and not toggling anything")
    	return True

    def set_perception_roi(self, pointstamped, length_x=0.5, length_y=0.5, length_z=0.5):
    	rospy.logwarn("Assuming we're using ED and not setting anything")
        return True

    def set_table_roi(self, pointstamped, length_x=0.5, length_y=0.5, length_z=0.5):
    	rospy.logwarn("Assuming we're using ED and not setting anything")
        return True

    def toggle_bin_detection(self, pointstamped, length_x=3.0, length_y=3.0, length_z=1.0):
    	rospy.logwarn("Assuming we're using ED and not toggling anything")
    	return True

    def load_template_matching_config(self, config_name, status='inactive'):
    	rospy.logwarn("Assuming we're using ED and not loading anything")
        return True

    def learn_person(self, name, n_models = 10, view = 'front', publish_while_learning = False):
    	rospy.logwarn("Assuming we're using ED and not learning anything")
        return True

    '''Cancel face learning'''
    def cancel_learn_persons(self):
    	rospy.logwarn("Assuming we're using ED and not canceling anything")
        return True

    '''Face stuff'''
    def get_learn_face_counter(self):
    	rospy.logwarn("Assuming we're using ED and not counting anything")
        return 0

    def rec_start(self, source, context, duration, freq):
        msg = tue_recorder.msg.Start()
        msg.source = source
        msg.context = context
        msg.max_duration = rospy.Duration(duration)
        msg.frequency = freq
        
        self.pub_rec.publish(msg)        


    def people_detection_torso_laser(self, pointstamped, time=4.0, length_x=3.0, length_y=3.0, length_z=1.0):
    	rospy.logwarn("Assuming we're using ED and not toggling anything")
    	return True