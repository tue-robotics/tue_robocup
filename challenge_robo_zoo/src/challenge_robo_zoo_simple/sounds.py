#! /usr/bin/env python
import smach
import rospy
import os

class R2D2(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])

    def execute(self, userdata=None):
        
        try:
            os.system("mpg123 ~/Music/R2D2.mp3")
        except:
            rospy.logerror("Something went terribly wrong when playing r2d2")        
        
        return "Done"
        
class Toeter(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["Done"])

    def execute(self, userdata=None):
        
        try:
            os.system("mpg123 ~/Music/Toeter1.mp3")
        except:
            rospy.logerror("Something went terribly wrong when playing Toeter1 ")        
        
        return "Done"
