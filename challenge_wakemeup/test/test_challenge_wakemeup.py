#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Person(object):
    def __init__(self, W, x = 0, y = 0):
        self.id = id
        self.x = x
        self.y = y
        self.awake = False

        self.operator = W.add_object("operator", "ddw.chair", self.x, self.y, 0.0, 0.0, -3.0 )

    def toggleAwake(self):
        if self.awake:
            self.operator.set_position(self.x, self.y, -3.0, 0.0, 0.0, 0.0)
            self.awake = False
            print "Operator fell back asleep"
        else:
            self.operator.set_position(self.x, self.y, 0.3, 0.0, 0.0, 0.0)
            self.awake = True
            print "Operator woke up"


if __name__ == "__main__":
    rospy.init_node('wake_me_up_test_object_spawner')

    settings = termios.tcgetattr(sys.stdin)

    W = client.SimWorld()

    person = Person(W, -2.75, -1.0)

    print "Dynamic wake me up simulator"
    print "Usage: press 1 to make the person in the bed wake up"

    while not rospy.is_shutdown():

        key = getKey()
        if key == '1':
            person.toggleAwake()
        else:
            break
