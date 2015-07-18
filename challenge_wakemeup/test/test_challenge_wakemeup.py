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
    def __init__(self, W, positions):
        self.id = id
        self.positions = positions
        self.position = 0
        self.x = self.positions[self.position][0]
        self.y = self.positions[self.position][1]
        self.awake = False
        self.height = -3.0

        self.operator = W.add_object("operator", "ddw.chair", self.x, self.y, self.height, 0.0, 0.0 )

    def updatePosition(self):
        self.operator.set_position(self.x, self.y, self.height, 0.0, 0.0, 0.0 )

    def toggleAwake(self):
        if self.awake:
            self.height = -3.0
            self.awake = False
            self.updatePosition()
            print "Operator fell back asleep"
        else:
            self.height = 0.0
            self.awake = True
            self.updatePosition()
            print "Operator woke up"

    def changePosition(self):
        self.position += 1
        if self.position < len(self.positions):
            pass
        else:
            self.position = 0
        self.x = self.positions[self.position][0]
        self.y = self.positions[self.position][1]
        self.updatePosition()
        print "Operator changed to position {}".format(self.position)


if __name__ == "__main__":
    rospy.init_node('wake_me_up_test_object_spawner')

    settings = termios.tcgetattr(sys.stdin)

    W = client.SimWorld()

    possible_positions = [[11.0, -7.8], [11.0, -8.5]]

    person = Person(W, possible_positions)

    print "Dynamic wake me up simulator"
    print "Usage: press 1 to make the person in the bed wake up"

    while not rospy.is_shutdown():

        key = getKey()
        if key == '1':
            person.toggleAwake()
        elif key == '2':
            person.changePosition()
        else:
            break
