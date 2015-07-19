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

    object_names = ["deodorant", "bubblemint", "coke", "mints"]

    dx1 = 0.02
    # Kitchen counter (drinks)
    W.add_object("milk",  "mints", 2.25+(dx1*1), 2.0, 0.85, 0.0, 0.0 )
    W.add_object("milk2", "coke", 2.35+(dx1*2), 2.0, 0.85, 0.0, 0.0 )
    W.add_object("milk3", "mints", 2.45+(dx1*3), 2.0, 0.85, 0.0, 0.0 )
    W.add_object("milk4", "coke", 2.55+(dx1*4), 2.0, 0.85, 0.0, 0.0 )
    W.add_object("milk5", "mints", 2.65+(dx1*5), 2.0, 0.85, 0.0, 0.0 )

    #W.add_object("milk6", "coke", 2.30+(dx1*1), 1.75, 0.85, 0.0, 0.0 )
    #W.add_object("milk7", "mints", 2.40+(dx1*2), 1.75, 0.85, 0.0, 0.0 )
    #W.add_object("milk8", "coke", 2.50+(dx1*3), 1.75, 0.85, 0.0, 0.0 )
    #W.add_object("milk6", "mints", 2.60+(dx1*4), 1.75, 0.85, 0.0, 0.0 )

    # Kitchen table (food)
    dx = 0.03
    W.add_object("fruit1", "mints", 2.80+dx, -1.25, 0.85, 0.0, 0.0 )
    #W.add_object("fruit2", "coke", 2.85+dx*2, -1.05, 0.85, 0.0, 0.0 )
    W.add_object("fruit3", "coke", 2.90+dx*3, -1.25, 0.85, 0.0, 0.0 )
    #W.add_object("fruit4", "coke", 2.95+dx*4, -1.05, 0.85, 0.0, 0.0 )
    W.add_object("fruit5", "mints", 3.00+dx*5, -1.25, 0.85, 0.0, 0.0 )
    #W.add_object("fruit6", "coke", 3.05+dx*6, -1.05, 0.85, 0.0, 0.0 )
    W.add_object("fruit7", "coke", 3.10+dx*7, -1.25, 0.85, 0.0, 0.0 )
    #W.add_object("fruit8", "coke", 3.15+dx*8, -1.05, 0.85, 0.0, 0.0 )
    W.add_object("fruit9", "coke", 3.20+dx*9, -1.25, 0.85, 0.0, 0.0 )
    

    # W.add_object("coconut_cereals", "coconut_cereals",  3.20, -1.3, 0.8, 0.0, 0.0 )
    # W.add_object("apple", "apple",                      2.75, -1.4, 0.8, 0.0, 0.0 )
    # W.add_object("lemon", "lemon",                      2.90, -1.4, 0.8, 0.0, 0.0 )
    # W.add_object("pear", "pear",                        3.05, -1.4, 0.8, 0.0, 0.0 )

    print "Dynamic wake me up simulator"
    print "Usage: press 1 to make the person in the bed wake up"
    print "       press 2 to make the person move to another position (to test if it can see it anywhere on the bed)"

    while not rospy.is_shutdown():

        key = getKey()
        if key == '1':
            person.toggleAwake()
        elif key == '2':
            person.changePosition()
        else:
            break
