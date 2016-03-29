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
    def __init__(self, W, position):
        self.id = id
        self.position = position
        self.x = self.position[0]
        self.y = self.position[1]
        self.z = -3.0
        self.awake = False
        self.delta = 0.05

        self.operator = W.add_object("operator", "ddw.chair", self.x, self.y, self.z, 0.0, 0.0 )

    def updatePosition(self):
        self.operator.set_position(self.x, self.y, self.z, 0.0, 0.0, 0.0 )

    def toggleAwake(self):
        if self.awake:
            self.z = -3.0
            self.awake = False
            self.updatePosition()
            print "Operator fell back asleep"
        else:
            self.z = 0.0
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

    def move(self, key):
        dx = 0
        dy = 0
        dz = 0

        if key == '8':
            dx = 0
            dy = 1
        elif key == '2':
            dx = 0
            dy = -1
        elif key == '4':
            dx = -1
            dy = 0
        elif key == '6':
            dx = 1
            dy = 0
        elif key == '9':
            dx = 1
            dy = 1
        elif key == '3':
            dx = 1
            dy = -1
        elif key == '1':
            dx = -1
            dy = -1
        elif key == '7':
            dx = -1
            dy = 1
        elif key == '+':
            dz = 1
        elif key == '-':
            dz = -1
        else:
            print "Don't do that"

        self.x = self.x + dx * self.delta
        self.y = self.y + dy * self.delta
        self.z = self.z + dz * self.delta
        self.updatePosition()


if __name__ == "__main__":
    rospy.init_node('wake_me_up_test_object_spawner')

    settings = termios.tcgetattr(sys.stdin)

    W = client.SimWorld()

    initial_position = [0.0,0.0]

    person = Person(W, initial_position)

    # object_names = ["deodorant", "bubblemint", "coke", "mints"]

    # dx1 = 0.02
    # # Kitchen counter (food)
    # W.add_object("apple",  "sim-apple", 2.25+(dx1*1), 2.0, 0.85, 0.0, 0.0 )
    # W.add_object("lemon", "sim-lemon", 2.35+(dx1*2), 2.0, 0.85, 0.0, 0.0 )
    # W.add_object("pear", "sim-pear", 2.45+(dx1*3), 2.0, 0.85, 0.0, 0.0 )
    # W.add_object("coconut_cereals", "sim-coconut_cereals", 2.55+(dx1*4), 2.0, 0.85, 0.0, 0.0 )
    # W.add_object("coco_balls", "sim-coco_balls", 2.65+(dx1*5), 2.0, 0.85, 0.0, 0.0 )
    # W.add_object("egg_stars", "sim-egg_stars", 2.75+(dx1*6), 2.0, 0.85, 0.0, 0.0 )

    # # Kitchen table (drinks)
    # dx = 0.03
    # W.add_object("pure_milk", "sim-pure_milk", 2.80+dx, -1.25, 0.85, 0.0, 0.0 )
    # #W.add_object("fruit2", "coke", 2.85+dx*2, -1.05, 0.85, 0.0, 0.0 )
    # W.add_object("papaya_milk", "sim-papaya_milk", 2.90+dx*3, -1.25, 0.85, 0.0, 0.0 )
    # #W.add_object("fruit4", "coke", 2.95+dx*4, -1.05, 0.85, 0.0, 0.0 )
    # W.add_object("pure_milk", "sim-pure_milk", 3.00+dx*5, -1.25, 0.85, 0.0, 0.0 )
    # #W.add_object("fruit6", "coke", 3.05+dx*6, -1.05, 0.85, 0.0, 0.0 )
    # W.add_object("papaya_milk", "sim-papaya_milk", 3.10+dx*7, -1.25, 0.85, 0.0, 0.0 )
    # #W.add_object("fruit8", "coke", 3.15+dx*8, -1.05, 0.85, 0.0, 0.0 )
    # W.add_object("papaya_milk", "sim-papaya_milk", 3.20+dx*9, -1.25, 0.85, 0.0, 0.0 )
    

    print "Dynamic wake me up simulator"
    print "Usage: press 5 to make the person in the bed wake up"
    print "       use the numpad keys to move the thing around"

    while not rospy.is_shutdown():

        key = getKey()
        if key == '5':
            person.toggleAwake()
        elif key == '0':
            person.changePosition()
        elif key in ['1','2','3','4','6','7','8','9','+','-']:
            person.move(key)
        else:
            break
