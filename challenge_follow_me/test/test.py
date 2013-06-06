#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

# For reading key press:
import sys, select, termios, tty

from fast_simulator import client

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__ == "__main__":
    rospy.init_node('open_challenge_rgo2013_tester')

    settings = termios.tcgetattr(sys.stdin)

    W = client.SimWorld()

    op_x = 1
    op_y = 0

    op = W.add_object("operator", "person", op_x, op_y, 0)

    while not rospy.is_shutdown():
        key = getKey()
        if key == '8':
            op_x += 0.1
        elif key == '2':
            op_x -= 0.1
        elif key == '4':
            op_y += 0.1
        elif key == '6':
            op_y -= 0.1
        else:
            break

        op.set_position(op_x, op_y, 0)


        
