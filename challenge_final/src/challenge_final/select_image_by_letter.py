import sys
import cv2

# ROS
import rospy
import smach
import cv_bridge
from sensor_msgs.msg import Image

# Robot skills
from robot_smach_states import WaitTime
from robot_smach_states.util.designators import EdEntityDesignator
import robot_smach_states as states


class SelectImageByLetter(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,
                             outcomes=['selected', 'failed'],
                             input_keys=['label2image'],
                             output_keys=['selected_label'])

    def execute(self, ud):
        # Get a dict {str: Image}
        # Publish these to telegram:
        #   image 1
        #   text 1
        #   image 2
        #   text 2
        # Please select a picture by it's label
        #  Receive the desired label
        # Return the label to the next state

        return 'failed'

if __name__ == '__main__':
    from robot_skills import get_robot

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]

        rospy.init_node('test_find_person_in_room')
        _robot = get_robot(robot_name)

        bridge = cv_bridge.CvBridge()

        sm = SelectImageByLetter(_robot)

        image_paths = sys.argv[1:]
        labels = ['a', 'b', 'c', 'd', 'e']

        ud = {'label2image': {}}
        for label, image_path in zip(labels, image_paths):
            cv_image = cv2.imread(image_path)

            ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
            ud['label2image'][label] = ros_image

        sm.execute(ud)

        _robot.speech.speak(ud['selected_label'])
    else:
        print "Please provide robot name as argument."
        exit(1)


