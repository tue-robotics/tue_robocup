#! /usr/bin/env python

# ROS
import rospy

# Body parts
import base
import torso
import arms
import head
import perception
import ssl

# Human Robot Interaction
import speech
import api
import ears
import ebutton
import lights

# tf
import tf

# Reasoning/world modeling
import world_model_ed

# Misc: do we need this???
import geometry_msgs
from collections import OrderedDict

# Check hardware status
from diagnostic_msgs.msg import DiagnosticArray


class Robot(object):
    """
    Interface to all parts of the robot.
    """
    def __init__(self, robot_name="", wait_services=False):

        self.robot_name = robot_name
        self.tf_listener = tf.TransformListener()

        self.parts = dict()

        # Reasoning/world modeling
        self.parts['ed'] = world_model_ed.ED(self.robot_name, self.tf_listener)

        # Body parts
        self.parts['base'] = base.Base(self.robot_name, self.tf_listener)
        self.parts['torso'] = torso.Torso(self.robot_name, self.tf_listener)

        self.parts['leftArm'] = arms.Arm(self.robot_name, self.tf_listener, side="left", world_model=self.parts['ed'])
        self.parts['rightArm'] = arms.Arm(self.robot_name, self.tf_listener, side="right", world_model=self.parts['ed'])

        self.parts['head'] = head.Head(self.robot_name, self.tf_listener)
        self.parts['perception'] = perception.Perception(self.robot_name, self.tf_listener)
        self.parts['ssl'] = ssl.SSL(self.robot_name, self.tf_listener)

        # Human Robot Interaction
        self.parts['lights'] = lights.Lights(self.robot_name, self.tf_listener)
        self.parts['speech'] = speech.Speech(self.robot_name, self.tf_listener,
                                             lambda: self.lights.set_color_colorRGBA(lights.SPEAKING),
                                             lambda: self.lights.set_color_colorRGBA(lights.RESET))
        self.parts['hmi'] = api.Api(self.robot_name, self.tf_listener,
                                    lambda: self.lights.set_color_colorRGBA(lights.LISTENING),
                                    lambda: self.lights.set_color_colorRGBA(lights.RESET))
        self.parts['ears'] = ears.Ears(self.robot_name, self.tf_listener,
                                       lambda: self.lights.set_color_colorRGBA(lights.LISTENING),
                                       lambda: self.lights.set_color_colorRGBA(lights.RESET))

        self.parts['ebutton'] = ebutton.EButton(self.robot_name, self.tf_listener)

        # Ignore diagnostics: parts that are not present in the real robot
        self._ignored_parts = []

        # Miscellaneous
        self.pub_target = rospy.Publisher("/target_location", geometry_msgs.msg.Pose2D, queue_size=10)
        self.base_link_frame = "/"+self.robot_name+"/base_link"

        # Check hardware status
        self._hardware_status_sub = rospy.Subscriber("/" + self.robot_name + "/hardware_status", DiagnosticArray, self.handle_hardware_status)

        # Grasp offsets
        #TODO: Don't hardcode, load from parameter server to make robot independent.
        self.grasp_offset = geometry_msgs.msg.Point(0.5, 0.2, 0.0)

        # Create attributes from dict
        for partname, bodypart in self.parts.iteritems():
            setattr(self, partname, bodypart)
        self.arms = OrderedDict(left=self.leftArm, right=self.rightArm)  # (ToDo: kind of ugly, why do we need this???)
        self.ears._hmi = self.hmi  # ToDo: when ears is gone, remove this line

        # Wait for connections
        s = rospy.Time.now()
        for partname, bodypart in self.parts.iteritems():
            bodypart.wait_for_connections(1.0)
        e = rospy.Time.now()
        rospy.logdebug("Connecting took {} seconds".format((e-s).to_sec()))

        if not self.operational:
            not_operational_parts = [name for name, part in self.parts.iteritems() if not part.operational]
            rospy.logwarn("Not all hardware operational: {parts}".format(parts=not_operational_parts))

    def standby(self):
        if not self.robot_name == 'amigo':
            rospy.logerr('Standby only works for amigo')
            return
        self.leftArm.reset()
        self.rightArm.reset()
        self.leftArm.send_gripper_goal('close')
        self.rightArm.send_gripper_goal('close')
        self.head.look_down()
        self.torso.low()
        self.lights.set_color(0, 0, 0)

    def publish_target(self, x, y):
        self.pub_target.publish(geometry_msgs.msg.Pose2D(x, y, 0))

    def tf_transform_pose(self, ps, frame):
        output_pose = geometry_msgs.msg.PointStamped
        self.tf_listener.waitForTransform(frame, ps.header.frame_id, rospy.Time(), rospy.Duration(2.0))
        output_pose = self.tf_listener.transformPose(frame, ps)
        return output_pose

    def get_arm(self, side):
        """Get an arm object and a backup for that arm by giving a side as either a string or an Arm-object
        @param side Either string from robot.arms.keys() or Arm from robot.arms.values()
        >>> robot = Robot("dummy")
        >>> arm, backup_arm = robot.get_arm("left")
        >>> assert(arm == robot.leftArm)
        >>> assert(backup_arm == robot.rightArm)"""
        preferred_side = self.arms[self.arms.keys()[0]]

        #Define which arm is which's backup arm (left backs up for right etc)
        backup_arms = self.arms.values() #Get a *list* of arms i.e. the values of the arm-dict, not the keys
        backup_arms.insert(0, backup_arms.pop()) #Make the last arm the first in the list, so we shift by 1
        backup_str_dict = dict(zip(self.arms.keys(), backup_arms)) #Create a dict again that maps strings to backup-arms
        backup_obj_dict = {self.arms[side]:backup_str_dict[side] for side in self.arms.keys()} #Create a dict that maps e.g. self.LeftArm to self.rightArm

        if isinstance(side, basestring):
            try:
                preferred_side = self.arms[side]
            except KeyError:
                print "Unknown arm side:" + str(side) + ". Defaulting to 'right'"
                preferred_side = self.arms[self.arms.keys()[0]]
        elif isinstance(side, arms.Arm):
            preferred_side = side
        else:
            print "Unknown arm side:" + str(side) + ". Defaulting to '{0}'".format(preferred_side.side)

        backup_side = backup_obj_dict[preferred_side]
        return preferred_side, backup_side

    def close(self):
        try:
            self.head.close()
        except: pass

        try:
            self.base.close()
        except: pass

        try:
            self.torso.close()
        except: pass

        try:
            self.speech.close()
        except: pass

        try:
            self.arms.close()
        except: pass

        try:
            self.leftArm.close()
        except: pass

        try:
            self.rightArm.close()
        except: pass

        try:
            self.ears.close()
        except: pass

        try:
            self.ebutton.close()
        except: pass

        try:
            self.lights.close()
        except: pass

    @property
    def operational(self):
        """
        :returns if all parts are operational"""
        return all(bodypart.operational for bodypart in self.parts.values())

    def handle_hardware_status(self, diagnostic_array):
        """
        hardware_status callback to determine if the bodypart is operational
        :param msg: diagnostic_msgs.msg.DiagnosticArray
        :return: no return
        """

        diagnostic_dict = {diagnostic_status.name:diagnostic_status for diagnostic_status in diagnostic_array.status}

        for name, part in self.parts.iteritems():
            # Pass a dict mapping the name to the item.
            # Bodypart.handle_hardware_status needs to find the element relevant to itself
            # iterating over the array would be done be each bodypart, but with a dict they can just look theirs up.
            if name not in self._ignored_parts:
                part.process_hardware_status(diagnostic_dict)

    def __enter__(self):
        pass

    def __exit__(self, exception_type, exception_val, trace):
        if any((exception_type, exception_val, trace)):
            rospy.logerr("Robot exited with {0},{1},{2}".format(exception_type, exception_val, trace))
        self.close()


if __name__ == "__main__":
    rospy.init_node("robot")

    import doctest
    doctest.testmod()
