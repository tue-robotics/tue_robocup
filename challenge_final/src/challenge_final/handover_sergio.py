#!/usr/bin/python

import math

import rospy
import geometry_msgs.msg
import PyKDL as kdl

from robot_skills.sergio import Sergio
from robot_smach_states.navigation.navigate_to_pose import NavigateToPose
from action_result import ActionResult

TESTMODE = True

ENTITY_FRAME_ID = "/bar"
X_OFFSET = 3.0  # Distance between the center point of the bar and the pose where SERGIO will stand

TRANS_ERROR_CONSTRAINT = 0.05  # We're satisfied if we're within 5 cm in each direction
ROT_ERROR_CONSTRAINT = 10.0/180.0 * math.pi  # and within 10 degrees

MAX_TRANS_ERROR = 0.5  # If we're further than 1.5 meters apart, better skip it
MAX_ROT_ERROR = 30.0/180.0 * math.pi  # or if the angle offset is more than 30 degrees

# Controller parameters
TRANS_GAIN = 0.2
ROT_GAIN = 0.15
MAX_TRANS_VEL = 0.5
# MIN_TRANS_VEL = 0.05
MAX_ROT_VEL = 1.0
# MIN_ROT_VEL = 0.1

OBJ_OFFSET_X = 0.0425  # Desired x-pos of the object in base link frame
OBJ_OFFSET_Y = -0.14  # Desired y-pos of the object in base link frame


def move_sergio_to_pre_handover_pose(sergio, entity_frame_id):
    """ Makes SERGIO navigate to the pre handover pose """
    nav_state = NavigateToPose(sergio, x=X_OFFSET, y=0.0, angle_offset=math.pi, frame_id=entity_frame_id)
    nav_state.execute()
    if result == 'arrived':
        return ActionResult(ActionResult.SUCCEEDED, 'SERGIO reached pre handover pose')
    else:
        return ActionResult(ActionResult.FAILED, 'SERGIO failed to reach the pre-handover pose, goal {0}'.format(result))


def move_sergio_to_handover_pose(sergio, x_gripper, y_gripper, yaw_gripper):
    """ Makes SERGIO force drive until its tray is underneath AMIGO's arm
     :param sergio: robot object
     :param x: x coordinate of amigo gripper in map frame
     :param y: y coordinate of amigo gripper in map frame
     :param yaw
     :param
     """
    # if TESTMODE:
    #     source_frame = "/map"
    # else:
    #     source_frame = "/amigo/grippoint_{0}".format(side)

    yaw_ref = yaw_gripper + math.pi  # SERGIO should be opposite to the AMIGO gripper
    gripper_pose_map = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw_ref),
                                 kdl.Vector(x_gripper, y_gripper, 0))  # Note: using yaw ref instead of yaw gripper
    # implies that we have already rotated 180 degrees

    rate = rospy.Rate(20.0)

    while True and not rospy.is_shutdown():

        # # Check the current offset
        # try:
        #     t, r = sergio.tf_listener.lookupTransform("sergio/base_link", source_frame)
        # (x, y, z), (rx, ry, rz, rw) =
        # except:
        #     print "TF error"
        #     return ActionResult(ActionResult.FAILED, "TF Lookup failed")
        #
        # trans = kdl.Vector(t[0], t[1], t[2])
        # rot = kdl.Rotation.Quaternion(r[0], r[1], r[2], r[3])
        # (roll, pitch, yaw) = rot.GetRPY()

        base_pose_map_msg = sergio.base.get_location()  #
        base_pose_map = kdl.Frame(kdl.Rotation.Quaternion(base_pose_map_msg.pose.orientation.x,
                                                          base_pose_map_msg.pose.orientation.y,
                                                          base_pose_map_msg.pose.orientation.z,
                                                          base_pose_map_msg.pose.orientation.w),
                                  kdl.Vector(base_pose_map_msg.pose.position.x,
                                             base_pose_map_msg.pose.position.y,
                                             base_pose_map_msg.pose.position.z))

        # error = kdl.diff(gripper_pose_map, base_pose_map)
        # trans = error.vel
        # rot = error.rot.z()

        ref_pose_robot = base_pose_map.Inverse() * gripper_pose_map
        ref_pose_robot.pose.x(ref_pose_robot.p.x() - OBJ_OFFSET_X)
        ref_pose_robot.pose.y(ref_pose_robot.p.y() - OBJ_OFFSET_Y)
        trans = ref_pose_robot.p
        (roll, pitch, rot) = ref_pose_robot.M.GetRPY()
        # import ipdb;ipdb.set_trace()

        # If within goal constraints, return succeeded
        if abs(trans.x()) < TRANS_ERROR_CONSTRAINT and \
                abs(trans.y()) < TRANS_ERROR_CONSTRAINT and \
                abs(rot) < ROT_ERROR_CONSTRAINT:
            return ActionResult(ActionResult.SUCCEEDED, "SERGIO is in position for place action")

        # If error too large, return failed (don't want to force drive too far
        if abs(trans.x()) > MAX_TRANS_ERROR or \
                abs(trans.y()) > MAX_TRANS_ERROR or \
                abs(rot) > MAX_ROT_ERROR:
            return ActionResult(ActionResult.FAILED, "SERGIO and AMIGO are too far apart for safe handover")

        # Else, control action
        vx = TRANS_GAIN * trans.x()
        vy = TRANS_GAIN * trans.y()
        vth = ROT_GAIN * rot

        v = geometry_msgs.msg.Twist()  # Initialize velocity
        v.linear.x = max(-MAX_TRANS_VEL, min(MAX_TRANS_VEL, vx))
        v.linear.y = max(-MAX_TRANS_VEL, min(MAX_TRANS_VEL, vy))
        v.angular.x = max(-MAX_ROT_VEL, min(MAX_ROT_VEL, vth))

        # import ipdb;        ipdb.set_trace()

        sergio.base._cmd_vel.publish(v)

        rate.sleep()


def move_sergio_back(sergio):
    """ Makes SERGIO forcedrive backwards until it is out of the way """
    # ToDo: as an additional measure of robustness, we could check if SERGIO is really out of the way...
    sergio.base.force_drive(-0.1, 0, 0, 3.0)
    return ActionResult(ActionResult.SUCCEEDED, "SERGIO is out of the way")


if __name__ == "__main__":

    """ Test stuff """
    rospy.init_node("Test handover motions")
    sergio = Sergio(wait_services=True)

    """ Testing in simulation """
    x_gripper = float(raw_input("Enter the x coordinate of the gripper: "))
    y_gripper = float(raw_input("Enter the y coordinate of the gripper: "))
    yaw_gripper = float(raw_input("Enter the yaw of the gripper: "))
    result = move_sergio_to_handover_pose(sergio, x_gripper=x_gripper, y_gripper=y_gripper, yaw_gripper=yaw_gripper)
    rospy.loginfo("{0}".format(result))

    # """ Testing for real """
    # rospy.loginfo("SERGIO is loaded and will move to the pre-handover pose")
    # result = move_sergio_to_pre_handover_pose(sergio, ENTITY_FRAME_ID)
    # rospy.loginfo("{0}".format(result))
    #
    # x_gripper = float(raw_input("Enter the x coordinate of the gripper: "))
    # y_gripper = float(raw_input("Enter the y coordinate of the gripper: "))
    # yaw_gripper = float(raw_input("Enter the yaw of the gripper: "))
    # result = move_sergio_to_handover_pose(sergio, x_gripper=x_gripper, y_gripper=y_gripper, yaw_gripper=yaw_gripper)
    # rospy.loginfo("{0}".format(result))
    #
    # raw_input("Press enter as soon as AMIGO has place the object on SERGIO's tray")
    # result = move_sergio_back(sergio)
    # print "Result: {0}".format(result)
