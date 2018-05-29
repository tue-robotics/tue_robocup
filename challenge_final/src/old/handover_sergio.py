#!/usr/bin/python

import math

import rospy
import geometry_msgs.msg
import PyKDL as kdl

from robot_skills.sergio import Sergio
from robot_smach_states.navigation.navigate_to_pose import NavigateToPose
from action_result import ActionResult

TESTMODE = True

ENTITY_FRAME_ID = "bar"
X_OFFSET = 2.5  # Distance between the center point of the bar and the pose where SERGIO will stand

TRANS_ERROR_CONSTRAINT = 0.05  # We're satisfied if we're within 5 cm in each direction
ROT_ERROR_CONSTRAINT = 10.0/180.0 * math.pi  # and within 10 degrees

MAX_TRANS_ERROR = 1.0  # If we're further than 1.5 meters apart, better skip it
MAX_ROT_ERROR = 45.0/180.0 * math.pi  # or if the angle offset is more than 30 degrees

# Controller parameters
TRANS_GAIN = 0.4  #0.2
ROT_GAIN = 0.3  #0.15

MAX_TRANS_VEL = 0.5
# MIN_TRANS_VEL = 0.05
MAX_ROT_VEL = 1.0
# MIN_ROT_VEL = 0.1

OBJ_OFFSET_X = 0.0425  # Desired x-pos of the object in base link frame
OBJ_OFFSET_Y = -0.14  # Desired y-pos of the object in base link frame


def move_sergio_to_pre_handover_pose(sergio, entity_frame_id):
    """ Makes SERGIO navigate to the pre handover pose """
    nav_state = NavigateToPose(sergio, x=X_OFFSET, y=0.0, rz=math.pi, radius=0.10, frame_id=entity_frame_id)
    result = nav_state.execute()
    if result == 'arrived':
        return ActionResult(ActionResult.SUCCEEDED, 'SERGIO reached pre handover pose')
    else:
        return ActionResult(ActionResult.FAILED, 'SERGIO failed to reach the pre-handover pose, goal {0}'.format(result))


def move_sergio_to_handover_pose(sergio, x_gripper_map, y_gripper_map, yaw_gripper_map):
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

    # Convert the gripper pose to odom frame to get rid of localization jitter
    gripper_pose_map = geometry_msgs.msg.PoseStamped()
    gripper_pose_map.header.frame_id = "/map"
    gripper_pose_map.header.stamp = rospy.Time.now()
    gripper_pose_map.pose.position.x = x_gripper_map
    gripper_pose_map.pose.position.y = y_gripper_map
    o = kdl.Rotation.RPY(0, 0, yaw_gripper_map)
    (rx, ry, rz, rw) = o.GetQuaternion()
    gripper_pose_map.pose.orientation.x = rx
    gripper_pose_map.pose.orientation.y = ry
    gripper_pose_map.pose.orientation.z = rz
    gripper_pose_map.pose.orientation.w = rw

    gripper_pose_odom_msg = sergio.tf_listener.transformPose("/sergio/odom", gripper_pose_map)
    gripper_pose_odom = kdl.Frame(kdl.Rotation.Quaternion(gripper_pose_odom_msg.pose.orientation.x,
                                                          gripper_pose_odom_msg.pose.orientation.y,
                                                          gripper_pose_odom_msg.pose.orientation.z,
                                                          gripper_pose_odom_msg.pose.orientation.w),
                                  kdl.Vector(gripper_pose_odom_msg.pose.position.x,
                                             gripper_pose_odom_msg.pose.position.y,
                                             gripper_pose_odom_msg.pose.position.z))

    gripper_pose_odom.M.DoRotZ(math.pi)  # Rotate PI around the z axis to make sure SERGIO is opposite to AMIGO's gripper

    # yaw_ref = yaw_gripper + math.pi  # SERGIO should be opposite to the AMIGO gripper
    # gripper_pose_map = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw_ref),
    #                              kdl.Vector(x_gripper, y_gripper, 0))  # Note: using yaw ref instead of yaw gripper
    # # implies that we have already rotated 180 degrees

    rate = rospy.Rate(20.0)
    timeout_time = 30

    start_time = rospy.Time.now()
    while True and not rospy.is_shutdown():

        if (rospy.Time.now() - start_time).to_sec() > timeout_time:
            return ActionResult(ActionResult.FAILED, "Timeout of %d seconds passsed, aborted" % timeout_time)

        # # Check the current offset
        # try:
        #     t, r = sergio.tf_listener.lookupTransform("sergio/base_link", source_frame, rospy.Time(0))
        # (x, y, z), (rx, ry, rz, rw) =
        # except:
        #     print "TF error"
        #     return ActionResult(ActionResult.FAILED, "TF Lookup failed")
        #
        # trans = kdl.Vector(t[0], t[1], t[2])
        # rot = kdl.Rotation.Quaternion(r[0], r[1], r[2], r[3])
        # (roll, pitch, yaw) = rot.GetRPY()

        # base_pose_map_msg = sergio.base.get_location().frame  #
        # base_pose_map = kdl.Frame(kdl.Rotation.Quaternion(base_pose_map_msg.pose.orientation.x,
        #                                                   base_pose_map_msg.pose.orientation.y,
        #                                                   base_pose_map_msg.pose.orientation.z,
        #                                                   base_pose_map_msg.pose.orientation.w),
        #                           kdl.Vector(base_pose_map_msg.pose.position.x,
        #                                      base_pose_map_msg.pose.position.y,
        #                                      base_pose_map_msg.pose.position.z))

        # tf_listener.waitForTransform("/map", "/" + robot_name + "/base_link", rospy.Time.now(), rospy.Duration(20.0))
        # t = f.getLatestCommonTime("sergio/base_link", "sergio/odom")
        try:
            (ro_trans, ro_rot) = sergio.tf_listener.lookupTransform("sergio/odom", "/sergio/base_link", rospy.Time(0))

            # position = geometry_msgs.msg.Point()
            # orientation = geometry_msgs.msg.Quaternion()

            # position.x = ro_trans[0]
            # position.y = ro_trans[1]
            # orientation.x = ro_rot[0]
            # orientation.y = ro_rot[1]
            # orientation.z = ro_rot[2]
            # orientation.w = ro_rot[3]
        except:
            rospy.logerr("Cannot get transform between baselink and odom")
            return ActionResult(ActionResult.FAILED, "Transform error, feedback loop does not work")

        base_pose_odom = kdl.Frame(kdl.Rotation.Quaternion(ro_rot[0],
                                                           ro_rot[1],
                                                           ro_rot[2],
                                                           ro_rot[3]),
                                   kdl.Vector(ro_trans[0], ro_trans[1], ro_trans[2]))

        # error = kdl.diff(gripper_pose_map, base_pose_map)
        # trans = error.vel
        # rot = error.rot.z()

        ref_pose_robot = base_pose_odom.Inverse() * gripper_pose_odom
        ref_pose_robot.p.x(ref_pose_robot.p.x() - OBJ_OFFSET_X)
        ref_pose_robot.p.y(ref_pose_robot.p.y() - OBJ_OFFSET_Y)
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
            rospy.logwarn("Distance x: {0}, y: {1}, yaw: {2}".format(trans.x(), trans.y(), rot))
            return ActionResult(ActionResult.FAILED, "SERGIO and AMIGO are too far apart for safe handover")

        # Else, control action
        vx = TRANS_GAIN * trans.x()
        vy = TRANS_GAIN * trans.y()
        vth = ROT_GAIN * rot

        v = geometry_msgs.msg.Twist()  # Initialize velocity
        v.linear.x = max(-MAX_TRANS_VEL, min(MAX_TRANS_VEL, vx))
        v.linear.y = max(-MAX_TRANS_VEL, min(MAX_TRANS_VEL, vy))
        v.angular.z = max(-MAX_ROT_VEL, min(MAX_ROT_VEL, vth))

        # import ipdb;        ipdb.set_trace()

        sergio.base._cmd_vel.publish(v)

        rate.sleep()


def move_sergio_back(sergio):
    """ Makes SERGIO forcedrive backwards until it is out of the way """
    # ToDo: as an additional measure of robustness, we could check if SERGIO is really out of the way...
    sergio.base.force_drive(-0.2, 0, 0, 3.0)
    return ActionResult(ActionResult.SUCCEEDED, "SERGIO is out of the way")


if __name__ == "__main__":

    """ Test stuff """
    rospy.init_node("test_handover_motions")
    sergio = Sergio(wait_services=True)

    """ Testing in simulation """
    # x_gripper = float(raw_input("Enter the x coordinate of the gripper: "))
    # y_gripper = float(raw_input("Enter the y coordinate of the gripper: "))
    # yaw_gripper = float(raw_input("Enter the yaw of the gripper: "))
    # result = move_sergio_to_handover_pose(sergio, x_gripper_map=x_gripper,
    #                                       y_gripper_map=y_gripper,
    #                                       yaw_gripper_map=yaw_gripper)
    # rospy.loginfo("{0}".format(result))

    # """ Testing for real """
    rospy.loginfo("SERGIO is loaded and will move to the pre-handover pose")
    result = move_sergio_to_pre_handover_pose(sergio, ENTITY_FRAME_ID)
    rospy.loginfo("{0}".format(result))

    # x_gripper = float(raw_input("Enter the x coordinate of the gripper: "))
    # y_gripper = float(raw_input("Enter the y coordinate of the gripper: "))
    # yaw_gripper = float(raw_input("Enter the yaw of the gripper: "))
    inputstr = raw_input("Copy the input string from the AMIGO console: ")
    inputstrlist = inputstr.split(';')
    x_gripper = float(inputstrlist[0])
    y_gripper = float(inputstrlist[1])
    yaw_gripper = float(inputstrlist[2])
    result = move_sergio_to_handover_pose(sergio, x_gripper_map=x_gripper,
                                          y_gripper_map=y_gripper,
                                          yaw_gripper_map=yaw_gripper)
    rospy.loginfo("{0}".format(result))

    raw_input("Press enter as soon as AMIGO has place the object on SERGIO's tray")
    result = move_sergio_back(sergio)
    print "Result: {0}".format(result)
