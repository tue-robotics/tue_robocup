import action_result
import rospy
import robot_smach_states as states
import PyKDL
import json

BAR_ENTITY_FRAME_ID = "/bar"
DIST_FROM_BAR = 1.0
PLACE_JOINT_CONFIG = [0, 1.0, 0.3, 0.8, -0.2, -.2, 0]
RESET_JOINT_CONFIG = [-0.1, -0.2, 0.2, 0.8, 0.0, 0.0, 0.0]
PLACE_TORSO_HEIGHT = 0.2
RETRACT_TORSO_HEIGHT = 0.4
HANDOVER_POSE_RADIUS =0.05
ARM_SIDE = "left"

def amigo_navigate_amigo_to_handover(amigo):
    navigateToPoseSM = states.NavigateToPose(amigo,
                                             x=DIST_FROM_BAR,
                                             y=0.0,
                                             z=0.0,
                                             radius=HANDOVER_POSE_RADIUS,
                                             frame_id=BAR_ENTITY_FRAME_ID)

def amigo_move_arm_to_place_position(amigo):
    if amigo.rightArm._send_joint_trajectory([PLACE_JOINT_CONFIG]):
        res = ActionResult.SUCCEEDED
        (x, y, z), (rx, ry, rz, rw) = amigo.tf_listener.lookupTransform("/map", "/amigo/grippoint_%s"%ARM_SIDE)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([rx, ry, rz, rw])
        msg = "Amigo: I'm ready to place the drink at: %s" %json.dumps({'x':x, 'y':y, 'yaw': yaw})
    else:
        res = ActionResult.FAILED
        msg = "Amigo: Place joint goal could not be reached"

    return ActionResult(res,msg)

def amigo_place(amigo):
    res = ActionResult.FAILED

    # Send lower torso goal and wait for the torso to reach it
    if not amigo.torso._send_goal([PLACE_TORSO_HEIGHT]):
        return ActionResult(res,"Amigo: Could not reach lower torso goal")
    amigo.torso.wait_for_motion_done()

    # Open gripper and wait for it to open
    if ARM_SIDE == "left":
        amigo.leftArm.send_gripper_goal('open')
    elif ARM_SIDE == "right":
        amigo.rightArm.send_gripper_goal('open')
    rospy.sleep(1.0)

    # Send upper torso goal and wait for the torso to reach it
    if not amigo.torso.send_goal([RETRACT_TORSO_HEIGHT]):
        return ActionResult(res,"Amigo: Could not reach upper torso goal")
    amigo.torso.wait_for_motion_done()

    # Close gripper and wait for it to close
    if ARM_SIDE == "left":
        amigo.leftArm.send_gripper_goal('close')
    elif ARM_SIDE == "right":
        amigo.rightArm.send_gripper_goal('close')
    rospy.sleep(1.0)

    # If all went wel, return succeeded
    res = ActionResult.SUCCEEDED
    return ActionResult(res, "Amigo: Successfully placed item")

def amigo_reset_arm(amigo):
    if ARM_SIDE == "left":
        if not amigo.leftArm._send_joint_trajectory([RESET_JOINT_CONFIG]):
            return ActionResult(ActionResult.FAILED, "Amigo: Failed to move arm to reset position")
        amigo.leftArm.wait_for_motion_done()
    elif ARM_SIDE == "right":
        if not amigo.rightArm._send_joint_trajectory([RESET_JOINT_CONFIG]):
            return ActionResult(ActionResult.FAILED, "Amigo: Failed to move arm to reset position")
        amigo.rightArm.wait_for_motion_done()

    return ActionResult(ActionResult.SUCCEEDED, "Amigo: Reset arm succeeded")
