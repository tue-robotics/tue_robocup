import rospy
import smach
from robot_skills.util import transformations


import geometry_msgs

from robot_smach_states.human_interaction import Say
import robot_skills.util.msg_constructors as msgs
from robot_skills.arms import ArmState
from robot_smach_states.util.designators import PointStampedOfEntityDesignator


# TODO: poses to move to robot_description:
# carrying_pose: 0.18, y_home, 0.75, 0, 0, 0, 60
# handover_pose: 0.6, y_home, 0.966, 0, 0, 0, 30
# handover_to_human: 
# prepare_grasp: -0.2, -0.044, 0.69, 1.4, -0.13, 0.38, 0.42
# 
# TODO: trajectories to move to robot_description:
#
#
#
#
#
#

class ArmToJointConfig(smach.State):
    def __init__(self, robot, arm_designator, configuration):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot
        self.arm_designator = arm_designator
        self.configuration = configuration

    def execute(self, userdata=None):
        arm = self.arm_designator.resolve()
        if arm.send_joint_goal(self.configuration):
            return 'succeeded'
        return "failed"

class ArmToJointTrajectory(smach.State):
    def __init__(self, robot, arm_designator, trajectory):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot
        self.arm_designator = arm_designator
        self.trajectory = trajectory

    def execute(self, userdata=None):
        arm = self.arm_designator.resolve()
        if arm.send_joint_trajectory(self.trajectory):
            return 'succeeded'
        return "failed"

class PrepareGraspSafe(smach.State):
    def __init__(self, robot, arm_designator, grab_entity_designator):
        ''' Similar to PrepareGrasp but has a more elaborate joint trajectory to avoid hitting the table when standing close to it'''
        smach.State.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot
        self.arm_designator = arm_designator
        self.grab_entity_designator = grab_entity_designator

    def execute(self, gl):

        # Move arm to desired joint coordinates (no need to wait)
        # ToDo: don't hardcode
        self.arm.send_joint_trajectory([
        [-0.1,-0.6,0.1,1.2,0.0,0.1,0.0],
        [-0.1,-0.8,0.1,1.6,0.0,0.2,0.0],
        [-0.1,-1.0,0.1,2.0,0.0,0.3,0.0],
        [-0.1,-0.5,0.1,2.0,0.0,0.3,0.0],
        ],
        timeout=20)
        #[-0.2, -0.044, 0.69, 1.4, -0.13, 0.38, 0.42]

        # If the z-position of the object is above a suitable threshold, move the spindle so that the object position can later be updated using the laser
        # Probably do this afterwards
        answers = self.robot.reasoner.query(self.grab_entity_designator)

        if answers:
            answer = answers[0] #ToDo: 0.55 is a dirty hack, might be better
            spindle_target = float(answer["Z"]) - 0.45

            rospy.logwarn("Temp spindle target")
            spindle_target = 0.4

            # ToDo: parameterize
            if (spindle_target > self.robot.spindle.lower_limit):
                self.robot.spindle.send_goal(spindle_target,timeout=5.0)
            # Else: there's no point in doing it any other way

        return 'succeeded'


########################################### State Grab ###############################################

class GrabMachine(smach.StateMachine):
    def __init__(self, robot, arm_designator, grab_entity_designator):
        """@param grab_entity_designator resolves to an entity to grab.
            Some child states require a (designator of) the PointStamped of that entity. PointStampedOfEntityDesignator does this conversion"""
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        '''check check input and output keys'''
        with self:
            smach.StateMachine.add('PREPARE_GRAB', ArmToJointConfig(robot, arm_designator, "prepare_grasp"),
                        transitions={'succeeded'    :   'OPEN_GRIPPER',
                                     'failed'       :   'failed'})

            smach.StateMachine.add('OPEN_GRIPPER', SetGripper(robot, arm_designator, gripperstate=ArmState.OPEN, timeout=0.0),
                        transitions={'succeeded'    :   'PREPARE_ORIENTATION',
                                     'failed'       :   'PREPARE_ORIENTATION'})

            smach.StateMachine.add('PRE_GRASP', ArmToQueryPoint(robot, arm_designator, 
                                    PointStampedOfEntityDesignator(grab_entity_designator), 
                                    time_out=20, pre_grasp=True, first_joint_pos_only=True),
                        transitions={'succeeded'    :   'GRAB',
                                     'failed'       :   'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('GRAB', GrabWithVisualServoing(robot, arm_designator, grab_entity_designator),
                        transitions={'grab_succeeded':  'CLOSE_GRIPPER',
                                     'grab_failed'   :  'CLOSE_GRIPPER',
                                     'target_lost'   :  'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('CLOSE_GRIPPER', SetGripper(robot, arm_designator, gripperstate=ArmState.CLOSE, grab_entity_designator=grab_entity_designator),
                        transitions={'succeeded'    :   'LIFT',
                                     'failed'       :   'LIFT'})

            smach.StateMachine.add('LIFT', ArmToUserPose(arm_designator, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'RETRACT','failed':'RETRACT'})

            smach.StateMachine.add('RETRACT', ArmToUserPose(arm_designator, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'CARR_POS','failed':'CARR_POS'})

            smach.StateMachine.add('CARR_POS', ArmToJointConfig(robot, arm_designator, "carrying_pose"),
                        transitions={'succeeded':'succeeded','failed':'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('CLOSE_GRIPPER_UPON_FAIL', SetGripper(robot, arm_designator, gripperstate=ArmState.CLOSE, timeout=0.0),
                        transitions={'succeeded'    :   'failed',
                                     'failed'       :   'failed'})


class GrabWithVisualServoing(smach.State):
    def __init__(self, robot, arm_designator, grab_point_designator):
        smach.State.__init__(self, outcomes=['succeeded','failed','target_lost'])

        self.robot = robot
        self.arm_designator = arm_designator
        self.grab_point_designator = grab_point_designator


    def execute(self, userdata=None):
        arm = self.arm_designator.resolve()

        if arm == self.robot.arms["left"]:
            self.end_effector_frame_id = "/"+self.robot.robot_name+"/grippoint_left"
            self.ar_frame_id = "/hand_marker_left"
        elif arm == self.robot.arms["right"]:
            self.end_effector_frame_id = "/"+self.robot.robot_name+"/grippoint_right"
            self.ar_frame_id = "/hand_marker_right"

        try:
            target_position = self.grab_point_designator.resolve()
        except Exception, e:
            rospy.loginfo("Could not resolve grab_point_designator {0}: {1}".format(self.grab_point_designator, e))
            return "target_lost"

        ''' Keep looking at end-effector for ar marker detection '''
        self.robot.head.set_position(msgs.PointStamped(0,0,0,frame_id=self.end_effector_frame_id),keep_tracking=True)
        rospy.loginfo("[robot_smach_states:grasp] Target position: {0}".format(target_position))

        target_position_bl = transformations.tf_transform(target_position.point, target_position.header.frame_id,"/amigo/base_link", tf_listener=self.robot.tf_listener)
        rospy.loginfo("[robot_smach_states] Target position in base link: {0}".format(target_position_bl))

        target_position_delta = geometry_msgs.msg.Point()

        ''' First check to see if visual servoing is possible '''
        self.robot.perception.toggle(['ar_pose'])

        ''' Transform point(0,0,0) in ar marker frame to grippoint frame '''
        ar_point = msgs.PointStamped(0, 0, 0, frame_id = self.ar_frame_id, stamp = rospy.Time())

        ''' Check several times if transform is available '''
        cntr = 0
        ar_marker_available = False
        while (cntr < 5 and not ar_marker_available):
            rospy.logdebug("Checking AR marker for the {0} time".format(cntr+1))
            ar_point_grippoint = transformations.tf_transform(ar_point, self.ar_frame_id, self.end_effector_frame_id, tf_listener=self.robot.tf_listener)
            if not ar_point_grippoint == None:
                ar_marker_available = True
            else:
                cntr += 1
                rospy.sleep(0.2)

        ''' If transform is not available, try again, but use head movement as well '''
        if not ar_marker_available:
            arm.send_delta_goal(0.05,0.0,0.0,0.0,0.0,0.0, timeout=5.0, frame_id=self.end_effector_frame_id, pre_grasp = False)
            self.robot.speech.speak("Let me have a closer look", block=False)

        ar_point_grippoint = transformations.tf_transform(ar_point, self.ar_frame_id, self.end_effector_frame_id, tf_listener=self.robot.tf_listener)
        rospy.loginfo("AR marker in end-effector frame = {0}".format(ar_point_grippoint))

        ''' Transform target position to grippoint frame '''
        target_position_grippoint = transformations.tf_transform(target_position, "/map", self.end_effector_frame_id, tf_listener=self.robot.tf_listener)
        rospy.loginfo("Target position in end-effector frame = {0}".format(target_position_grippoint))

        ''' Compute difference = delta (only when both transformations have succeeded) and correct for offset ar_marker and grippoint '''
        if not (ar_point_grippoint == None or target_position_grippoint == None):
            target_position_delta = msgs.Point(target_position_grippoint.x - ar_point_grippoint.x + arm.markerToGrippointOffset.x,
                target_position_grippoint.y - ar_point_grippoint.y + arm.markerToGrippointOffset.y,
                target_position_grippoint.z - ar_point_grippoint.z + arm.markerToGrippointOffset.z)
            rospy.loginfo("Delta target = {0}".format(target_position_delta))
            ar_marker_available = True
        else:
            ar_marker_available = False
        rospy.logwarn("ar_marker_available (2) = {0}".format(ar_marker_available))

        ''' Sanity check '''
        if ar_marker_available:
            #rospy.loginfo("Delta target = {0}".format(target_position_delta))
            if (target_position_delta.x < 0 or target_position_delta.x > 0.6 or target_position_delta.y < -0.3 or target_position_delta.y > 0.3 or target_position_delta.z < -0.3 or target_position_delta.z > 0.3):
                rospy.logwarn("Ar marker detection probably incorrect")
                self.robot.speech.speak("I guess I cannot see my hand properly", block=False)
                ar_marker_available = False
        rospy.logwarn("ar_marker_available (3) = {0}".format(ar_marker_available))

        ''' Switch off ar marker detection '''
        self.robot.perception.toggle([])

        ''' Original, pregrasp is performed by the compute_pre_grasp node '''
        if not ar_marker_available:
            self.robot.speech.speak("Let's see", block=False)
            if arm.send_goal(target_position_bl.x, target_position_bl.y, target_position_bl.z, 0, 0, 0, 120, pre_grasp = True):
                rospy.loginfo("arm at object")
            else:
                rospy.logerr("Goal unreachable: {0}".format(target_position_bl).replace("\n", " "))
                self.robot.speech.speak("I am sorry but I cannot move my arm to the object position", block=False)
                return 'grab_failed'
        else:
            self.robot.speech.speak("Let's go", block=False)
            if arm.send_delta_goal(target_position_delta.x, target_position_delta.y, target_position_delta.z,
                                        0, 0, 0, 120, frame_id=self.end_effector_frame_id, pre_grasp = True):
                rospy.loginfo("arm at object")
            else:
                rospy.logerr("Goal unreachable: {0}".format(target_position_bl).replace("\n", " "))
                self.robot.speech.speak("I am sorry but I cannot move my arm to the object position", block=False)
                return 'grab_failed'

        self.robot.head.reset_position(timeout=0.0)
        return 'grab_succeeded'


class HandoverFromHuman(smach.StateMachine):
    def __init__(self, robot, arm_designator):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        with self:
            smach.StateMachine.add("POSE", ArmToJointConfig(robot, arm_designator, "carrying_pose"),
                            transitions={'succeeded':'OPEN_BEFORE_INSERT','failed':'OPEN_BEFORE_INSERT'})

            smach.StateMachine.add( 'OPEN_BEFORE_INSERT', SetGripper(robot, arm_designator, gripperstate=0), #open
                                transitions={'succeeded'    :   'SAY1',
                                             'failed'       :   'SAY1'})

            smach.StateMachine.add("SAY1", Say(self.robot,'Please hand over the object by sliding it in my gripper'),
                            transitions={'spoken':'CLOSE_AFTER_INSERT'})

            smach.StateMachine.add( 'CLOSE_AFTER_INSERT', SetGripper(robot, arm_designator, gripperstate=1), #close
                                transitions={'succeeded'    :   'succeeded',
                                             'failed'       :   'failed'})


class HandoverToHuman(smach.StateMachine):
    def __init__(self, side, robot=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot

        with self:

            smach.StateMachine.add("SPINDLE_MEDIUM",
                        ResetTorso(self.robot),
                        transitions={'succeeded':'MOVE_HUMAN_HANDOVER_JOINT_GOAL',
                                     'failed'   :'MOVE_HUMAN_HANDOVER_JOINT_GOAL'})

            smach.StateMachine.add("MOVE_HUMAN_HANDOVER_JOINT_GOAL",
                        ArmToJointPos(self.robot, self.side, (-0.2, -0.7, 0.2, 2.0, 0, 0.5, 0.3), timeout=1.0),
                        transitions={   'done':'SAY_OPEN_GRIPPER',
                                      'failed':'SAY_OPEN_GRIPPER'})
            #smach.StateMachine.add('MOVE_HUMAN_HANDOVER', ArmToUserPose(self.side, 0.2, 0.3, 1.0, 0.0, 0.0 , 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=False),
            #            transitions={'succeeded':'SAY_OPEN_GRIPPER','failed':'SAY_OPEN_GRIPPER'})

            smach.StateMachine.add("SAY_OPEN_GRIPPER",
                        Say(robot, [ "Be careful, I will open my gripper now"]),
                        transitions={   'spoken':'OPEN_GRIPPER_HANDOVER'})

            smach.StateMachine.add('OPEN_GRIPPER_HANDOVER', SetGripper(self.robot, self.side, gripperstate=ArmState.OPEN),
                        transitions={'succeeded'    :   'CLOSE_GRIPPER_HANDOVER',
                                     'failed'       :   'CLOSE_GRIPPER_HANDOVER'})

            smach.StateMachine.add('CLOSE_GRIPPER_HANDOVER', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE, timeout=0.0),
                        transitions={'succeeded'    :   'RESET_ARM',
                                     'failed'       :   'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM',
                        ArmToJointPos(self.robot, self.side, StandardPoses.RESET_POSE, timeout=1.0),
                        transitions={   'done':'RESET_TORSO',
                                      'failed':'RESET_TORSO'    })

            smach.StateMachine.add('RESET_TORSO',
                        ResetTorso(self.robot),
                        transitions={'succeeded':'succeeded',
                                     'failed'   :'succeeded'})


class SetGripper(smach.State):
    def __init__(self, robot, arm_designator, gripperstate='open', drop_from_frame=None, grab_entity_designator=None, timeout=10):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.arm_designator = arm_designator
        self.robot = robot
        self.gripperstate = gripperstate
        self.grab_entity_designator = grab_entity_designator
        self.timeout = timeout

    def execute(self, userdata):
        arm = self.arm_designator.resolve()
        if arm == self.robot.arms["left"]:
            self.end_effector_frame_id = "/amigo/grippoint_left"
        elif arm == self.robot.arms["right"]:
            self.end_effector_frame_id = "/amigo/grippoint_right"

        ''' If needs attaching to gripper, the grab_entity_designator is used '''
        if self.grab_entity_designator:
            try:
                entity = self.grab_entity_designator.resolve()
                arm.occupied_by = entity
            except Exception, e:
                rospy.logerr("Could not resolve {0}: {1}".format(self.grab_entity_designator, e))

        if arm.send_gripper_goal(self.gripperstate, timeout=self.timeout):
            result = True
        else:
            result = False

        # ToDo: make sure things can get attached to the gripper in this state. Use userdata?
        if self.gripperstate == ArmState.OPEN:
            arm.occupied_by = None

        # ToDo: check for failed in other states
        if result:
            return 'succeeded'
        else:
            return 'failed'

class Gripper_to_query_position(smach.StateMachine):
    def __init__(self, robot, side, point_query):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed', 'target_lost'])
        self.side = side
        self.robot = robot
        self.point_query = point_query

        with self:
            smach.StateMachine.add("PREPARE",
                                    PrepareOrientation(side, robot, point_query),
                                    transitions={   'orientation_succeeded':'MOVE_ARM',
                                                    'orientation_failed':'failed',
                                                    'abort':'failed',
                                                    'target_lost':'target_lost'})

            @smach.cb_interface(outcomes=['succeeded', 'failed', 'target_lost'])
            def move_to_point(userdata):
                answers = self.robot.reasoner.query(self.point_query)

                if not answers:
                    rospy.loginfo("No answers for query {0}".format(self.point_query))
                    return "target_lost"
                else:
                    answer = answers[0]
                    target_position = msgs.PointStamped(float(answer["X"]), float(answer["Y"]), float(answer["Z"]), frame_id = "/map", stamp = rospy.Time())
                    # Question: why is pre_grasp per definition true?
                    if self.side.send_goal(target_position.point.x, target_position.point.y, target_position.point.z, 0, 0, 0, 120,
                                            pre_grasp=True,
                                            frame_id="/map"):
                        return 'succeeded'
                    else:
                        return 'failed'
            smach.StateMachine.add('MOVE_ARM', smach.CBState(move_to_point),
                                transitions={   'succeeded':'succeeded',
                                                'failed':'failed',
                                                'target_lost':'target_lost'})

class ArmToJointPos(smach.State):
    def __init__(self, robot, side, jointgoal, timeout=0, delta=False):
        smach.State.__init__(self, outcomes=['done', "failed"])
        self.side = side
        self.robot = robot
        self.jointgoal = jointgoal
        self.timeout = timeout
        self.delta = delta

    def execute(self, userdata):
        if not self.delta:
            result = self.side.send_joint_goal(*self.jointgoal, timeout=self.timeout)
        if self.delta:
            result = self.side.send_delta_joint_goal(*self.jointgoal, timeout=self.timeout)
        if result:
            return "done"
        else:
            return "failed"

ArmToPose = ArmToJointPos

class ArmFollowTrajectory(smach.State):
    def __init__(self, robot, side, trajectory, timeout=0):
        smach.State.__init__(self, outcomes=['done', "failed"])
        self.side = side
        self.robot = robot
        self.trajectory = trajectory
        self.timeout = timeout

    def execute(self, userdata):
        result = self.side.send_joint_trajectory(*self.jointgoal,timeout=self.timeout)
        if result:
            return "done"
        else:
            return "failed"

class ArmFollowDeltaTrajectory(smach.State):
    def __init__(self, robot, side, delta_dict_list, timeout=0, origin=None):
        smach.State.__init__(self, outcomes=['done', "failed"])
        self.side = side
        self.robot = robot
        self.delta_dict_list = delta_dict_list
        self.timeout = timeout
        self.origin = origin

    def execute(self, userdata):
        result = self.side.send_delta_joint_trajectory(self.delta_dict_list, timeout=self.timeout, origin=self.origin)
        if result:
            return "done"
        else:
            return "failed"

class ArmToUserPose(smach.State):
    def __init__(self, side, x, y, z, roll=0, pitch=0, yaw=0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=False):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.side = side
        self.x = x
        self.y = y
        self.z = z
        self.roll  = roll
        self.pitch = pitch
        self.yaw   = yaw
        self.time_out   = time_out
        self.pre_grasp = pre_grasp
        self.frame_id  = frame_id
        self.delta = delta

    def execute(self, userdata):
        if not self.delta:
            rospy.logwarn("Transforming to baselink, should become obsolete but this is not yet the case")
            #target_position_bl = transformations.tf_transform(target_position, "/map","/base_link", tf_listener=self.robot.tf_listener)
            if self.side.send_goal(self.x, self.y, self.z, self.roll, self.pitch, self.yaw,
                timeout=self.time_out,
                pre_grasp=self.pre_grasp,
                frame_id=self.frame_id):
                return 'succeeded'
            else:
                return 'failed'
        else:
            if self.side.send_delta_goal(self.x, self.y, self.z, self.roll, self.pitch, self.yaw,
                timeout=self.time_out,
                pre_grasp=self.pre_grasp,
                frame_id=self.frame_id):
                return 'succeeded'
            else:
                return 'failed'

class ArmToQueryPoint(smach.State):
    def __init__(self, robot, side, point_designator, time_out=20, pre_grasp=False, first_joint_pos_only=False):
        """Move the arm to de designated point
        @param point_designator resolves to a PointStamped the gripper should move to"""
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot                = robot
        self.side                 = side
        self.point_designator     = point_designator
        self.time_out             = time_out
        self.pre_grasp            = pre_grasp
        self.first_joint_pos_only = first_joint_pos_only

    def execute(self, userdata):
        # ToDo: check point_designator?
        try:
            goal = self.point_designator.resolve()
        except Exception, e:
            rospy.loginfo("point_designator {0} cannot be resolved: {1}".format(self.point_designator, e))
            return 'failed'

        rospy.loginfo("ArmToQueryPoint: goal = {0}".format(goal))

        # Note: answers are typically in "map"frame, check whether this works out
        rospy.logwarn("Transforming to base_link frame for amigo_arm_navigation")
        goal_bl = transformations.tf_transform(goal.point, goal.header.frame_id, "/amigo/base_link", tf_listener=self.robot.tf_listener)
        if goal_bl == None:
            return 'failed'

        if self.side.send_goal(goal_bl.x, goal_bl.y, goal_bl.z, 0, 0, 0,
            frame_id="/amigo/base_link",
            timeout=self.time_out,
            pre_grasp=self.pre_grasp,
            first_joint_pos_only=self.first_joint_pos_only):
            return 'succeeded'
        else:
            rospy.logwarn("Goal unreachable: {0}".format(goal_bl).replace("\n", " "))
            return 'failed'

class TorsoToUserPos(smach.State):
    def __init__(self, robot, torso_pos, time_out=0.0):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot
        self.torso_pos = torso_pos
        self.time_out = time_out

    def execute(self, userdata):
        if self.robot.spindle.send_goal(self.torso_pos,timeout=self.time_out):
            return 'succeeded'
        else:
            return 'failed'

########################################### State Point ###############################################

class PointMachine(smach.StateMachine):
    def __init__(self, side, robot, grab_entity_designator):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot
        self.grab_entity_designator = grab_entity_designator
        '''check check input and output keys'''
        with self:
            smach.StateMachine.add('PREPARE_POINT', PrepareGrasp(self.side, self.robot, self.grab_entity_designator),
                        transitions={'succeeded'    :   'PREPARE_ORIENTATION',
                                     'failed'       :   'failed'})

            smach.StateMachine.add('PREPARE_ORIENTATION', PrepareOrientation(self.side, self.robot, self.grab_entity_designator),
                        transitions={'orientation_succeeded':'CLOSE_GRIPPER','orientation_failed':'CLOSE_GRIPPER','abort':'failed','target_lost':'failed'})

            smach.StateMachine.add('CLOSE_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE),
                        transitions={'succeeded'    :   'PRE_POINT',
                                     'failed'       :   'PRE_POINT'})

            smach.StateMachine.add('PRE_POINT', ArmToQueryPoint(self.robot, self.side, self.grab_entity_designator, time_out=20, pre_grasp=True, first_joint_pos_only=True),
                        transitions={'succeeded'    :   'POINT',
                                     'failed'       :   'RESET_ARM_FAILED'})

            smach.StateMachine.add('POINT', Point_at_object(self.side, self.robot, self.grab_entity_designator),
                        transitions={'point_succeeded':  'SAY_POINTED',
                                     'point_failed'   :  'RETRACT',
                                     'target_lost'   :  'RESET_ARM_FAILED'})

            smach.StateMachine.add('SAY_POINTED',
                                    Say(robot, "I hope this is the object you were looking for."),
                        transitions={ 'spoken':'RETRACT' })

            smach.StateMachine.add('RETRACT', ArmToUserPose(self.side, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'RESET_ARM_SUCCEEDED','failed':'RESET_ARM_SUCCEEDED'})

            smach.StateMachine.add('RESET_ARM_SUCCEEDED',
                                    ArmToJointPos(robot, robot.leftArm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)),
                        transitions={   'done':'succeeded',
                                        'failed':'succeeded'})

            smach.StateMachine.add('RESET_ARM_FAILED',
                                    ArmToJointPos(robot, robot.leftArm, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)),
                        transitions={   'done':'failed',
                                        'failed':'failed'})

class Point_at_object(smach.State):
    def __init__(self, side, robot, grab_entity_designator):
        smach.State.__init__(self, outcomes=['point_succeeded','point_failed','target_lost'])

        self.side = side
        self.robot = robot
        self.grab_entity_designator = grab_entity_designator

        if self.side == self.robot.leftArm:
            self.end_effector_frame_id = "/amigo/grippoint_left"
        elif self.side == self.robot.rightArm:
            self.end_effector_frame_id = "/amigo/grippoint_right"

    def execute(self, gl):

        answers = self.robot.reasoner.query(self.grab_entity_designator)

        if answers:
            answer = answers[0] #TODO Loy/Sjoerd: sort answers by distance to gripper/base?
            target_position = msgs.PointStamped(float(answer["X"]), float(answer["Y"]), float(answer["Z"]), frame_id = "/map", stamp = rospy.Time())
            rospy.loginfo("[robot_smach_states:grasp] Target position: {0}".format(target_position))
        else:
            rospy.loginfo("No answers for query {0}".format(self.grab_entity_designator))
            return "target_lost"

        ''' Keep looking at end-effector for ar marker detection '''
        self.robot.head.set_position(msgs.PointStamped(0,0,0,frame_id=self.end_effector_frame_id),keep_tracking=True)

        ''' Transform to base link '''
        target_position_bl = transformations.tf_transform(target_position, "/map","/amigo/base_link", tf_listener=self.robot.tf_listener)
        rospy.loginfo("[robot_smach_states] Target position in base link: {0}".format(target_position_bl))

        ''' Send goal '''
        if self.side.send_goal(target_position_bl.x-0.1, target_position_bl.y, target_position_bl.z, 0, 0, 0, 120, pre_grasp = True):
            rospy.loginfo("arm at object")
        else:
            rospy.loginfo("Arm cannot reach object")

        self.robot.head.reset_position()
        return 'point_succeeded'

class Point_location_hardcoded(smach.State):
    def __init__(self, robot, side, point_amount):
        smach.State.__init__(self, outcomes=['pointed'])
        self.side = side
        self.robot = robot
        self.i = 0
        self.point_amount = point_amount

    def execute(self, userdata):

        self.side.send_gripper_goal_close()
        i = self.i
        while i < self.point_amount:
            #Backwards
            self.side.send_joint_goal(-0.4 ,-0.750 , 0.50 , 1.50 , 0.000 , 0.7500 , 0.000)
            rospy.sleep(3.2)
            #Forwards
            self.side.send_joint_goal(-0.2 ,-0.250 , 0.40 , 1.25 , 0.000 ,0.500 , 0.000)
            rospy.sleep(3.2)
            i += 1

        self.side.reset_arm()

        return 'pointed'

