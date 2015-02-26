import rospy
import smach
import robot_smach_states.navigation
from robot_skills.util import transformations


import geometry_msgs

from robot_smach_states.human_interaction import Say
import robot_skills.util.msg_constructors as msgs
from robot_skills.arms import ArmState
from ed.srv import SimpleQueryRequest

#TODO: Replace Point_location_hardcoded with a ArmToJointPos-sequence.
#TODO: Make Place_Object also use a query

#Enum class.
# ToDo: move to robot_skills because this is robot specific
class StandardPoses:
    POINT_AT_OBJECT_BACKWARD = [-0.4 ,-0.750 , 0.50 , 1.50 , 0.000 , 0.7500 , 0.000]
    POINT_AT_OBJECT_FORWARD = [-0.2 ,-0.250 , 0.40 , 1.25 , 0.000 ,0.500 , 0.000]
    HOLD_TRAY_POSE = [-0.1, 0.13, 0.4, 1.5, 0, 0.5, 0]
    SUPPORT_PERSON_POSE = [-0.1, -1.57, 0, 1.57, 0,0,0]
    RESET_POSE = [-0.1, -0.2, 0.2, 0.8, 0.0, 0.0, 0.0] # This is the usual
    #RESET_POSE = [-0.1, 0.13, 0, 0.3, 0, 0.3, 0]
    #RESET_POSE = [-0.3, -0.05, 0.2, 1.0, 0, 1.0, 0]

class DetermineBaseGraspPose(smach.State):
    def __init__(self, side, robot, grab_point_designator, basepose_designator, x_offset=None, y_offset=None):
        """The point from which to grab an entity """
        smach.State.__init__(self, outcomes=['succeeded','failed','target_lost'])
        self.side = side
        self.robot = robot
        self.grab_point_designator = grab_point_designator
        self.basepose_designator = basepose_designator

        if x_offset == None:
            rospy.logwarn("x_offset not specified, defaulting to 0.5 for arms.")
            self.x_offset = 0.5
        else:
            self.x_offset = x_offset

        if y_offset == None:
            rospy.logwarn("y_offset not specified, defaulting to (-)0.2 for ... arm.")
            if self.side == self.robot.leftArm:
                self.y_offset = 0.2
            elif self.side == self.robot.rightArm:
                self.y_offset = -0.2
        else:
            self.y_offset = y_offset

        self.nr_inverse_reachability_calls = 0
        self.max_nr_inverse_reachability_calls = 2
        self.desired_base_poses_MAP = []
        self.grasp_point = geometry_msgs.msg.PointStamped()

    def execute(self, userdata):

        ''' Only query the reasoner once (grasp position will not be updated anyway so there is no use in doing this more often)'''
        if self.nr_inverse_reachability_calls == 0:
            answers = self.robot.reasoner.query(self.grab_point_designator)

            if answers:
                answer = answers[0]
                rospy.loginfo("Answer(0) = {0}".format(answer))
                self.grasp_point = msgs.PointStamped(float(answer["X"]), float(answer["Y"]), float(answer["Z"]), frame_id = "/map", stamp = rospy.Time())
            else:
                #rospy.logerr("Cannot get target from reasoner, query = {0}".format(self.grab_point_designator))
                return 'target_lost'

        ''' Do a new inverse reachability request if nr < max_nr.
            This way, the grasp pose will be optimal (and the first few tries the change of new information is the highest)
            If nr == max_nr: check all entries of the latest call to increase robustness'''
        #rospy.loginfo('Nr of calls = {0} of {1}'.format(self.nr_inverse_reachability_calls,self.max_nr_inverse_reachability_calls))
        if self.nr_inverse_reachability_calls < self.max_nr_inverse_reachability_calls:
            self.nr_inverse_reachability_calls += 1
            #grasp_point_BASE_LINK = transformations.tf_transform(grasp_point, "/map", "/base_link", self.robot.tf_listener)
            #rospy.loginfo('Grasp point base link = {0}'.format(grasp_point_BASE_LINK))
            self.desired_base_poses_MAP = self.robot.get_base_goal_poses(self.grasp_point, self.x_offset, self.y_offset)
            #rospy.loginfo('Grasp poses base in map = {0}'.format(self.desired_base_poses_MAP))

            ''' If there are no base poses, reset costmap and retry '''
            if not self.desired_base_poses_MAP:
                self.robot.base.reset_costmap()
                self.desired_base_poses_MAP = self.robot.get_base_goal_poses(self.grasp_point, self.x_offset, self.y_offset)

        ''' Sanity check: if the orientation is all zero, no feasible base pose has been found '''
        if not self.desired_base_poses_MAP:
            self.robot.speech.speak("I am very sorry but the goal point is out of my reach",mood="sad", block=False)
            return 'failed'

        x   = self.desired_base_poses_MAP[0].pose.position.x
        y   = self.desired_base_poses_MAP[0].pose.position.y
        phi = transformations.euler_z_from_quaternion(self.desired_base_poses_MAP[0].pose.orientation)
        self.desired_base_poses_MAP.pop(0)

        ''' Assert the goal to the reasoner such that navigate generic can use it '''
        #TODO 3-12-2014: Instead of asserting a pose_2d to the reasoner, store it (or a constraint) in self.basepose_designator
        self.robot.reasoner.query(Compound("retractall", Compound("base_grasp_pose", Compound("pose_2d", "X", "Y", "Phi"))))
        self.robot.reasoner.assertz(Compound("base_grasp_pose", Compound("pose_2d", x, y, phi)))

        return 'succeeded'


class PrepareOrientation(smach.StateMachine):
    def __init__(self, side, robot, grab_entity_designator, x_offset=None, y_offset=None):
        smach.StateMachine.__init__(self, outcomes=['orientation_succeeded','orientation_failed','abort','target_lost'])
        self.side = side
        self.robot = robot
        self.grab_entity_designator = grab_entity_designator
        if x_offset == None:
            rospy.logdebug("x_offset not specified, defaulting to 0.5 for arms.")
            self.x_offset = 0.5
        else:
            self.x_offset = x_offset

        if y_offset == None:
            rospy.logdebug("y_offset not specified, defaulting to (-)0.2 for ... arm.")
            if self.side == self.robot.leftArm:
                self.y_offset = 0.2
            elif self.side == self.robot.rightArm:
                self.y_offset = -0.2
        else:
            self.y_offset = y_offset

        #TODO 3-12-2014: DetermineBaseGraspPose now asserts a base_grasp_pose to the reasoner. Can be replaced by a VariableDesignator
        basepose_designator = VariableDesignator()
        self.determine_grasp_pose = DetermineBaseGraspPose(self.side, self.robot, self.grab_entity_designator, basepose_designator, self.x_offset, self.y_offset)

        @smach.cb_interface(outcomes=['done'])
        def init_prepare_orientation(userdata):
            self.determine_grasp_pose.nr_inverse_reachability_calls = 0
            return "done"

        with self:

            smach.StateMachine.add('INITIALIZE_PREPARE_ORIENTATION', smach.CBState(init_prepare_orientation),
                                    transitions={   'done':'DETERMINE_GRASP_POSE'})

            smach.StateMachine.add('DETERMINE_GRASP_POSE', self.determine_grasp_pose,
                transitions={'succeeded'    :'DRIVE_TO_GRASP_POSE',
                             'failed'       :'orientation_failed',
                             'target_lost'  :'target_lost'})

            smach.StateMachine.add('DRIVE_TO_GRASP_POSE',
                navigation.NavigateGeneric(self.robot, basepose_designator), #TODO 3-12-2014: Replace by new navigation state
                transitions={'arrived'          :'orientation_succeeded',
                             'unreachable'      :'DETERMINE_GRASP_POSE',
                             'preempted'        :'abort',
                             'goal_not_defined' :'DETERMINE_GRASP_POSE'})

########################################### State Prepare grab###############################################
class Carrying_pose(smach.State):
    def __init__(self, arm, robot=None):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot
        self.arm = arm

    def execute(self, gl):
        if self.arm == self.robot.leftArm:
            y_home = 0.2
        elif self.arm == self.robot.rightArm:
            y_home = -0.2

        rospy.loginfo("start moving to carrying pose")
        if self.arm.send_goal(0.18, y_home, 0.75, 0, 0, 0, 60):
            rospy.loginfo("arm at carrying pose")
            return 'succeeded'
        else:
            rospy.logerr("failed to go to the approach pose")
            return 'failed'

class PrepareGrasp(smach.State):
    def __init__(self, arm, robot, grab_entity_designator):
        """
        @param grab_entity_designator resolves to a PointStamped
        """
        smach.State.__init__(self, outcomes=['succeeded','failed'])

        self.arm = arm
        self.robot = robot
        self.grab_entity_designator = grab_entity_designator

    def execute(self, gl):

        # Move arm to desired joint coordinates (no need to wait)
        # ToDo: don't hardcode
        self.arm.send_joint_trajectory([[-0.2, -0.044, 0.69, 1.4, -0.13, 0.38, 0.42]])

        # If the z-position of the object is above a suitable threshold, move the spindle so that the object position can later be updated using the laser
        # Probably do this afterwards
        grabpoint = self.grab_entity_designator.resolve() #ToDo: 0.55 is a dirty hack, might be better

        if grabpoint:
            spindle_target = float(grabpoint.point.z) - 0.45

            rospy.logwarn("Temp spindle target")
            spindle_target = 0.4

            # ToDo: parameterize
            if (spindle_target > self.robot.spindle.lower_limit):
                self.robot.spindle.send_goal(spindle_target,timeout=5.0)
            # Else: there's no point in doing it any other way

        return 'succeeded'

class PrepareGraspSafe(smach.State):
    def __init__(self, arm, robot, grab_entity_designator):
        ''' Similar to PrepareGrasp but has a more elaborate joint trajectory to avoid hitting the table when standing close to it'''
        smach.State.__init__(self, outcomes=['succeeded','failed'])

        self.arm = arm
        self.robot = robot
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

class UpdateObjectPose(smach.State):
    def __init__(self, arm, robot, entity_center_designator):
        """Look at an entity to get a very accurate position of it (e.g. for grasping)
        @param entity_center_designator resolves to the PointStamped to look at in order to update the object's position
        """
        smach.State.__init__(self, outcomes=['succeeded','failed','target_lost'])

        self.arm = arm
        self.robot = robot
        self.entity_center_designator = entity_center_designator

    def execute(self, gl):

        rospy.loginfo("Trying to update object pose")

        ''' Remember current spindle position '''
        spindle_pos = self.robot.spindle.get_position()

        ''' Query reasoner '''
        try:
            target_point = self.entity_center_designator.resolve()
        except Exception, e:
            rospy.logerr(e)
            return 'target_lost'

        spindle_target = target_point.point.z + 0.5 -0.98 #ToDo: don't hardcode
        spindle_target = max(self.robot.spindle.lower_limit, spindle_target)
        spindle_target = min(self.robot.spindle.upper_limit, spindle_target)
        self.robot.spindle.send_goal(spindle_target, timeout = 5.0)

        self.robot.head.send_goal(target_point, keep_tracking=False, timeout=5.0, pan_vel=0.75, tilt_vel=0.75)

        self.robot.perception.toggle(["tabletop_segmentation"])
        rospy.logwarn("target point for roi = {0}".format(target_point))
        self.robot.perception.set_perception_roi(target_point, length_x=0.3, length_y=0.3, length_z=0.4)

        rospy.logwarn("Here we should keep track of the uncertainty, how can we do that? Now we simply use a sleep")
        timeout = 2.0
        rospy.loginfo("Waiting for {0} seconds for tabletop segmentation update".format(timeout))
        rospy.sleep(rospy.Duration(timeout))

        ''' Reset head and stop all perception stuff '''
        self.robot.perception.toggle([])

        if self.arm == self.robot.leftArm:
            self.robot.head.look_at_hand("left")
        elif self.arm == self.robot.rightArm:
            self.robot.head.look_at_hand("right")

        rospy.loginfo("Sending spindle to top for safety")
        spindle_pos = self.robot.spindle.upper_limit
        self.robot.spindle.send_goal(spindle_pos,timeout=40.0)

        return 'succeeded'

class UpdateDropPose(smach.State):
    def __init__(self, robot=None):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot

    def execute(self, gl):
        return 'failed'

class Handover_pose(smach.State):
    def __init__(self, arm, robot=None):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot
        self.arm = arm

    def execute(self, gl):
        if self.arm == self.robot.leftArm:
            y_home = 0.2
        elif self.arm == self.robot.rightArm:
            y_home = -0.2

        rospy.loginfo("start moving to handover pose")
        if self.arm.send_goal(0.6, y_home, 0.966, 0, 0, 0, 30):
            rospy.loginfo("arm at handover pose")
            return 'succeeded'
        else:
            rospy.logerr("failed to go to the handover pose")
            return 'failed'

@smach.cb_interface(outcomes=['done'])
def reset_arm(userdata, side):
    side.reset_arm()
    return 'done'

########################################### State Grab ###############################################

''' Grab machine which will switch hand (hence the A of arbitrary), if possible
    API is kept the same as with the GrabMachine, but now side is only considered to be a preference'''
class GrabMachineA(smach.StateMachine):
    def __init__(self, side, robot, grab_entity_designator):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot

        if isinstance(side, basestring):
            if side == "left":
                self.side = self.robot.leftArm
                self.backupside = self.robot.rightArm
            elif side == "right":
                self.side = self.robot.rightArm
                self.backupside = self.robot.leftArm
            else:
                print "Unknown arm side:" + str(side) + ". Defaulting to 'right'"
                self.side = self.robot.rightArm
        elif side == self.robot.leftArm:
            self.side = side
            self.backupside = self.robot.rightArm
        elif side == self.robot.rightArm:
            self.side = side
            self.backupside = self.robot.rightArm
        else:
            print "Unknown arm side:" + str(side) + ". Defaulting to 'right'"
            self.side = self.robot.rightArm
            self.backupside = self.robot.leftArm

        self.grab_entity_designator = grab_entity_designator

        with self:

            @smach.cb_interface(outcomes=['free', 'occupied'])
            def check_occupancy(userdata, arm):
                if arm.occupied_by:
                    return 'occupied'
                else:
                    return 'free'

            smach.StateMachine.add('CHECK_PRIMARY_OCCUPANCY', smach.CBState(check_occupancy, cb_args=[self.side]),
                                    transitions={'free'         : 'GRASP',
                                                 'occupied'     : 'CHECK_SECONDARY_OCCUPANCY'})

            smach.StateMachine.add('GRASP', GrabMachine(self.side, self.robot, self.grab_entity_designator),
                                    transitions={'succeeded'    : 'succeeded',
                                                 'failed'       : 'CHECK_SECONDARY_OCCUPANCY'})

            smach.StateMachine.add('CHECK_SECONDARY_OCCUPANCY', smach.CBState(check_occupancy, cb_args=[self.backupside]),
                                    transitions={'free'         : 'GRASP_BACKUP',
                                                 'occupied'     : 'failed'})

            smach.StateMachine.add('GRASP_BACKUP', GrabMachine(self.side, self.robot, self.grab_entity_designator),
                                    transitions={'succeeded'    : 'succeeded',
                                                 'failed'       : 'failed'})

class GrabMachine(smach.StateMachine):
    def __init__(self, side, robot, grab_entity_designator):
        """@param grab_entity_designator resolves to an entity to grab.
            Some child states require a (designator of) the PointStamped of that entity. PointStampedOfEntityDesignator does this conversion"""
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot

        if isinstance(side, basestring):
            if side == "left":
                self.side = self.robot.leftArm
            elif side == "right":
                self.side = self.robot.rightArm
            else:
                print "Unknown arm side:" + str(side) + ". Defaulting to 'right'"
                self.side = self.robot.rightArm
        else:
            self.side = side

        self.grab_entity_designator = grab_entity_designator
        self.grab_point_designator = PointStampedOfEntityDesignator(self.grab_entity_designator)

        '''check check input and output keys'''
        with self:
            smach.StateMachine.add('PREPARE_GRAB', PrepareGrasp(self.side, self.robot, self.grab_point_designator),
                        transitions={'succeeded'    :   'OPEN_GRIPPER',
                                     'failed'       :   'failed'})

            smach.StateMachine.add('OPEN_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.OPEN, timeout=0.0),
                        transitions={'succeeded'    :   'PREPARE_ORIENTATION',
                                     'failed'       :   'PREPARE_ORIENTATION'})

            # smach.StateMachine.add('PREPARE_ORIENTATION', PrepareOrientation(self.side, self.robot, self.grab_entity_designator),
            #             transitions={'orientation_succeeded':'UPDATE_OBJECT_POSE','orientation_failed':'UPDATE_OBJECT_POSE','abort':'failed','target_lost':'failed'})


            # Even if the update state fails, try to grasp anyway
            smach.StateMachine.add('UPDATE_OBJECT_POSE', UpdateObjectPose(self.side, self.robot, self.grab_point_designator),
                        transitions={'succeeded'    :   'PRE_GRASP',
                                     'failed'       :   'PRE_GRASP',
                                     'target_lost'  :   'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('PRE_GRASP', ArmToQueryPoint(self.robot, self.side, self.grab_point_designator, time_out=20, pre_grasp=True, first_joint_pos_only=True),
                        transitions={'succeeded'    :   'GRAB',
                                     'failed'       :   'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('GRAB', GrabOld(self.side, self.robot, self.grab_entity_designator),
                        transitions={'grab_succeeded':  'CLOSE_GRIPPER',
                                     'grab_failed'   :  'CLOSE_GRIPPER',
                                     'target_lost'   :  'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('CLOSE_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE, grab_entity_designator=self.grab_entity_designator),
                        transitions={'succeeded'    :   'LIFT',
                                     'failed'       :   'LIFT'})

            smach.StateMachine.add('LIFT', ArmToUserPose(self.side, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'RETRACT','failed':'RETRACT'})

            smach.StateMachine.add('RETRACT', ArmToUserPose(self.side, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'CARR_POS','failed':'CARR_POS'})

            smach.StateMachine.add('CARR_POS', Carrying_pose(self.side, self.robot),
                        transitions={'succeeded':'succeeded','failed':'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('CLOSE_GRIPPER_UPON_FAIL', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE, timeout=0.0),
                        transitions={'succeeded'    :   'failed',
                                     'failed'       :   'failed'})

class GrabMachineWithoutBase(smach.StateMachine):
    def __init__(self, side, robot, grab_entity_designator):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])

        self.robot = robot

        if isinstance(side, basestring):
            if side == "left":
                self.side = self.robot.leftArm
            elif side == "right":
                self.side = self.robot.rightArm
            else:
                print "Unknown arm side:" + str(side) + ". Defaulting to 'right'"
                self.side = self.robot.rightArm
        else:
            self.side = side

        self.grab_entity_designator = grab_entity_designator
        '''check check input and output keys'''
        with self:

            smach.StateMachine.add('UPDATE_OBJECT_POSE', UpdateObjectPose(self.side, self.robot, self.grab_entity_designator),
                        transitions={'succeeded'    :   'PREPARE_GRAB',
                                     'failed'       :   'PREPARE_GRAB',
                                     'target_lost'  :   'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('PREPARE_GRAB', PrepareGraspSafe(self.side, self.robot, self.grab_entity_designator),
                        transitions={'succeeded'    :   'OPEN_GRIPPER',
                                     'failed'       :   'failed'})

            smach.StateMachine.add('OPEN_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.OPEN, timeout=0.0),
                        transitions={'succeeded'    :   'PRE_GRASP',
                                     'failed'       :   'PRE_GRASP'})

            smach.StateMachine.add('PRE_GRASP', ArmToQueryPoint(self.robot, self.side, self.grab_entity_designator, time_out=20, pre_grasp=True, first_joint_pos_only=True),
                        transitions={'succeeded'    :   'GRAB',
                                     'failed'       :   'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('GRAB', GrabOld(self.side, self.robot, self.grab_entity_designator),
                        transitions={'grab_succeeded':  'CLOSE_GRIPPER',
                                     'grab_failed'   :  'CLOSE_GRIPPER',
                                     'target_lost'   :  'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('CLOSE_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE, grab_entity_designator=self.grab_entity_designator),
                        transitions={'succeeded'    :   'LIFT',
                                     'failed'       :   'LIFT'})

            smach.StateMachine.add('LIFT', ArmToUserPose(self.side, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'RETRACT','failed':'RETRACT'})

            smach.StateMachine.add('RETRACT', ArmToUserPose(self.side, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'CARR_POS','failed':'CARR_POS'})

            smach.StateMachine.add('CARR_POS', Carrying_pose(self.side, self.robot),
                        transitions={'succeeded':'succeeded','failed':'CLOSE_GRIPPER_UPON_FAIL'})

            smach.StateMachine.add('CLOSE_GRIPPER_UPON_FAIL', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE, timeout=0.0),
                        transitions={'succeeded'    :   'failed',
                                     'failed'       :   'failed'})

class GrabOld(smach.State):
    def __init__(self, side, robot, grab_point_designator):
        smach.State.__init__(self, outcomes=['grab_succeeded','grab_failed','target_lost'])

        self.side = side.resolve() if hasattr(side, "resolve") else side
        self.robot = robot
        self.grab_point_designator = grab_point_designator

        if self.side == self.robot.leftArm:
            self.end_effector_frame_id = "/amigo/grippoint_left"
            self.ar_frame_id = "/hand_marker_left"
        elif self.side == self.robot.rightArm:
            self.end_effector_frame_id = "/amigo/grippoint_right"
            self.ar_frame_id = "/hand_marker_right"

    def execute(self, gl):
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
            self.side.send_delta_goal(0.05,0.0,0.0,0.0,0.0,0.0, timeout=5.0, frame_id=self.end_effector_frame_id, pre_grasp = False)
            self.robot.speech.speak("Let me have a closer look", block=False)

        ar_point_grippoint = transformations.tf_transform(ar_point, self.ar_frame_id, self.end_effector_frame_id, tf_listener=self.robot.tf_listener)
        rospy.loginfo("AR marker in end-effector frame = {0}".format(ar_point_grippoint))

        ''' Transform target position to grippoint frame '''
        target_position_grippoint = transformations.tf_transform(target_position, "/map", self.end_effector_frame_id, tf_listener=self.robot.tf_listener)
        rospy.loginfo("Target position in end-effector frame = {0}".format(target_position_grippoint))

        ''' Compute difference = delta (only when both transformations have succeeded) and correct for offset ar_marker and grippoint '''
        if not (ar_point_grippoint == None or target_position_grippoint == None):
            target_position_delta = msgs.Point(target_position_grippoint.x - ar_point_grippoint.x + self.side.markerToGrippointOffset.x,
                target_position_grippoint.y - ar_point_grippoint.y + self.side.markerToGrippointOffset.y,
                target_position_grippoint.z - ar_point_grippoint.z + self.side.markerToGrippointOffset.z)
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
            if self.side.send_goal(target_position_bl.x, target_position_bl.y, target_position_bl.z, 0, 0, 0, 120, pre_grasp = True):
                rospy.loginfo("arm at object")
            else:
                rospy.logerr("Goal unreachable: {0}".format(target_position_bl).replace("\n", " "))
                self.robot.speech.speak("I am sorry but I cannot move my arm to the object position", block=False)
                return 'grab_failed'
        else:
            self.robot.speech.speak("Let's go", block=False)
            if self.side.send_delta_goal(target_position_delta.x, target_position_delta.y, target_position_delta.z,
                                        0, 0, 0, 120, frame_id=self.end_effector_frame_id, pre_grasp = True):
                rospy.loginfo("arm at object")
            else:
                rospy.logerr("Goal unreachable: {0}".format(target_position_bl).replace("\n", " "))
                self.robot.speech.speak("I am sorry but I cannot move my arm to the object position", block=False)
                return 'grab_failed'

        self.robot.head.reset_position(timeout=0.0)
        return 'grab_succeeded'


class Human_handover(smach.StateMachine):
    def __init__(self, side, robot=None):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot

        with self:
            smach.StateMachine.add("POSE", Carrying_pose(self.side,self.robot),
                            transitions={'succeeded':'OPEN_BEFORE_INSERT','failed':'OPEN_BEFORE_INSERT'})

            smach.StateMachine.add( 'OPEN_BEFORE_INSERT', SetGripper(robot, robot.leftArm, gripperstate=0), #open
                                transitions={'succeeded'    :   'SAY1',
                                             'failed'       :   'SAY1'})

            smach.StateMachine.add("SAY1", Say(self.robot,'Please hand over the object by sliding it in my gripper'),
                            transitions={'spoken':'CLOSE_AFTER_INSERT'})

            smach.StateMachine.add( 'CLOSE_AFTER_INSERT', SetGripper(robot, robot.leftArm, gripperstate=1), #close
                                transitions={'succeeded'    :   'succeeded',
                                             'failed'       :   'failed'})

class PlaceObject(smach.StateMachine):
    def __init__(self, side, robot, placement_query, dropoff_height_offset=0.1):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed','target_lost'])

        self.robot = robot

        if isinstance(side, basestring):
            if side == "left":
                self.side = self.robot.leftArm
            elif side == "right":
                self.side = self.robot.rightArm
            else:
                print "Unknown arm side:" + str(side) + ". Defaulting to 'right'"
                self.side = self.robot.rightArm
        else:
            self.side = side

        self.dropoff_query = Compound("current_dropoff_point", "X","Y","Z")

        self.placement_query = placement_query
        self.dropoff_height_offset = dropoff_height_offset

        with self:
            smach.StateMachine.add('DETERMINE_DROPOFF_POINT', DetermineDropoffPoint(self.robot, self.placement_query),
                        transitions={'succeeded'    :   'PREPARE_PLACEMENT',
                                     'all_points_tried'       :   'FAILURE_RESET_DROPOFF',
                                     'no_dropoff_points'       :   'FAILURE_RESET_DROPOFF'})

            smach.StateMachine.add('PREPARE_PLACEMENT', PrepareGrasp(self.side, self.robot, self.dropoff_query),
                        transitions={'succeeded'    :   'PREPARE_ORIENTATION',
                                     'failed'       :   'DETERMINE_DROPOFF_POINT'})

            smach.StateMachine.add('PREPARE_ORIENTATION', PrepareOrientation(self.side, self.robot, self.dropoff_query),
                        transitions={'orientation_succeeded':'PRE_POSITION','orientation_failed':'PRE_POSITION','abort':'failed','target_lost':'DETERMINE_DROPOFF_POINT'})

            smach.StateMachine.add('PRE_POSITION', ArmToQueryPoint(self.robot, self.side, self.dropoff_query, time_out=20, pre_grasp=True, first_joint_pos_only=False),
                        transitions={'succeeded'    :   'POSITION',
                                     'failed'       :   'DETERMINE_DROPOFF_POINT'})
            # When pre-position fails, there is no use in dropping the object
            smach.StateMachine.add('POSITION', ArmToUserPose(   self.side, 0.0, 0.0, -self.dropoff_height_offset, 0.0, 0.0 , 0.0,
                                                                time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'OPEN_GRIPPER','failed':'OPEN_GRIPPER'})
            # When position fails, it might be tried
            smach.StateMachine.add('OPEN_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.OPEN),
                        transitions={'succeeded'    :   'LIFT',
                                     'failed'       :   'LIFT'})

            smach.StateMachine.add('LIFT', ArmToUserPose(self.side, 0.0, 0.0, self.dropoff_height_offset, 0.0, 0.0 , 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'RETRACT','failed':'RETRACT'})

            smach.StateMachine.add('RETRACT', ArmToUserPose(self.side, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'CARR_POS','failed':'CARR_POS'})

            smach.StateMachine.add('CARR_POS', Carrying_pose(self.side, self.robot),
                        transitions={'succeeded':'CLOSE_GRIPPER','failed':'CLOSE_GRIPPER'})

            smach.StateMachine.add('CLOSE_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE, timeout=0.0),
                        transitions={'succeeded'    :   'RESET_DROPOFF',
                                     'failed'       :   'RESET_DROPOFF'})

            smach.StateMachine.add("FAILURE_RESET_DROPOFF",
                        ResetAfterDropoff(self.side, self.robot),
                        transitions={'succeeded':'failed'})

            smach.StateMachine.add("RESET_DROPOFF",
                        ResetAfterDropoff(self.side, self.robot),
                        transitions={'succeeded':'succeeded'})


class ResetAfterDropoff(smach.StateMachine):
    def __init__(self, side, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded'])

        self.robot = robot

        if isinstance(side, basestring):
            if side == "left":
                self.side = self.robot.leftArm
            elif side == "right":
                self.side = self.robot.rightArm
            else:
                print "Unknown arm side:" + str(side) + ". Defaulting to 'right'"
                self.side = self.robot.rightArm
        else:
            self.side = side

        with self:
            smach.StateMachine.add('RESET_ARM',
                        ArmToJointPos(self.robot, self.side, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), #Copied from demo_executioner NORMAL
                        transitions={   'done':'RESET_TORSO',
                                      'failed':'RESET_TORSO'    })

            smach.StateMachine.add('RESET_TORSO',
                        ResetTorso(self.robot),
                        transitions={'succeeded':'RETRACT_CURRENT_DROPOFF_POINT',
                                     'failed'   :'RETRACT_CURRENT_DROPOFF_POINT'})

            smach.StateMachine.add("RETRACT_CURRENT_DROPOFF_POINT",
                                reasoning.Retract_facts(robot, [Compound("current_dropoff_point", "X", "Y", "Z")]),
                                transitions={'retracted':'RETRACT_PREVIOUS_DROPOFF_POINTS'})

            smach.StateMachine.add("RETRACT_PREVIOUS_DROPOFF_POINTS",
                                reasoning.Retract_facts(robot, [Compound("previous_dropoff_point", "X", "Y", "Z")]),
                                transitions={'retracted':'succeeded'})


class DetermineDropoffPoint(smach.State):
    def __init__(self, robot, placement_query):
        smach.State.__init__(self, outcomes=["succeeded", "all_points_tried", "no_dropoff_points"])

        self.robot = robot
        self.placement_query = placement_query

    def execute(self, userdata=None):

        list_previous_dropoff_points = []

        # Check if there is already a current dropoff point
        answers_dropoff_point = self.robot.reasoner.query(Compound("current_dropoff_point", "X","Y","Z"))

        # If this is the case, move it to a fact "previous_dropoff_point"
        if answers_dropoff_point:
            answer_dropoff_point = answers_dropoff_point[0]
            self.robot.reasoner.query(Compound("assert", Compound("previous_dropoff_point", float(answer_dropoff_point["X"]), float(answer_dropoff_point["Y"]), float(answer_dropoff_point["Z"]))))

        answers_previous_dropoff_points = self.robot.reasoner.query(Compound("previous_dropoff_point", "X","Y","Z"))
        self.robot.reasoner.query(Compound("retractall", Compound("current_dropoff_point", "X", "Y", "Z")))

        if answers_previous_dropoff_points:
            for i in range(0,len(answers_previous_dropoff_points)):
                answer = answers_previous_dropoff_points[i]
                x,y,z = float(answer["X"]), float(answer["Y"]), float(answer["Z"])
                list_previous_dropoff_points.append([x,y,z])

        answers = self.robot.reasoner.query(self.placement_query)

        if answers:
            for i in range(0,len(answers)):
                answer = answers[i]

                rospy.loginfo("Answer(0) = {0}".format(answer))
                x,y,z = float(answer["X"]), float(answer["Y"]), float(answer["Z"])

                if [x,y,z] not in list_previous_dropoff_points:
                    self.robot.reasoner.query(Compound("assert", Compound("current_dropoff_point", x,y,z)))
                    rospy.loginfo("Current Dropoff Point is x = {0}, y = {1}, z = {2}".format(x,y,z))
                    return 'succeeded'

            self.robot.reasoner.query(Compound("retractall", Compound("current_dropoff_point", "X", "Y", "Z")))
            self.robot.reasoner.query(Compound("retractall", Compound("previous_dropoff_point", "X", "Y", "Z")))
            return 'all_points_tried'

        else:
            #rospy.logerr("Cannot get target from reasoner, query = {0}".format(self.grab_entity_designator))
            return 'no_dropoff_points'


class PlaceObjectWithoutBase(smach.StateMachine):
    def __init__(self, side, robot, placement_query, dropoff_height_offset=0.1):
        smach.StateMachine.__init__(self, outcomes=['succeeded','failed','target_lost'])

        self.robot = robot

        if isinstance(side, basestring):
            if side == "left":
                self.side = self.robot.leftArm
            elif side == "right":
                self.side = self.robot.rightArm
            else:
                print "Unknown arm side:" + str(side) + ". Defaulting to 'right'"
                self.side = self.robot.rightArm
        else:
            self.side = side


        self.placement_query = placement_query
        self.dropoff_height_offset = dropoff_height_offset

        with self:
            smach.StateMachine.add('PREPARE_PLACEMENT', PrepareGraspSafe(self.side, self.robot, self.placement_query),
                        transitions={'succeeded'    :   'PRE_POSITION',
                                     'failed'       :   'failed'})

            smach.StateMachine.add('PRE_POSITION', ArmToQueryPoint(self.robot, self.side, self.placement_query, time_out=20, pre_grasp=True, first_joint_pos_only=False),
                        transitions={'succeeded'    :   'POSITION',
                                     'failed'       :   'failed'})
            # When pre-position fails, there is no use in dropping the object
            smach.StateMachine.add('POSITION', ArmToUserPose(   self.side, 0.0, 0.0, -self.dropoff_height_offset, 0.0, 0.0 , 0.0,
                                                                time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'OPEN_GRIPPER','failed':'OPEN_GRIPPER'})
            # When position fails, it might be tried
            smach.StateMachine.add('OPEN_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.OPEN),
                        transitions={'succeeded'    :   'LIFT',
                                     'failed'       :   'LIFT'})

            smach.StateMachine.add('LIFT', ArmToUserPose(self.side, 0.0, 0.0, self.dropoff_height_offset, 0.0, 0.0 , 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'RETRACT','failed':'RETRACT'})

            smach.StateMachine.add('RETRACT', ArmToUserPose(self.side, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, time_out=20, pre_grasp=False, frame_id="/amigo/base_link", delta=True),
                        transitions={'succeeded':'CARR_POS','failed':'CARR_POS'})

            smach.StateMachine.add('CARR_POS', Carrying_pose(self.side, self.robot),
                        transitions={'succeeded':'CLOSE_GRIPPER','failed':'CLOSE_GRIPPER'})

            smach.StateMachine.add('CLOSE_GRIPPER', SetGripper(self.robot, self.side, gripperstate=ArmState.CLOSE, timeout=0.0),
                        transitions={'succeeded'    :   'RESET_ARM',
                                     'failed'       :   'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM',
                        ArmToJointPos(self.robot, self.side, (-0.0830 , -0.2178 , 0.0000 , 0.5900 , 0.3250 , 0.0838 , 0.0800)), #Copied from demo_executioner NORMAL
                        transitions={   'done':'RESET_TORSO',
                                      'failed':'RESET_TORSO'    })

            smach.StateMachine.add('RESET_TORSO',
                        ResetTorso(self.robot),
                        transitions={'succeeded':'succeeded',
                                     'failed'   :'succeeded'})

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


class DropObject(smach.StateMachine):
    def __init__(self, side, robot, placement_query, dropoff_height_offset=0.1):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'failed', 'target_lost'])
        self.side = side
        self.robot = robot
        self.placement_query = placement_query
        self.dropoff_height_offset = dropoff_height_offset

        otherarm = self.robot.rightArm
        if otherarm == side:
            otherarm = self.robot.leftArm

        # removes previous defined dropoff points
        self.robot.reasoner.query(Compound("retractall", Compound("current_dropoff_point", "X", "Y", "Z")))
        self.robot.reasoner.query(Compound("retractall", Compound("previous_dropoff_point", "X", "Y", "Z")))

        with self:
            smach.StateMachine.add( "RESET_OTHER_ARM",
                                    ArmToJointPos(robot, otherarm, otherarm.RESET_POSE, timeout=1.0),
                                    transitions={   'done'                  : 'PLACE_OBJECT',
                                                    'failed'                : 'PLACE_OBJECT'}) #This is risky, but try anyways

            smach.StateMachine.add( 'PLACE_OBJECT',
                                    PlaceObject(self.side, self.robot, self.placement_query, self.dropoff_height_offset),
                                    transitions={'succeeded'    : 'succeeded',
                                                 'failed'       : 'SAY_HUMAN_HANDOVER',
                                                 'target_lost'  : 'SAY_HUMAN_HANDOVER' })

            smach.StateMachine.add( 'SAY_HUMAN_HANDOVER',
                                    Say(robot, [ "I am terribly sorry, but I cannot place the object. Can you please take it from me",
                                                        "My apologies, but i cannot place the object. Would you be so kind to take it from me"], block=False),
                                     transitions={   'spoken':'HANDOVER_TO_HUMAN'})

            smach.StateMachine.add( 'HANDOVER_TO_HUMAN',
                                    HandoverToHuman(self.side, self.robot),
                                    transitions={'succeeded'    : 'succeeded',
                                                 'failed'       : 'failed'})



class SetGripper(smach.State):
    def __init__(self, robot, side, gripperstate='open', drop_from_frame=None, grab_entity_designator=None, timeout=10):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.side = side
        self.robot = robot
        self.gripperstate = gripperstate
        self.grab_entity_designator = grab_entity_designator
        self.timeout = timeout
        if self.side == self.robot.leftArm:
            self.end_effector_frame_id = "/amigo/grippoint_left"
        elif self.side == self.robot.rightArm:
            self.end_effector_frame_id = "/amigo/grippoint_right"

    def execute(self, userdata):
        ''' If needs attaching to gripper, the grab_entity_designator is used '''
        if self.grab_entity_designator:
            try:
                entity = self.grab_entity_designator.resolve()
                try:
                    self.robot.reasoner.attach_object_to_gripper(entity.id, self.end_effector_frame_id, True)
                except KeyError, ke:
                    rospy.logerr("Could not attach object to gripper, do not know which ID: {0}".format(ke))

                #TODO: the designator required by this state should resolve to an entity and not to its ID.
                self.side.occupied_by = entity
            except Exception, e:
                rospy.logerr("Could not resolve {0}: {1}".format(self.grab_entity_designator, e))

        if self.side.send_gripper_goal(self.gripperstate, timeout=self.timeout):
            result = True
        else:
            result = False

        # ToDo: make sure things can get attached to the gripper in this state. Use userdata?
        if self.gripperstate == ArmState.OPEN:
            self.robot.reasoner.detach_all_from_gripper(self.end_effector_frame_id)
            self.side.occupied_by = None

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

class ResetTorso(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.robot = robot

    def execute(self, userdata):
        if self.robot.spindle.reset():
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

