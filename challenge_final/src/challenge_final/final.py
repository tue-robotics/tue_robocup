# ROS
import PyKDL as kdl
import robot_smach_states as states
import rospy
import smach
from robocup_knowledge import knowledge_loader
from robot_skills.util import kdl_conversions

from track_operator import TrackFace

# Load the knowledge
knowledge = knowledge_loader.load_knowledge("challenge_final")


def get_frame_from_vector(x_vector, origin):
    unit_z = kdl.Vector(0, 0, 1)
    unit_z_cross_diff = (unit_z * x_vector) / (unit_z * x_vector).Norm()
    y_vector = x_vector * unit_z_cross_diff
    z_vector = x_vector * y_vector

    rotation = kdl.Rotation(x_vector, y_vector, z_vector)
    translation = origin.vector

    frame_stamped = kdl_conversions.FrameStamped(kdl.Frame(rotation, translation), origin.frame_id)
    return frame_stamped


def get_ray_trace_from_closest_person(robot, arm_norm_threshold=0.1, upper_arm_norm_threshold = 0.7):
    persons = []
    while not rospy.is_shutdown() and not persons:
        persons = robot.head.detect_persons_3d()

        # TODO: Constraints on the detected person!

        rospy.sleep(1)

    # Take the first person
    # TODO: Optimization
    person = persons[0]

    left_wrist = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["left_wrist"]).projectToFrame("/map",
                                                                                                          robot.tf_listeners)
    right_wrist = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["right_wrist"]).projectToFrame("/map",
                                                                                                            robot.tf_listeners)
    left_elbow = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["left_elbow"]).projectToFrame("/map",
                                                                                                          robot.tf_listeners)
    right_elbow = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["left_elbow"]).projectToFrame("/map",
                                                                                                           robot.tf_listeners)
    left_shoulder = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["left_shoulder"]).projectToFrame("/map",
                                                                                                                robot.tf_listeners)
    right_shoulder = kdl_conversions.kdlVectorStampedFromPointStampedMsg(person["left_shoulder"]).projectToFrame("/map",
                                                                                                                 robot.tf_listeners)

    left_lower_arm_vector = (left_wrist - left_elbow) / (left_wrist - left_elbow).Norm()
    left_upper_arm_vector = (left_elbow - left_shoulder) / (left_elbow - left_shoulder).Norm()

    right_lower_arm_vector = (right_wrist - right_elbow) / (right_wrist - right_elbow).Norm()
    right_upper_arm_vector = (right_elbow - right_shoulder) / (right_elbow - right_shoulder).Norm()

    right_frame = get_frame_from_vector(right_lower_arm_vector, right_wrist.vector)
    left_frame = get_frame_from_vector(left_lower_arm_vector, left_wrist.vector)

    left_arm_norm = (left_lower_arm_vector * left_upper_arm_vector).Norm()
    right_arm_norm = (right_lower_arm_vector * right_upper_arm_vector).Norm()

    rospy.loginfo("Arn norm threshold: %.2f", arm_norm_threshold)
    rospy.loginfo("Left arm norm: %.2f", left_arm_norm)
    rospy.loginfo("Right arm norm: %.2f", right_arm_norm)

    left_upper_arm_norm = (left_upper_arm_vector * kdl.Vector(0, 0, 1)).Norm()
    right_upper_arm_norm = (right_upper_arm_vector * kdl.Vector(0, 0, 1)).Norm()

    rospy.loginfo("Upper arm norm threshold: %.2f", upper_arm_norm_threshold)
    rospy.loginfo("Upper left arm norm: %.2f", left_upper_arm_norm)
    rospy.loginfo("Upper right arm norm: %.2f", right_upper_arm_norm)

    # Check if arms are pointing
    left_arm_valid = True
    right_arm_valid = True

    # Constraint based on pointing sideways
    if left_upper_arm_norm < upper_arm_norm_threshold:
        rospy.loginfo("Rejecting left arm because of not pointing sideways ..")
        left_arm_valid = False
    if right_upper_arm_norm < upper_arm_norm_threshold:
        rospy.loginfo("Rejecting right arm because of not pointing sideways ..")
        right_arm_valid = False

    # Constraint based on parralelliness
    if left_arm_valid and left_arm_norm > right_arm_norm:
        rospy.loginfo("Rejecting left arm because of norm threshold ...")
        left_arm_valid = False
    if right_arm_valid and right_arm_norm > arm_norm_threshold:
        rospy.loginfo("Rejecting right arm because of norm threshold ...")
        right_arm_valid = False

    # Optimize
    if left_arm_valid and right_arm_valid:
        if left_arm_norm > right_arm_norm
            rospy.loginfo("Right arm is pointing the most, using this one")
            frame = right_frame
        else:
            rospy.loginfo("Left arm is pointing the most, using this one")
            frame = left_frame
    if left_arm_valid:
        frame = left_frame
    if right_arm_valid:
        frame = right_frame
    else:
        rospy.loginfo("No valid arms found ...")
        return None

    return robot.head.ray_trace(kdl_conversions.kdlFrameStampedToPoseStampedMsg(frame))


class ChallengeFinal(smach.StateMachine):
    """ State machine for the final challenge """

    def __init__(self, robot):
        """ Constructor
        :param robot: robot object
        """
        smach.StateMachine.__init__(self, outcomes=["Done", "Aborted"])

        with self:
            # Start challenge
            # smach.StateMachine.add("START_CHALLENGE",
            #                       states.StartChallengeRobust(robot=robot, initial_pose=knowledge.initial_pose),
            #                       transitions={"Done": "LEARN_OPERATOR",
            #                                    "Aborted": "Aborted",
            #                                    "Failed": "Aborted"})

            # Learn operator
            smach.StateMachine.add("LEARN_OPERATOR",
                                   states.LearnPerson(robot, person_name="operator", nr_tries=5),
                                   transitions={"succeeded": "TRACK_OPERATOR",
                                                "failed": "Done"})

            # TrackOperator state
            smach.StateMachine.add("TRACK_OPERATOR",
                                   TrackFace(robot=robot),
                                   transitions={"aborted": "Done",
                                                "lost": "Done"})
