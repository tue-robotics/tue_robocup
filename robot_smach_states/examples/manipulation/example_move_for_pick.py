# System
import argparse
import functools
import threading

# ROS
import geometry_msgs.msg
import PyKDL as kdl
import rospy
import visualization_msgs.msg

# TU/e Robotics
import robot_skills
from robot_skills.util.kdl_conversions import FrameStamped
from robot_skills.get_robot import get_robot

# Robot Smach States
import robot_smach_states.util.designators as ds
from robot_smach_states import MoveToGrasp


def _clicked_point_callback(entity_pose, marker_pub, event, msg):
    # type: (FrameStamped, rospy.Publisher, threading.Event, geometry_msgs.msg.PointStamped) -> None
    """
    Callback for clicked points. Stores the position in the provided entity pose and sets the threading event.
    N.B.: if the event is already set, this callback does nothing.
    """
    if event.is_set():
        rospy.logwarn("Event is already set, discarding clicked point")
        return

    marker_msg = visualization_msgs.msg.Marker()
    marker_msg.header.frame_id = "map"
    marker_msg.header.stamp = rospy.Time.now()
    marker_msg.id = 666
    marker_msg.type = visualization_msgs.msg.Marker.SPHERE
    marker_msg.action = visualization_msgs.msg.Marker.ADD
    marker_msg.pose.position = msg.point
    marker_msg.pose.orientation.w = 1.0
    marker_msg.scale.x = 0.05
    marker_msg.scale.y = 0.05
    marker_msg.scale.z = 0.05
    marker_msg.color.r = 1.0
    marker_msg.color.a = 1.0
    marker_msg.lifetime = rospy.Duration(30.0)
    marker_pub.publish(marker_msg)

    entity_pose.frame.p.x(msg.point.x)
    entity_pose.frame.p.y(msg.point.y)
    entity_pose.frame.p.z(msg.point.z)
    event.set()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Listens to the '/clicked_point' topic, assumes there's an object"
                                                 "at that position, computes the corresponding grasp offset and"
                                                 "drives to the pick pose")
    parser.add_argument("--robot", default="hero", help="Robot name (amigo, hero, sergio)")
    args = parser.parse_args()

    rospy.init_node("test_move_to_grasp")

    robot_skills.robot.CONNECTION_TIMEOUT = 1.0
    robot = get_robot(args.robot)

    entity_id = "test_item"
    pose = FrameStamped(frame=kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0)), frame_id="/map")
    marker_pub = rospy.Publisher("clicked_entity_pose", visualization_msgs.msg.Marker, queue_size=1)
    threading_event = threading.Event()

    rospy.Subscriber(
        "/clicked_point",
        geometry_msgs.msg.PointStamped,
        functools.partial(_clicked_point_callback, pose, marker_pub, threading_event)
    )

    # Start main loop
    while not rospy.is_shutdown():
        threading_event.clear()
        rospy.loginfo("Waiting for the user to click on a point in rviz")
        threading_event.wait()
        if rospy.is_shutdown():
            break
        rospy.loginfo("Setting test entity to {}".format(pose.frame.p))

        robot.ed.update_entity(id=entity_id, frame_stamped=pose)
        item = ds.EdEntityDesignator(robot, id=entity_id)
        arm = ds.UnoccupiedArmDesignator(robot, {})

        move_state = MoveToGrasp(robot, item, arm)
        move_state.execute()

