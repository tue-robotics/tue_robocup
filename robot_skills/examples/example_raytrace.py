#!/usr/bin/env python
import rospy
from ed_sensor_integration_msgs.srv import RayTrace
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf2_ros

from robot_skills import perception

rospy.init_node('ray_trace_example')

p = perception.Perception("hero", None, None, camera_base_ns="hero/head_rgbd_sensor")

srv_proxy = rospy.ServiceProxy("/hero/ed/ray_trace", RayTrace)

tf_buffer = tf2_ros.Buffer()
tf = tf2_ros.TransformListener(tf_buffer)

while not rospy.is_shutdown():
    rgb, depth, depth_info = p.get_rgb_depth_caminfo()
    if rgb:
        persons = p.detect_person_3d(*p.get_rgb_depth_caminfo())
        for person in persons:
            if "is_pointing" in person.tags:
                try:
                    map_pose = tf_buffer.transform(PoseStamped(
                        header=Header(
                            frame_id="head_rgbd_sensor_rgb_optical_frame",
                            stamp=rospy.Time.now() - rospy.Duration.from_sec(0.5)
                        ),
                        pose=person.pointing_pose
                    ), "map")
                    srv_proxy(raytrace_pose=map_pose)

                except Exception as e:
                    rospy.logerr("Could not get ray trace from closest person: {}".format(e))
