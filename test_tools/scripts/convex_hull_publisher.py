#! /usr/bin/env python

import rospy
from tf import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from ed_msgs.srv import SimpleQuery, SimpleQueryRequest, SimpleQueryResponse
# from ed_msgs.msg import EntityInfo
# from std_msgs.msg import Header

if __name__ == "__main__":
    node = rospy.init_node("convex_hull_publisher")

    tf_pub = TransformBroadcaster()
    rospy.wait_for_service("/amigo/ed/simple_query")
    ed_proxy = rospy.ServiceProxy("/amigo/ed/simple_query", SimpleQuery)

    rate = rospy.Rate(10)

    req = SimpleQueryRequest()
    req.id = "dinner_table"
    # resp = ed_proxy.call(req)
    resp = SimpleQueryResponse

    counter = 0
    while not rospy.is_shutdown():
        counter = counter % 50
        if counter == 0:
            resp = ed_proxy.call(req)
        for e in resp.entities:
            # e = EntityInfo()
            msg = TransformStamped()
            msg.header.frame_id = e.id
            msg.header.stamp = rospy.Time.now()
            msg.transform.rotation = Quaternion(0, 0, 0, 1)
            i = 1
            for point in e.convex_hull:
                msg.child_frame_id = e.id + str(i)
                msg.transform.translation = Vector3(point.x, point.y, point.z)
                tf_pub.sendTransformMessage(msg)
                i = i + 1

        counter = counter + 1
        rate.sleep()


