import rospy
from visualization_msgs.msg import Marker

def visualize_location(base_pose, location):
    '''
    Visualize a marker on the base_pose with the text 'location' and rotated to the correct side.
    :param base_pose: PoseStamped of where the robot is at that moment
    :param location: The name of the location as a label
    :return:
    '''

    pub = rospy.Publisher("/restaurant_waypoints", Marker, queue_size=10)
    m = Marker()
    if location == "one":
        m.id = 1
        m.color.r = 1
    if location == "two":
        m.id = 2
        m.color.g = 1
    if location == "three":
        m.id = 3
        m.color.b = 1
    m.color.a = 1
    m.pose = base_pose.pose
    m.header = base_pose.header
    m.type = 0  # Arrow
    m.scale.x = 1.0
    m.scale.y = 0.2
    m.scale.z = 0.2
    m.action = 0
    m.ns = "arrow"
    pub.publish(m)
    m.type = 9
    m.text = location
    m.ns = "text"
    m.pose.position.z = 0.5
    pub.publish(m)
    # Store waypoint in world model
    print "Asserting waypoint %s to world model" % location
    print "\n\n\n\nCURRENT BASE POSE:\n\n\n"
    print base_pose
    print "\n\n\n"

