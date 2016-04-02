#! /usr/bin/env python

#
import math
import PyKDL as kdl

# ROS
import rospy

# Tech United Eindhoven
from robot_smach_states.util import geometry_helpers

def in_room(robot, pose):
    """ Checks in which room a pose is located.
    :param robot: Robot object, necessary to query ed
    :param pose: geometry_msgs.msg.Pose that is checked. Note this MUST be defined in /map frame, since this is the
     default frame of ED
    :return: string with the room in which the pose is located. If it is in no room, None will be returned
    """
    # Convert the pose into a KDL vector (we don't need the orientation)
    testposition = kdl.Vector(pose.position.x, pose.position.y, pose.position.z)

    # For each room:
    rooms = robot.ed.get_entities(type='room', parse=True)
    for room in rooms:

        # Filter entrance and exit
        if room.id in ['entrance', 'exit']:
            continue

        # Create a convex hull to check
        convex_hull = []
        for testarea in room.data['areas']:
            ''' See if the area is in the list of inspection areas '''
            if testarea['name'] == 'in':
                ''' Check if we have a shape '''
                if 'shape' not in testarea:
                    rospy.logwarn("No shape in area {0}".format(testarea['name']))
                    continue
                ''' Check if length of shape equals one '''
                if not len(testarea['shape']) == 1:
                    rospy.logwarn("Shape of area {0} contains multiple entries, don't know what to do".format(testarea['name']))
                    continue
                ''' Check if the first entry is a box '''
                if not 'box' in testarea['shape'][0]:
                    rospy.logwarn("No box in {0}".format(testarea['name']))
                    continue
                box = testarea['shape'][0]['box']
                if 'min' not in box or 'max' not in box:
                    rospy.logwarn("Box in {0} either does not contain min or max".format(testarea['name']))
                    continue
                # Now we're sure to have the correct bounding box
                roompose = kdl.Frame(kdl.Rotation.Quaternion(room.pose.orientation.x,
                                                             room.pose.orientation.y,
                                                             room.pose.orientation.z,
                                                             room.pose.orientation.w),
                                     kdl.Vector(room.pose.position.x,
                                                room.pose.position.y,
                                                room.pose.position.z))
                hpose = roompose * kdl.Frame(kdl.Rotation(),
                                             kdl.Vector(box['min']['x'], box['min']['y'], box['min']['z']))
                convex_hull.append(hpose.p)
                hpose = roompose * kdl.Frame(kdl.Rotation(),
                                             kdl.Vector(box['max']['x'], box['min']['y'], box['min']['z']))
                convex_hull.append(hpose.p)
                hpose = roompose * kdl.Frame(kdl.Rotation(),
                                             kdl.Vector(box['max']['x'], box['max']['y'], box['min']['z']))
                convex_hull.append(hpose.p)
                hpose = roompose * kdl.Frame(kdl.Rotation(),
                                             kdl.Vector(box['min']['x'], box['max']['y'], box['min']['z']))
                convex_hull.append(hpose.p)

                if geometry_helpers.isPointInsideHull(testposition, convex_hull):
                    return room.id

    return None


if __name__ == "__main__":
    rospy.init_node('test_in_room')

    import sys
    import robot_skills
    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    else:
        print "unknown robot"
        sys.exit()

    robot = Robot()

    import geometry_msgs.msg as gm
    tp = gm.Pose()
    for x in [-1, 1, 3, 5]:
        for y in [-8, -6, -4, -2, 0, 2, 4, 6]:

            tp.position.x = x
            tp.position.y = y
            print in_room(robot, tp)
