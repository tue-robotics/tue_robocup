#! /usr/bin/env python
import roslib; roslib.load_manifest('challenge_cleanup')
import rospy

import smach

from robot_skills.amigo import Amigo
import robot_smach_states as states

from robot_skills.reasoner  import Conjunction, Compound, Reasoner

query_meeting_point = Compound("waypoint", 
                        Compound("meeting_point", "Waypoint"), 
                        Compound("pose_2d", "X", "Y", "Phi"))
query_exploration_target_in_room = Conjunction( Compound("goal", Compound("clean_up", "Room")),
                                                Compound("exploration_target", "Room", "Target"),
                                                Compound("not", Compound("explored", "Target")),
                                                Compound("base_pose", "Target", Compound("pose_2d", "X", "Y", "Phi"))
                                               )
query_room = Conjunction(   Compound("goal", Compound("clean_up", "Room")), 
                            Compound("waypoint", "Room", Compound("pose_2d", "X", "Y", "Phi"))) 
query_exploration_target = Conjunction( Compound("current_exploration_target", "Target"),
                                        Compound("base_pose", "Target", Compound("pose_2d", "X", "Y", "Phi")))
query_lookat = Conjunction( Compound("current_exploration_target", "Target"),
                            Compound("point_of_interest", "Target", Compound("point_3d", "X", "Y", "Z")))
#Make sure the object we're dealing with isn't already disposed (i.e. handled for cleanup)
#After cleaning the object up/disposing it, 
#MARK_DISPOSED asserts disposed(current_objectID)
query_object = Conjunction(
                    Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")),
                    Compound("not", Compound("disposed", "ObjectID")))
query_grabpoint = Conjunction(  Compound("current_object", "ObjectID"),
                                Compound("position", "ObjectID", Compound("point", "X", "Y", "Z")))
query_current_object_class = Conjunction(
                        Compound("current_object",      "Obj_to_Dispose"), #Of the current object
                        Compound("instance_of",         "Obj_to_Dispose",   Compound("exact", "ObjectType")))
query_dropoff_loc = Conjunction(
                        Compound("current_object",      "Obj_to_Dispose"), #Of the current object
                        Compound("instance_of",         "Obj_to_Dispose",   Compound("exact", "ObjectType")), #Gets its type
                        Compound("storage_class",       "ObjectType",       "Disposal_type"), #Find AT what sort of thing it should be disposed, e.g. a trashbin
                        Compound("instance_of",         "Dispose_to_object","Disposal_type"), #Find objects of that are of type trashbin
                        Compound("point_of_interest",   "Dispose_to_object", Compound("point_3d", "X", "Y", "Z"))) #Get locations of those things
query_dropoff_loc_backup = Conjunction( Compound("instance_of", "Dispose_to_object",  "trashbin"), #Find objects of that are of type trashbin
                                            Compound("point_of_interest",  "Dispose_to_object",  Compound("point_3d", "X", "Y", "Z"))) #Get locations of those things
meeting_point = Conjunction(    Compound("waypoint", Compound("meeting_point", "Waypoint"), Compound("pose_2d", "X", "Y", "Phi")),
                                Compound("not", Compound("unreachable", Compound("meeting_point", "Waypoint"))))

if __name__ == "__main__":
    rospy.init_node('knowledge_debugger', log_level=rospy.DEBUG)

    r = Reasoner()

    queries = { "query_meeting_point":query_meeting_point, 
                "query_exploration_target_in_room":query_exploration_target_in_room, 
                "query_room":query_room, 
                "query_exploration_target":query_exploration_target, 
                "query_lookat":query_lookat, 
                "query_object":query_object, 
                "query_grabpoint":query_grabpoint, 
                "query_current_object_class":query_current_object_class, 
                "query_dropoff_loc":query_dropoff_loc, 
                "query_dropoff_loc_backup":query_dropoff_loc_backup}

    old_mask = {name:[] for name in queries.keys()}
    new_mask = {}
    while not rospy.is_shutdown():
        try:
            for name, term in queries.iteritems():
                answer = r.query(term)
                new_mask[name] = answer
                rospy.logdebug("querying {0}:{1} = {2}".format(name, term, answer))
                if new_mask[name] != old_mask[name]:
                    rospy.loginfo("query {0} changed. Was {1}, now is {2}".format(name, old_mask[name], new_mask[name]))
            old_mask = new_mask

            rospy.sleep(1)
        except Exception, e:
            rospy.logerr(e)

