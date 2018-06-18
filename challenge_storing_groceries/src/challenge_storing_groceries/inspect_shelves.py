# System
import os

# ROS
import rospy
import smach

# TU/e
from robot_skills.util.kdl_conversions import VectorStamped

# Challenge storing groceries
import config


class InspectShelves(smach.State):
    """ Inspect all object shelves """

    def __init__(self, robot, cabinet):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'nothing_found'])
        self.robot = robot
        self.cabinet = cabinet
        self.object_shelves = config.OBJECT_SHELVES

    def execute(self, userdata=None):

        # Get cabinet entity
        # Sleep for a while to make sure that the robot is actually in ED
        rospy.sleep(rospy.Duration(0.25))
        cabinet_entity = self.robot.ed.get_entity(id=self.cabinet.id_, parse=True)

        # Get the pose of all shelves
        shelves = []
        for k, v in cabinet_entity.volumes.iteritems():
            if k in config.OBJECT_SHELVES:
                rospy.loginfo("Adding {} to shelves".format(k))
                vector = 0.5 * (v.min_corner + v.max_corner)
                shelves.append({'ps': VectorStamped(frame_id=cabinet_entity.id, vector=vector), 'name': k})

        # Sort the list in ascending order
        shelves = sorted(shelves, key=lambda x: x['ps'].vector.z())
        for shelf in shelves:

            ps = shelf['ps']

            # Send goals to torso and head
            height = min(0.4, max(0.1, ps.vector.z() - 0.55))
            self.robot.torso._send_goal([height])
            self.robot.head.look_at_point(ps)

            # Wait for the motions to finish
            self.robot.torso.wait_for_motion_done(timeout=5.0)
            # ToDo: wait for head?

            # Sleep for 1 second
            do_wait = os.environ.get('ROBOT_REAL')
            if do_wait == 'true':
                rospy.sleep(3.0)  # ToDo: remove???
                rospy.logwarn("Do we have to wait this long???")

            if config.DEBUG:
                rospy.loginfo('Stopping: debug mode. Press c to continue to the next point')
                import ipdb; ipdb.set_trace()
                continue

            # Enable kinect segmentation plugin (only one image frame)
            segmented_entities = self.robot.ed.update_kinect("{} {}".format(shelf['name'], cabinet_entity.id))
            # print "Segmented new entities: {}".format(segmented_entities.new_ids)

            for id_ in segmented_entities.new_ids:
                # In simulation, the entity type is not yet updated...
                entity = self.robot.ed.get_entity(id=id_, parse=False)
                config.SEGMENTED_ENTITIES.append((entity, id_))
            # print "Config.SEGMENTED_ENTITIES: {}".format(config.SEGMENTED_ENTITIES)

            # ToDo: classification threshold
            entity_types_and_probs = self.robot.ed.classify(ids=segmented_entities.new_ids, types=config.OBJECT_TYPES)
            # print "Types and probs: {}".format(entity_types_and_probs)

            # Recite entities
            types = [etp.type for etp in entity_types_and_probs]
            if len(types) > 1:
                types_desc = ", ".join([t for t in types[:-1]]) + " and " + types[-1]
                self.robot.speech.speak("{0}".format(types_desc, block=False))
            elif len(types) == 1:
                types_desc = types[0]
                self.robot.speech.speak("{0}".format(types_desc, block=False))

            # Lock entities
            self.robot.ed.lock_entities(lock_ids=[e.id for e in entity_types_and_probs], unlock_ids=[])

            for e in entity_types_and_probs:
                # In simulation, the entity type is not yet updated...
                entity = self.robot.ed.get_entity(id=e.id, parse=False)
                config.DETECTED_OBJECTS_WITH_PROBS.append((entity, e.probability))

            config.DETECTED_OBJECTS_WITH_PROBS = sorted(config.DETECTED_OBJECTS_WITH_PROBS, key=lambda o: o[1],
                                                        reverse=True)

        # Reset the head goal
        self.robot.head.cancel_goal()

        if not config.DETECTED_OBJECTS_WITH_PROBS:
            return "nothing_found"

        # Sort based on probability
        # DETECTED_OBJECTS_WITH_PROBS = sorted(DETECTED_OBJECTS_WITH_PROBS, key=lambda o: o[1], reverse=True)

        return 'succeeded'
