# ROS
import rospy
import smach

from config import *


class InspectShelves(smach.State):
    """ Inspect all object shelves """

    def __init__(self, robot, object_shelves):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'nothing_found'])
        self.robot = robot
        self.object_shelves = object_shelves

    def execute(self, userdata):

        global SEGMENTED_ENTITIES
        global DETECTED_OBJECTS_WITH_PROBS

        ''' Get cabinet entity '''
        rospy.sleep(rospy.Duration(0.25))  # Sleep for a while to make
        # sure that the robot is actually in ED
        cabinet_entity = self.robot.ed.get_entity(id=CABINET, parse=True)

        ''' Get the pose of all shelves '''
        shelves = []
        for area in cabinet_entity.data['areas']:
            ''' See if the area is in the list of inspection areas '''
            if area['name'] in OBJECT_SHELVES:
                ''' Check if we have a shape '''
                if 'shape' not in area:
                    rospy.logwarn("No shape in area {0}".format(area['name']))
                    continue
                ''' Check if length of shape equals one '''
                if not len(area['shape']) == 1:
                    rospy.logwarn("Shape of area {0} contains multiple entries, don't know what to do".format(area['name']))
                    continue
                ''' Check if the first entry is a box '''
                if not 'box' in area['shape'][0]:
                    rospy.logwarn("No box in {0}".format(area['name']))
                    continue
                box = area['shape'][0]['box']
                if 'min' not in box or 'max' not in box:
                    rospy.logwarn("Box in {0} either does not contain min or max".format(area['name']))
                    continue

                x = 0.5 * (box['min']['x'] + box['max']['x'])
                y = 0.5 * (box['min']['y'] + box['max']['y'])
                z = 0.5 * (box['min']['z'] + box['max']['z'])
                shelves.append({'ps': geom.PointStamped(x, y, z, cabinet_entity.id), 'name': area['name']})
            else:
                rospy.loginfo("{0} not in object shelves".format(area['name']))

        # rospy.loginfo("Inspection points: {0}".format(shelves))
        # ''' Loop over shelves '''
        # for shelf in self.object_shelves:
        for shelf in shelves:

            ps = shelf['ps']
            cp = ps.point

            # ''' Get entities '''
            # shelf_entity = self.robot.ed.get_entity(id=shelf, parse=False)

            # if shelf_entity:

            # ''' Extract center point '''
            # cp = shelf_entity.pose.position

            ''' Look at target '''
            self.robot.head.look_at_point(ps)

            ''' Move spindle
                Implemented only for AMIGO (hence the hardcoding)
                Assume table height of 0.8 corresponds with spindle reset = 0.35 '''
            # def _send_goal(self, torso_pos, timeout=0.0, tolerance = []):
            # ToDo: do head and torso simultaneously
            height = min(0.4, max(0.1, cp.z-0.55))
            self.robot.torso._send_goal([height], timeout=5.0)

            ''' Sleep for 1 second '''
            import os; do_wait = os.environ.get('ROBOT_REAL')
            if do_wait == 'true':
                rospy.sleep(3.0) # ToDo: remove???
                rospy.logwarn("Do we have to wait this long???")

            if DEBUG:
                rospy.loginfo('Stopping: debug mode. Press c to continue to the next point')
                import ipdb;ipdb.set_trace()
                continue

            ''' Enable kinect segmentation plugin (only one image frame) '''
            # entity_ids = self.robot.ed.segment_kinect(max_sensor_range=2)  ## Old
            # segmented_entities = self.robot.ed.update_kinect("{} {}".format("on_top_of", shelf))
            segmented_entities = self.robot.ed.update_kinect("{} {}".format(shelf['name'], cabinet_entity.id))

            for id_ in segmented_entities.new_ids:
                entity = self.robot.ed.get_entity(id=id_, parse=False)  # In simulation, the entity type is not yet updated...
                SEGMENTED_ENTITIES.append((entity, id_))

            entity_types_and_probs = self.robot.ed.classify(ids=segmented_entities.new_ids, types=OBJECT_TYPES)

            # Recite entities
            for etp in entity_types_and_probs:
                self.robot.speech.speak("I have seen {0}".format(etp.type), block=False)

            # Lock entities
            self.robot.ed.lock_entities(lock_ids=[e.id for e in entity_types_and_probs], unlock_ids=[])

            # DETECTED_OBJECTS_WITH_PROBS = [(e.id, e.type) for e in entity_types_and_probs]
            # DETECTED_OBJECTS_WITH_PROBS = [(e.id, e.type) for e in sorted(entity_types_and_probs, key=lambda o: o[1], reverse=True)]
            for e in entity_types_and_probs:
                entity = self.robot.ed.get_entity(id=e.id, parse=False)  # In simulation, the entity type is not yet updated...
                DETECTED_OBJECTS_WITH_PROBS.append((entity, e.probability))

            # print "Detected obs with props 1: {0}".format(DETECTED_OBJECTS_WITH_PROBS)
            DETECTED_OBJECTS_WITH_PROBS = sorted(DETECTED_OBJECTS_WITH_PROBS, key=lambda  o: o[1], reverse=True)
            # print "Detected obs with props 2: {0}".format(DETECTED_OBJECTS_WITH_PROBS)

        if not DETECTED_OBJECTS_WITH_PROBS:
            return "nothing_found"

        # Sort based on probability
        # DETECTED_OBJECTS_WITH_PROBS = sorted(DETECTED_OBJECTS_WITH_PROBS, key=lambda o: o[1], reverse=True)

        return 'succeeded'

