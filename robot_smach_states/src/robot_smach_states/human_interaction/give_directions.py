# ROS
import PyKDL as kdl
import rospy
import smach

# TU/e Robotics
from robot_skills.robot import Robot
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import point_msg_to_kdl_vector, VectorStamped

# Robot Smach States
from robot_smach_states.navigation.navigate_to_symbolic import NavigateToSymbolic


class GiveDirections(smach.State):
    """
    Robot tells the operator how to get to a certain entity
    """
    def __init__(self, robot, entity_designator, radius=1.5):
        # type: (Robot, any) -> any
        """
        Init

        :param robot: (Robot) API object
        :param entity_designator: (EdEntityDesignator) resolving to the entity the operator wants to go to
        :param radius: passing closer to an entity than this radius, the entity will be mentioned
        """

        super(GiveDirections, self).__init__(outcomes=["succeeded", "failed"])

        self._robot = robot
        self._entity_designator = entity_designator
        self._radius = radius

    def execute(self, ud):

        # Get the constraints for the global planner
        nav_contraints = NavigateToSymbolic.generate_constraint(
            robot=self._robot,
            entity_designator_area_name_map={self._entity_designator: "near"},
            entity_lookat_designator=self._entity_designator
        )
        if nav_contraints is None:
            rospy.logerr("Cannot give directions if I don't know where to go")
            self._robot.speech.speak("I'm sorry but I don't know where you want to go", mood="sad")
            return "failed"

        # Resolve the entity: we'll need it later
        target_entity = self._entity_designator.resolve()  # type: Entity

        # Call the global planner for the shortest path to this entity
        path = self._robot.base.global_planner.getPlan(position_constraint=nav_contraints[0])
        if path is None:

            rospy.logerr("No path to {}".format(target_entity.id))
            self._robot.speech.speak("I'm sorry but I don't know how to get to the {}".format(target_entity.id))
            return "failed"

        # Convert the path to a list of kdl Vectors
        assert(all([p.header.frame_id.endswith("map") for p in path])), "Not all path poses are defined w.r.t. 'map'"
        kdl_path = [point_msg_to_kdl_vector(p.pose.position) for p in path]

        # Get all entities
        entities = self._robot.ed.get_entities()
        room_entities = [e for e in entities if e.type == "room"]

        # Figure out which entities are passed along the way
        # N.B.: this is a first naive implementation: optimization might be desired
        t_start = rospy.Time.now()
        passed_room_ids = []

        for position in kdl_path:
            for e in room_entities:  # type: Entity

                # If the ID is already present: continue
                if e.id in passed_room_ids:
                    continue

                # Check if it meets the requirement
                if e.in_volume(VectorStamped(vector=position), "in"):
                    passed_room_ids.append(e.id)

        passed_ids = []
        furniture_entities = [e for e in entities if e.is_a("furniture")]
        for position in kdl_path:
            to_add = []
            reached_target = False
            for e in furniture_entities:  # type: Entity
                rospy.logdebug("\tEntity: {}".format(e.id))

                # If the id is already present: continue
                if e.id in passed_ids:
                    rospy.logdebug("Skipping {}, already in passed ids".format(e.id))
                    continue

                # Check the distance
                distance = e.distance_to_2d(position)

                if distance < self._radius:
                    rospy.logdebug("Appending {}: {}".format(e.id, distance))
                    to_add.append((e.id, distance))

            # Sort the list
            to_add.sort(key=lambda id_distance_tuple: id_distance_tuple[1])

            # Add all items to the list
            for item in to_add:
                if item[0] == target_entity.id:
                    reached_target = True
                    break

                passed_ids.append(item[0])

            if reached_target:
                break

        rospy.loginfo("Directions computation took {} seconds".format((rospy.Time.now() - t_start).to_sec()))

        self._robot.speech.speak("You have to through the {} to get to the {}".format(
            ", the ".join(passed_room_ids), target_entity.id
        ))
        self._robot.speech.speak("There you have to walk by the {} to get to the {}".format(
            ", the ".join(passed_ids), target_entity.id
        ))

        # ToDo's:
        # * Make acknowledged of rooms
        # * Identify left or right

        return "succeeded"


if __name__ == "__main__":

    import sys
    import robot_smach_states.util.designators as ds

    rospy.init_node('simple_navigate')

    robot_name = sys.argv[1]
    if robot_name == 'amigo':
        from robot_skills.amigo import Amigo as Robot
    elif robot_name == 'sergio':
        from robot_skills.sergio import Sergio as Robot
    elif robot_name == 'hero':
        from robot_skills.hero import Hero as Robot
    else:
        print "unknown robot"
        sys.exit()

    robot = Robot()

    if len(sys.argv) > 2:
        furniture_id = sys.argv[2]
    else:
        furniture_id = "bed"

    rospy.loginfo("Waiting for tf cache to be filled")
    rospy.sleep(2)  # wait for tf cache to be filled

    rospy.loginfo("Starting giving directions to {}".format(furniture_id))

    state = GiveDirections(robot=robot,
                           entity_designator=ds.EntityByIdDesignator(robot=robot, id=furniture_id),
                           )
    state.execute(None)
