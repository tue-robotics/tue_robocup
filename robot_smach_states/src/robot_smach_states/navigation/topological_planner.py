import networkx as nx
import rospy

from robot_skills.get_robot import get_robot_from_argv


def _get_distance(e1, e2):
    """
    Returns the distance between the center points of the 'in front of' volumes of two entities

    :param e1:
    :param e2:
    :return:
    """
    p1 = e1.pose.frame * e1.volumes["in_front_of"].center_point
    p2 = e2.pose.frame * e2.volumes["in_front_of"].center_point
    return (p1 - p2).Norm()


def setup_graph(entities):
    graph = nx.Graph()

    for entity in entities:
        if "in_front_of" not in entity.volumes:
            continue

        rospy.loginfo("Adding {}".format(entity.id))
        graph.add_node(entity)

        # ToDo: add more criteria
        neighbour_options = []
        for entity2 in [e for e in entities if e.id != entity.id and "in_front_of" in e.volumes]:
            neighbour_options.append((entity2, _get_distance(entity, entity2)))

        neighbours = sorted(neighbour_options, key=lambda tup: tup[1])[:3]
        for n in neighbours:
            rospy.loginfo("\tAdding edge to {}".format(n[0].id))
            graph.add_edge(entity, n[0], distance=n[1])

    return graph


def plan(robot, goal_object):
    entities = robot.ed.get_entities()
    furniture_objects = [e for e in entities if e.is_a("furniture")]
    graph = setup_graph(furniture_objects)
    closest_furniture = sorted(furniture_objects,
                               key=lambda fo: fo.distance_to_2d(robot.base.get_location().frame.p))[0]
    goal_entity = [ge for ge in entities if ge.id == goal_object][0]
    path = nx.shortest_path(graph, closest_furniture, goal_entity)
    for p in path:
        rospy.loginfo(p.id)


if __name__ == "__main__":
    rospy.init_node("topological_planner_test_node")
    _robot = get_robot_from_argv(1)
    plan(_robot, "dinner_table")
