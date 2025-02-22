from itertools import chain

# ROS
import rospy
import smach

# TU/e
from ed.entity import Entity

from robot_smach_states.navigation import NavigateToSymbolic, NavigateToObserve
from robot_smach_states.util.designators import Designator, VariableDesignator, check_resolve_type, check_type
from robot_smach_states.world_model import SegmentObjects
from robot_smach_states.designator_iterator import IterateDesignator
from robot_smach_states.rise import RiseForInspect

from robot_skills.classification_result import ClassificationResult


class InspectAreaDesignator(Designator):
    """
    Resolve to the inspect areas of a piece of furniture

    :param entityDes: entity designator
    :param knowledge: robocup knowledge object
    """
    def __init__(self, entityDes, knowledge, name=None):
        super(InspectAreaDesignator, self).__init__(resolve_type=[str], name=name)
        self.entityDes = entityDes
        self.knowledge = knowledge

    def _resolve(self):
        entity = self.entityDes.resolve()
        return self.knowledge.common.get_inspect_areas(entity.uuid)


class InspectAreas(smach.StateMachine):
    """
    Class to navigate to a(n) (furniture) object and segment the objects on top of it.
    """

    def __init__(self, robot, entityDes, objectIDsDes=None, roomDes=None, searchAreas=None, navigation_area=None,
                 knowledge=None, unknown_threshold=0.0, filter_threshold=0.0):
        """
        Constructor

        :param robot: robot object
        :param entityDes: EdEntityDesignator indicating the (furniture) object to inspect
        :param objectIDsDes: designator that is used to store the segmented objects
        :param roomDes: Room for robot to stay in while inspecting
        :param searchAreas: (designator to) array of strings defining where the objects are w.r.t. the entity,
            if none is provided we will use the knowledge to decide them
        :param navigation_area: string identifying the inspection area. If provided, NavigateToSymbolic is used.
            If left empty, NavigateToObserve is used.
        :param unknown_threshold: Entities whose classification score is lower than this float are not marked with a type
        :param filter_threshold: Entities whose classification score is lower than this float are ignored
            (i.e. are not added to the segmented_entity_ids_designator)
        """
        smach.StateMachine.__init__(self, outcomes=['done', 'failed'])

        check_type(entityDes, Entity)

        if objectIDsDes is None:
            objectIDsDes = VariableDesignator([], resolve_type=[ClassificationResult])

        if searchAreas:
            check_type(searchAreas, [str])
        else:
            if not knowledge:
                rospy.logerr("Please provide a list of searchAreas or a knowledge object!")
            # get search area's from knowledge
            searchAreas = InspectAreaDesignator(entityDes, knowledge)

        rospy.loginfo("searchAreas: {}".format(searchAreas.resolve()))
        searchArea = VariableDesignator(resolve_type=str)

        found_object_ids = VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            if navigation_area:
                smach.StateMachine.add('NAVIGATE_TO_INSPECT', NavigateToSymbolic(robot, {entityDes: navigation_area},
                                                                                 entityDes, room=roomDes),
                                       transitions={'unreachable': 'failed',
                                                    'goal_not_defined': 'failed',
                                                    'arrived': 'ITERATE_AREA'})
            else:
                smach.StateMachine.add('NAVIGATE_TO_INSPECT', NavigateToObserve(robot, entityDes,
                                                                                radius=1.0, room=roomDes),
                                       transitions={'unreachable': 'failed',
                                                    'goal_not_defined': 'failed',
                                                    'arrived': 'ITERATE_AREA'})

            smach.StateMachine.add('ITERATE_AREA',
                                   IterateDesignator(searchAreas, searchArea.writeable),
                                   transitions={'next': 'RISE',
                                                'stop_iteration': 'done'}
                                   )

            smach.StateMachine.add('RISE', RiseForInspect(robot, entityDes, searchArea),
                                   transitions={'succeeded': 'SEGMENT',
                                                'failed': 'SEGMENT'})

            smach.StateMachine.add('SEGMENT',
                                   SegmentObjects(robot, found_object_ids.writeable, entityDes, searchArea,
                                                  unknown_threshold=unknown_threshold,
                                                  filter_threshold=filter_threshold),
                                   transitions={'done': 'APPEND_OBJECT_IDS'})

            @smach.cb_interface(outcomes=["done"])
            def append_object_ids(_, stored_objects, segmented_objects):
                """
                @param stored_objects: is the new list of objects we have seen,
                @param segmented_objects: the list of the segmented objects which will be appended in the stored_objects
                """
                check_resolve_type(stored_objects, [ClassificationResult])
                check_resolve_type(segmented_objects, [ClassificationResult])
                objects_dict = {}
                for item in chain(stored_objects.resolve(), segmented_objects.resolve()):
                    objects_dict[item.uuid] = item
                all_objects = list(objects_dict.values())
                stored_objects.write(all_objects)
                return "done"

            smach.StateMachine.add("APPEND_OBJECT_IDS",
                                   smach.CBState(append_object_ids, cb_args=[objectIDsDes.writeable, found_object_ids]),
                                   transitions={"done": "ITERATE_AREA"})


if __name__ == "__main__":
    import sys
    from robot_skills.get_robot import get_robot
    from robot_smach_states.util.designators import EntityByIdDesignator

    if len(sys.argv) < 4:
        print(f"usage: python {sys.argv[0]} ROBOT ENTITY_ID SEARCH_VOLUMES")
        sys.exit()

    rospy.init_node('test_inspect_shelves')

    robot = get_robot(sys.argv[1])
    robot.reset()

    shelfDes = EntityByIdDesignator(robot, uuid=sys.argv[2])
    searchAreasDes = VariableDesignator(sys.argv[3:])

    sm = InspectAreas(robot, shelfDes, searchAreas=searchAreasDes, navigation_area="in_front_of")
    sm.execute()
