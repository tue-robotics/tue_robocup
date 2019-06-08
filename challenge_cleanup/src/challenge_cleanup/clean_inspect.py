import smach
import rospy

import robot_smach_states
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, EntityByIdDesignator, \
    Designator, check_type
from handle_detected_entities import HandleDetectedEntities
from robot_skills.classification_result import ClassificationResult

class ValueByKeyDesignator(Designator):
    def __init__(self, container, key, resolve_type, name=None):
        """
        Get a value from a dictionary by it's key
        :param container: any object with a __getitem__ method or a designator that resolves to it
        :param name: Name of the designator for introspection purposes
        """
        super(ValueByKeyDesignator, self).__init__(resolve_type=resolve_type, name=name)
        # TODO: Add type checks to make sure that we can do container[key]
        # OR container.resolve[key]
        self._container = container
        self._key = key

    def _resolve(self):
        container = self._container.resolve()
        return container[self._key]


class CleanInspect(smach.StateMachine):
#    def __init__(self, robot, location_id, room_id, navigate_area, segment_areas):
    def __init__(self, robot, location_des):
        """
        Visit all selected locations from the list, and handle the found objects
        :param location_des is a designator resolving to a dictionary with fields ... TODO
        """

        smach.StateMachine.__init__(self, outcomes=['done'])
#new
        # location = location_des.resolve()
        # segment_areas = location['segment_areas']
        # navigate_area = location['navigation_area']
        # location_id = location['name']
        # room_id = location['room']

        segment_areas = ValueByKeyDesignator(location_des, "segment_areas", [str], name='segment_areas')
        segment_area = VariableDesignator(resolve_type=str, name='segment_area')

        navigate_area = ValueByKeyDesignator(location_des, 'navigation_area', str, name='navigate_area')
        location_id = ValueByKeyDesignator(location_des, 'name', str, name='location_id')
        room_id = ValueByKeyDesignator(location_des, 'room', str, name='room_id')


#/new
        # Set up the designators for this machine
        e_classifications_des = VariableDesignator([], resolve_type=[ClassificationResult], name='e_classifications_des')
        e_des = EdEntityDesignator(robot, id=location_id, name='e_des')
        room_des = EntityByIdDesignator(robot, id=room_id, name='room_des')

        with self:
            smach.StateMachine.add('ITERATE_NEXT_AREA',
                                    robot_smach_states.IterateDesignator(segment_areas, segment_area.writeable),
                                    transitions={"next": 'INSPECT',
                                                 "stop_iteration": "done"})


            # Segment the area and handle segmented objects for the specified navigation area
            # for i, segment_area in enumerate(segment_areas):

            smach.StateMachine.add('INSPECT',
                                    robot_smach_states.NavigateToSymbolic(robot, {e_des: navigate_area, room_des: "in"},
                                                                             e_des),
                                    transitions={'arrived': 'SEGMENT_SAY',
                                                 'unreachable': "SAY_UNREACHABLE",
                                                 'goal_not_defined': "SAY_UNREACHABLE"})

            smach.StateMachine.add("SEGMENT_SAY",
                                    robot_smach_states.Say(robot, ["Looking %s the %s"
                                                                      % (segment_area, location_id)], block=False),
                                    transitions={"spoken": "SEGMENT"})

            smach.StateMachine.add('SEGMENT', robot_smach_states.SegmentObjects(robot,
                                                                                e_classifications_des.writeable,
                                                                                e_des,
                                                                                segment_area),
                                    transitions={'done': "HANDLE_DETECTED_ENTITIES"})

            # Determine the next state, either it is the next iter or done

            smach.StateMachine.add("SAY_UNREACHABLE",
                                    robot_smach_states.Say(robot, ["I failed to inspect the %s" % location_id], block=True),
                                    transitions={"spoken": "done"})

            smach.StateMachine.add("HANDLE_DETECTED_ENTITIES",
                                    HandleDetectedEntities(robot, e_classifications_des, location_id, segment_area),
                                    transitions={"done": "done"})
