import smach

import robot_smach_states as states
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, ValueByKeyDesignator
from handle_detected_entities import HandleDetectedEntities
from robot_skills.classification_result import ClassificationResult


class CleanInspect(smach.StateMachine):
    def __init__(self, robot, location_des):
        """
        Visit all selected locations from the list, and handle the found objects

        :param location_des: is a designator resolving to a dictionary with fields ... TODO
        """

        smach.StateMachine.__init__(self, outcomes=['done'])

        segment_areas = ValueByKeyDesignator(location_des, "segment_areas", [str], name='segment_areas')
        segment_area = VariableDesignator(resolve_type=str, name='segment_area')

        navigation_area_des = ValueByKeyDesignator(location_des, key='navigation_area', resolve_type=str,
                                                   name='navigation_area')
        location_id_des = ValueByKeyDesignator(location_des, key='name', resolve_type=str, name='location_id')

        room_des = EdEntityDesignator(robot, id_designator=ValueByKeyDesignator(location_des, key="room",
                                                                                resolve_type=str))

        # Set up the designators for this machine
        e_classifications_des = VariableDesignator([], resolve_type=[ClassificationResult], name='e_classifications_des')
        e_des = EdEntityDesignator(robot, id_designator=location_id_des, name='e_des')

        with self:
            smach.StateMachine.add('ITERATE_NEXT_AREA',
                                   states.IterateDesignator(segment_areas, segment_area.writeable),
                                   transitions={"next": 'INSPECT',
                                                "stop_iteration": "done"})

            # Segment the area and handle segmented objects for the specified navigation area
            # for i, segment_area in enumerate(segment_areas):

            smach.StateMachine.add('INSPECT',
                                   states.NavigateToSymbolic(robot, {e_des: navigation_area_des}, e_des),
                                   transitions={'arrived': 'SEGMENT_SAY',
                                                'unreachable': "SAY_UNREACHABLE",
                                                'goal_not_defined': "SAY_UNREACHABLE"})

            smach.StateMachine.add("SEGMENT_SAY",
                                   states.Say(robot, "Looking {area} the {entity}", area=segment_area,
                                              entity=location_id_des, block=False),
                                   transitions={"spoken": "RISE"})

            smach.StateMachine.add('RISE', states.RiseForInspect(robot, e_des, segment_area),
                                   transitions={'succeeded': 'SEGMENT',
                                                'failed': 'SEGMENT'})

            smach.StateMachine.add('SEGMENT', states.SegmentObjects(robot, e_classifications_des.writeable, e_des,
                                                                    segment_area),
                                   transitions={'done': "HANDLE_DETECTED_ENTITIES"})

            smach.StateMachine.add("SAY_UNREACHABLE", states.Say(robot, "I failed to inspect the {furn}",
                                                                 furn=location_id_des, block=True),
                                   transitions={"spoken": "done"})

            smach.StateMachine.add("HANDLE_DETECTED_ENTITIES",
                                   HandleDetectedEntities(robot, e_classifications_des, location_id_des, segment_area,
                                                          room_des),
                                   transitions={"done": "ITERATE_NEXT_AREA"})
