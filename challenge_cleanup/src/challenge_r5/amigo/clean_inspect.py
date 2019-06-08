import smach

import robot_smach_states
from robot_smach_states.util.designators import EdEntityDesignator, VariableDesignator, EntityByIdDesignator

from handle_detected_entities import HandleDetectedEntities
from robot_skills.classification_result import ClassificationResult


class CleanInspect(smach.StateMachine):
    def __init__(self, robot, location_id, room_id, navigate_area, segment_areas, known_types):

        smach.StateMachine.__init__(self, outcomes=['done'])

        # Set up the designators for this machine
        e_classifications_des = VariableDesignator([], resolve_type=[ClassificationResult], name='e_classifications_des')
        e_des = EdEntityDesignator(robot, id=location_id, name='e_des')
        room_des = EntityByIdDesignator(robot, id=room_id, name='room_des')

        with self:

            # Loop over the areas that we have to segment and handle segmented objects for the specified navigation area
            for i, segment_area in enumerate(segment_areas):

                smach.StateMachine.add("RESET_ED_%d" % i, robot_smach_states.ResetED(robot),
                                       transitions={'done': 'NAVIGATE_%d' % i})

                smach.StateMachine.add("NAVIGATE_%d" % i,
                                       robot_smach_states.NavigateToSymbolic(robot, {e_des: navigate_area, room_des: "in"},
                                                                             e_des),
                                       transitions={'arrived': 'SEGMENT_SAY_%d' % i,
                                                    'unreachable': "SAY_UNREACHABLE_%d" % i,
                                                    'goal_not_defined': "SAY_UNREACHABLE_%d" % i})

                smach.StateMachine.add("SEGMENT_SAY_%d" % i,
                                       robot_smach_states.Say(robot, ["Looking %s the %s"
                                                                      % (segment_area, location_id)], block=False),
                                       transitions={"spoken": "SEGMENT_%d" % i})

                smach.StateMachine.add('SEGMENT_%d' % i, robot_smach_states.SegmentObjects(robot,
                                                                                           e_classifications_des.writeable,
                                                                                           e_des,
                                                                                           segment_area),
                                       transitions={'done': "HANDLE_DETECTED_ENTITIES_%d" % i})

                # Determine the next state, either it is the next iter or done
                next_state = "RESET_ED_%d" % (i + 1) if i + 1 < len(segment_areas) else "done"

                smach.StateMachine.add("SAY_UNREACHABLE_%d" % i,
                                       robot_smach_states.Say(robot, ["I failed to inspect the %s" % location_id], block=True),
                                       transitions={"spoken":next_state})

                smach.StateMachine.add("HANDLE_DETECTED_ENTITIES_%d" % i,
                                       HandleDetectedEntities(robot, e_classifications_des, known_types, location_id, segment_area),
                                       transitions={"done": next_state})
