import smach
import robot_smach_states as states


class CustomInspect(smach.StateMachine):
    def __init__(self, robot, entity_des, waypoint_des):
        smach.StateMachine.__init__(self, outcomes=["done"])

        self.robot = robot

        with self:
            smach.StateMachine.add("NAVIGATE_TO_INSPECT_POINT",
                                   states.NavigateToWaypoint(self.robot, waypoint_des, look_at_designator=entity_des),
                                   transitions={"arrived": "INSPECTT",
                                                "unreachable": "INSPECTT",
                                                "goal_not_defined": "INSPECTT"})

            smach.StateMachine.add("INSPECTT",
                                   states.UpdateEntityPose(self.robot, entity_des),
                                   transitions={"done": "done"})
