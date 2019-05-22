import rospy
import robot_smach_states as states
import robot_smach_states.util.designators as ds
import smach
from hmi import HMIResult
from robocup_knowledge import load_knowledge
from robot_skills.util.entity import Entity
from robot_skills.classification_result import ClassificationResult

challenge_knowledge = load_knowledge('challenge_receptionist')


class GuestDescriptionStrDesignator(ds.Designator):
    def __init__(self, guest_name_des, drinkname, name=None):
        super(GuestDescriptionStrDesignator, self).__init__(resolve_type=str, name=name)

        ds.check_type(guest_name_des, str)
        ds.check_type(drinkname, str)

        self.guest_name_des = guest_name_des
        self.drinkname = drinkname

    def _resolve(self):
        name = self.guest_name_des.resolve()
        drinkname = self.drinkname.resolve()
        return "This is {name} whose favourite drink is {drink}".format(name=name, drink=drinkname)


class FieldOfHMIResult(ds.Designator):
    """
    Extract a field of a QueryResult
    """
    def __init__(self, query_result_des, semantics_field, name=None):
        """
        Construct a designator that picks a field out of the semantics dict of a QueryResult
        (such as resulting from a HearOptionsExtra-state)
        :param query_result_des: A designator resolving to a QueryResult
        :param semantics_field: str (or string designator) used in query_result.semantics[semantics_field]
        :param name: Name for this designator for debugging purposes
        """
        super(FieldOfHMIResult, self).__init__(resolve_type=str, name=name)

        ds.check_type(query_result_des, HMIResult)
        ds.check_type(semantics_field, str)

        self.query_result_des = query_result_des
        self.semantics_field = semantics_field

    def _resolve(self):
        try:
            field = self.semantics_field.resolve() if hasattr(self.semantics_field, 'resolve') else self.semantics_field
            return self.query_result_des.resolve().semantics[field]
        except Exception as e:
            rospy.logerr(e)
            return None


class LearnGuest(smach.StateMachine):
    def __init__(self, robot, door_waypoint, guest_ent_des, guest_name_des, guest_drink_des):
        """

        :param robot: Robot that should execute this state
        :param door_waypoint: Entity-designator resolving to a waypoint Where are guests expected to come in
        :param guest_ent_des: Entity of the guest
        :param guest_name_des: designator that the name (str) of the guest is written to
        :param guest_drink_des: designator that the drink type (str) of the drink the guest wants
        """
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'abort'])

        self.drink_spec_des = ds.Designator(challenge_knowledge.common.drink_spec, name='drink_spec')

        with self:
            smach.StateMachine.add('GOTO_DOOR',
                                   states.NavigateToWaypoint(robot,
                                                             door_waypoint,
                                                             challenge_knowledge.waypoint_door['radius']),
                                   transitions={'arrived': 'SAY_PLEASE_COME_IN',
                                                'unreachable': 'SAY_PLEASE_COME_IN',
                                                'goal_not_defined': 'abort'})

            smach.StateMachine.add('SAY_PLEASE_COME_IN',
                                   states.Say(robot, ["Please come in, I'm waiting"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'WAIT_FOR_GUEST'})

            smach.StateMachine.add("WAIT_FOR_GUEST",
                                   states.WaitForPersonInFront(robot, attempts=30, sleep_interval=1),
                                   transitions={'success': 'SAY_HELLO',
                                                'failed': 'SAY_PLEASE_COME_IN'})

            smach.StateMachine.add('SAY_HELLO',
                                   states.Say(robot, ["Hi there, I'll learn your face now"],
                                              block=True,
                                              look_at_standing_person=True),
                                   transitions={'spoken': 'ASK_GUEST_NAME'})

            smach.StateMachine.add('ASK_GUEST_NAME',
                                   states.AskPersonName(robot, guest_name_des.writeable, challenge_knowledge.common.names),
                                   transitions={'succeeded': 'LEARN_PERSON',
                                                'failed': 'SAY_HELLO',
                                                'timeout': 'SAY_HELLO'})

            smach.StateMachine.add('LEARN_PERSON',
                                   states.LearnPerson(robot, name_designator=guest_name_des),
                                   transitions={'succeeded': 'SAY_GUEST_LEARNED',
                                                'failed': 'SAY_FAILED_LEARNING'})

            smach.StateMachine.add('SAY_FAILED_LEARNING',
                                   states.Say(robot, ["Oops, I'm confused, let's try again"],
                                              block=False),
                                   transitions={'spoken': 'LEARN_PERSON'})

            smach.StateMachine.add('SAY_GUEST_LEARNED',
                                   states.Say(robot, ["Okidoki, now I know what you look like"], block=True),
                                   transitions={'spoken': 'SAY_DRINK_QUESTION'})

            smach.StateMachine.add('SAY_DRINK_QUESTION',
                                   states.Say(robot, ["What's your favorite drink?"], block=True),
                                   transitions={'spoken': 'HEAR_DRINK_ANSWER'})

            smach.StateMachine.add('HEAR_DRINK_ANSWER',
                                   states.HearOptionsExtra(robot,
                                                           self.drink_spec_des,
                                                           guest_drink_des.writeable),
                                   transitions={'heard': 'succeeded',
                                                'no_result': 'SAY_DRINK_QUESTION'})


class IntroduceGuestToOperator(smach.StateMachine):
    def __init__(self, robot, operator_des, guest_ent_des, guest_name_des, guest_drinkname_des):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'abort'])

        ds.check_type(guest_name_des, str)
        ds.check_type(guest_drinkname_des, str)
        ds.check_type(guest_ent_des, Entity)

        with self:
            smach.StateMachine.add('FIND_OPERATOR',
                                   states.FindPersonInRoom(robot,
                                                           # We're looking for any person and assume that thats the operator
                                                           discard_other_labels=False,
                                                           area=challenge_knowledge.waypoint_livingroom['id'],
                                                           name=challenge_knowledge.operator_name,
                                                           found_entity_designator=operator_des.writeable),
                                   transitions={'found': 'GOTO_OPERATOR',
                                                'not_found': 'GOTO_OPERATOR'})

            smach.StateMachine.add('GOTO_OPERATOR',
                                   states.NavigateToObserve(robot,
                                                            operator_des),
                                   transitions={'arrived': 'SAY_LOOKING_FOR_GUEST',
                                                'unreachable': 'SAY_LOOKING_FOR_GUEST',
                                                'goal_not_defined': 'abort'})

            smach.StateMachine.add('SAY_LOOKING_FOR_GUEST',
                                   states.Say(robot,
                                              ["Now I should be looking at the guest and pointing at him or her"],
                                              block=True),
                                   transitions={'spoken': 'INTRODUCE_GUEST'})

            smach.StateMachine.add('FIND_GUEST',
                                   states.FindPerson(robot=robot,
                                                     person_label=guest_name_des,
                                                     found_entity_designator=guest_ent_des.writeable),
                                   transitions={"found": "POINT_AT_GUEST",
                                                "failed": "abort"})

            smach.StateMachine.add('POINT_AT_GUEST',
                                   states.PointAt(robot=robot,
                                                  arm_designator=ds.UnoccupiedArmDesignator(robot,{'required_goals':['point_at']}),
                                                  point_at_designator=guest_ent_des,
                                                  look_at_designator=operator_des),
                                   transitions={"succeeded": "INTRODUCE_GUEST",
                                                "failed": "abort"})

            smach.StateMachine.add('INTRODUCE_GUEST',
                                   states.Say(robot, GuestDescriptionStrDesignator(guest_name_des, guest_drinkname_des),
                                              block=True,
                                              look_at_standing_person=False),
                                   transitions={'spoken': 'RESET_ARM'})

            smach.StateMachine.add('RESET_ARM',
                                   states.ResetArmsTorsoHead(robot),
                                   transitions={'done': 'succeeded'})


class CheckVolumeEmpty(smach.StateMachine):
    def __init__(self, robot, entity_des, volume):
        smach.StateMachine.__init__(self, outcomes=['empty', 'occupied', 'failed'])

        seen_entities_des = ds.VariableDesignator([], resolve_type=[ClassificationResult])

        with self:
            smach.StateMachine.add('INSPECT',
                                   states.Inspect(robot, entity_des, searchArea=volume, objectIDsDes=seen_entities_des),
                                   transitions = {"done": "CHECK",
                                                  "failed": "failed"})

            @smach.cb_interface(outcomes=['empty', 'occupied'])
            def check_occupied(userdata):
                seen_entities = seen_entities_des.resolve()
                if seen_entities:
                    return 'occupied'
                else:
                    return 'empty'
            smach.StateMachine.add('CHECK', smach.CBState(check_occupied),
                                   transitions={'empty': 'empty',
                                                'occupied': 'occupied'})


class FindEmptySeat(smach.StateMachine):
    def __init__(self, robot, seats_to_inspect):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'abort'])

        # We have to find an *empty* seat in the given room
        # I'd say: iterate over all seat-type objects and check that their 'on-top-of' volume is empty
        # That can be done with an Inspect and then query for any Entities inside that volume.
        # If there are none, then the seat is empty
        with self:
            for seat_id in seats_to_inspect:
                seat_ent_des = ds.EntityByIdDesignator(robot, seat_id)

                smach.StateMachine.add('CHECK_SEAT_EMPTY',
                                       CheckVolumeEmpty(robot, seat_ent_des, 'on_top_of'),
                                       transitions={'occupied': 'INTRODUCE_GUEST',  # TODO: next state...
                                                    'empty': 'SAY_SEAT_EMPTY',
                                                    'failed': 'abort'})

            smach.StateMachine.add('SAY_SEAT_EMPTY',
                                   states.Say(robot, ["Please sit here"], block=True),
                                   transitions={'spoken': 'succeeded'})


class ChallengeReceptionist(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'abort'])

        self.door_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_door['id'])
        self.livingroom_waypoint = ds.EntityByIdDesignator(robot, id=challenge_knowledge.waypoint_livingroom['id'])

        self.operator_designator = ds.VariableDesignator(resolve_type=Entity)

        self.guest1_entity_des = ds.VariableDesignator(resolve_type=Entity, name='guest1_entity')
        self.guest1_name_des = ds.VariableDesignator('guest 1', name='guest1_name')
        self.guest1_drink_des = ds.VariableDesignator(resolve_type=HMIResult, name='guest1_drink')
        self.guest1_drinkname_des = FieldOfHMIResult(self.guest1_drink_des, semantics_field='drink',
                                                     name='guest1_drinkname')

        with self:
            smach.StateMachine.add('INITIALIZE',
                                   states.Initialize(robot),
                                   transitions={'initialized': 'SET_INITIAL_POSE',
                                                'abort': 'abort'})

            smach.StateMachine.add('SET_INITIAL_POSE',
                                   states.SetInitialPose(robot, challenge_knowledge.starting_point),
                                   transitions={'done': 'LEARN_GUEST_1',
                                                "preempted": 'abort',
                                                'error': 'LEARN_GUEST_1'})

            smach.StateMachine.add('LEARN_GUEST_1',
                                   LearnGuest(robot, self.door_waypoint, self.guest1_entity_des, self.guest1_name_des,
                                              self.guest1_drink_des),
                                   transitions={'succeeded': 'GOTO_LIVINGROOM_1',
                                                'abort': 'abort'})

            smach.StateMachine.add('GOTO_LIVINGROOM_1',
                                   states.NavigateToWaypoint(robot,
                                                             self.livingroom_waypoint,
                                                             challenge_knowledge.waypoint_livingroom['radius']),
                                   transitions={'arrived': 'INTRODUCE_GUEST',
                                                'unreachable': 'INTRODUCE_GUEST',
                                                'goal_not_defined': 'abort'})

            smach.StateMachine.add('INTRODUCE_GUEST',
                                   IntroduceGuestToOperator(robot, self.operator_designator, self.guest1_entity_des,
                                                            self.guest1_name_des, self.guest1_drinkname_des),
                                   transitions={'succeeded': 'succeeded',
                                                'abort': 'abort'})

            # - [x] Wait at the door, say you're waiting
            # - [x] Wait until person can come in
            # - [x] Ask their name
            # - [x] Ask their favourite drink
            # - [x] Ask for favourite drink <drink1>
            # - [x] GOTO living room
            # - [x] Locate John (not sure how that should work, maybe just FindPersonInRoom)
            # - [x] GOTO John
            # - [x] Locate guest1:
            # - [x]   rotate head until <guest1> is detected
            # - [x] Point at guest1
            # - [x] Say: This is <guest1> and (s)he likes to drink <drink1>
            # - [ ] Say to guest what John's favourite drink is
            # - [ ] Iterate to guest 2
            # - Change ED API to accept list of entity IDs
