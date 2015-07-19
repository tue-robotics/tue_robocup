#!/usr/bin/python

"""This challenge is defined in https://raw.githubusercontent.com/RoboCupAtHome/RuleBook/master/WakeMeUp.tex

In short, the robot must go the the homeowners bedroom, wake him up gently and ask what he want for breakfast.
Then of course fetch that breakfast, bring it to the homeowner and optionally make the bed.
"""

import rospy
import smach
import sys
import random
import smach_ros

import wakemeup_states as wakeStates
from ed.msg import EntityInfo
from robot_smach_states.util.designators import Designator, VariableDesignator, EdEntityDesignator, ArmDesignator, LockingDesignator
import robot_smach_states as states
from robot_smach_states.util.startup import startup
from robot_smach_states import Grab
from robot_smach_states import Place
from robot_skills.util import msg_constructors as geom
from robot_skills.util import transformations
from dragonfly_speech_recognition.srv import GetSpeechResponse

from robot_skills.amigo import Amigo
from robot_skills.sergio import Sergio
from robot_skills.mockbot import Mockbot

from robocup_knowledge import load_knowledge
knowledge = load_knowledge('challenge_wakemeup')
knowledge_objs = load_knowledge('common').objects

# ----------------------------------------------------------------------------------------------------

class WakeMeUp(smach.StateMachine):
    def __init__(self, robot):
        smach.StateMachine.__init__(self, outcomes=['Done','Aborted'])

        # ------------------------ DEFINITIONS ------------------------

        def is_awake(entity):
            return True

        def is_not_bed(entity):
            return entity.id != knowledge.bed

        def is_not_prior(entity):
            return entity.type == '' or entity.type == 'human'

        def is_just_above_bed(entity):
            bed = robot.ed.get_entity(knowledge.bed)

            # Check if top of found object is above a certain threshold above the bed
            if entity.pose.position.z + entity.z_max < bed.pose.position.z + knowledge.matress_height + knowledge.find_person['under_z']:
                return False

            # Check if center point of entity is within chull of bed
            bed_chull = bed.convex_hull
            bed_chull = bed_chull + [bed_chull[0]]

            for i in range(len(bed_chull)-1):
                dx = bed_chull[i+1].x - bed_chull[i].x
                dy = bed_chull[i+1].y - bed_chull[i].y

                dxe = -bed_chull[i].x
                dye = -bed_chull[i].y

                # Cross product of these two gives either positive or negative, if one is negative, point is outside the chull
                if dx*dye-dy*dxe < 0:
                    return False

            return True

        def probably_exists(entity):
            return entity.existence_probability > knowledge.find_person['min_exist_prob']

        def is_large_enough(entity):
            if len(entity.convex_hull) < 3:
                return False

            chull = [entity.convex_hull[-1]] + entity.convex_hull + [entity.convex_hull[0]]
            area = 0
            for ppt, pt, npt in zip(chull, chull[1:], chull[2:]):
                area += pt.x*( npt.y - ppt.y );
            area /= 2

            return area > knowledge.find_person['min_chull_area']


        # ------------------------ INITIALIZATIONS ------------------------

        initial_ed_config = {'kinect_segmentation_continuous_mode':True, 'perception_continuous_mode':False, 'disabled_plugins':[]}

        armDesignator = LockingDesignator(ArmDesignator(robot.arms))

        # Waking up the operator
        bed_des = EdEntityDesignator(robot, id=knowledge.bed)

        entityOnBedDesignator = EdEntityDesignator( robot, 
                                                    center_point = robot.ed.get_entity(knowledge.bed).pose.position, 
                                                    radius = knowledge.find_person['within_range'], 
                                                    criteriafuncs = [   is_large_enough, 
                                                                        probably_exists, 
                                                                        is_just_above_bed, 
                                                                        is_not_bed, 
                                                                        is_not_prior])

        time_marker = VariableDesignator(resolve_type=type(rospy.Time.now()))         # Time to repeat wakeup loop

        # Taking breakfast order
        breakfastCerealDes = VariableDesignator("")       # designator containing chosen cereal name
        breakfastFruitDes  = VariableDesignator("")       # designator containing chosen fruit name
        breakfastMilkDes   = VariableDesignator("")       # designator containing chosen milk name
        orderConfirmationCounter = VariableDesignator(0)

        # Getting the order from the kitchen
        waypoint_kitchen       = EdEntityDesignator(robot, id="wakemeup_kitchen_table")
        selected_gen_item_des  = VariableDesignator("")
        selected_spec_item_des = VariableDesignator("")

        prep_eval_des = VariableDesignator({})
        for item in knowledge.generic_items: 
            prep_eval_des.current[item] = False

        current_item_nav_goal_at = VariableDesignator({})
        current_item_nav_goal_lookat = VariableDesignator(resolve_type=Designator)
        
        item_designator = VariableDesignator(resolve_type=EntityInfo)

        place_position = wakeStates.EmptySpotDesignator(robot, EdEntityDesignator(robot, id=knowledge.dinner_table))
        

        # ------------------------ STATE MACHINE ------------------------

        with self:
            smach.StateMachine.add( 'GET_NEWSPAPER',
                                    states.HandoverFromHuman(robot, armDesignator, grabbed_entity_label="newspaper", timeout=knowledge.get_newspaper_timeout),
                                    transitions={   'succeeded':'INITIALIZE',
                                                    'failed'   :'INITIALIZE'})

            smach.StateMachine.add( 'INITIALIZE',
                                    wakeStates.Initialize(robot,initial_ed_config),
                                    transitions={   'initialized':'START_CHALLENGE',
                                                    'abort':'START_CHALLENGE'})

            smach.StateMachine.add( 'START_CHALLENGE',
                                    states.StartChallengeRobust(robot, knowledge.initial_pose),
                                    transitions={   'Done'              :'SAY_OPERATOR_AWAKE',
                                                    'Aborted'           :'SAY_OPERATOR_AWAKE',
                                                    'Failed'            :'SAY_OPERATOR_AWAKE'})

            smach.StateMachine.add( "SAY_OPERATOR_AWAKE",
                                    states.Say(robot, "Lets see if my operator is awake", block=False), 
                                    transitions={   "spoken"            :"GO_TO_BED"})

            smach.StateMachine.add( 'GO_TO_BED',
                                    states.NavigateToSymbolic(robot, {
                                        # EdEntityDesignator(robot, id=knowledge.bed_nav_goal['near']) : "near", 
                                        EdEntityDesignator(robot, id=knowledge.bed_nav_goal['at_bedside']) : "at_bedside", 
                                        EdEntityDesignator(robot, id=knowledge.bed_nav_goal['in']) : "in" },
                                        EdEntityDesignator(robot, id=knowledge.bed_nav_goal['lookat'])),
                                    transitions={   'arrived'           :'WAKEUP_CONTAINER',
                                                    'unreachable'       :'GO_TO_BED_BACKUP',
                                                    'goal_not_defined'  :'GO_TO_BED_BACKUP'})

            smach.StateMachine.add( 'GO_TO_BED_BACKUP',
                                    states.NavigateToSymbolic(robot, {
                                        EdEntityDesignator(robot, id=knowledge.bed_nav_goal['at_bedside']) : "at_bedside", 
                                        EdEntityDesignator(robot, id=knowledge.bed_nav_goal['in']) : "in" },
                                        EdEntityDesignator(robot, id=knowledge.bed_nav_goal['lookat'])),
                                    transitions={   'arrived'           :'WAKEUP_CONTAINER',
                                                    'unreachable'       :'WAKEUP_CONTAINER',
                                                    'goal_not_defined'  :'WAKEUP_CONTAINER'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 WAKEUP_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            wakeupContainer = smach.StateMachine(outcomes = ['container_succeeded', 'container_failed'])

            with wakeupContainer:

                smach.StateMachine.add( 'CANCEL_HEAD_GOALS',
                                        wakeStates.CancelHeadGoals(robot),
                                        transitions={   'done':'LOOK_AT_BED'})

                smach.StateMachine.add( 'LOOK_AT_BED',
                                        wakeStates.LookAtBedTop(robot, knowledge.bed),
                                        transitions={   'succeeded':'SET_TIME_MARKER'})

                smach.StateMachine.add( "SET_TIME_MARKER",
                                        states.SetTimeMarker(robot, time_marker),
                                        transitions={   'done' :'WAKEUP_MESSAGE'})

                smach.StateMachine.add( "WAKEUP_MESSAGE",
                                        states.Say(robot, [ "Rise and shine", 
                                                            "Wakey wakey!",
                                                            "Wake up boss", 
                                                            "Hello there sleepy head! Please get up", 
                                                            "Time for breakfast!"], block=True),
                                        transitions={   'spoken' :'LOOK_IF_AWAKE'})

                smach.StateMachine.add( "LOOK_IF_AWAKE",
                                        wakeStates.LookIfSomethingsThere(robot, entityOnBedDesignator, timeout=knowledge.alarm_wait_time),
                                        transitions={    'there'    :'SAY_AWAKE',
                                                         'not_there':'CHECK_TIME'})

                smach.StateMachine.add( "CHECK_TIME",
                                        states.CheckTime(robot,  time_marker, knowledge.alarm_duration),
                                        transitions={   'ok'        :'WAKEUP_MESSAGE',
                                                        'timeout'   :'SAY_AWAKE'})

                smach.StateMachine.add( "SAY_AWAKE",
                                        states.Say(robot, ["Finally, you're awake, I will hand you your newspaper now"], block=True),
                                        transitions={   'spoken'    :'HANDOVER_NEWSPAPER'})

                smach.StateMachine.add( "HANDOVER_NEWSPAPER",
                                        states.HandoverToHuman(robot, armDesignator, timeout=knowledge.give_newspaper_timeout),
                                        transitions={   'succeeded' :'container_succeeded',
                                                        'failed'    :'container_succeeded'})

            smach.StateMachine.add( 'WAKEUP_CONTAINER',
                                    wakeupContainer,
                                    transitions={   'container_succeeded':'CANCEL_HEAD_GOALS_1',
                                                    'container_failed': 'CANCEL_HEAD_GOALS_1'})

            smach.StateMachine.add( 'CANCEL_HEAD_GOALS_1',
                                    wakeStates.CancelHeadGoals(robot),
                                    transitions={    'done':'TAKE_ORDER_CONTAINER'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 TAKE_ORDER_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            takeOrderContainer = smach.StateMachine(outcomes = ['container_succeeded'])
            with takeOrderContainer:

                smach.StateMachine.add( "COUNTER",
                                        wakeStates.Counter(counter=orderConfirmationCounter,limit=knowledge.order_confirmation_limit),
                                        transitions={   'counted'       :'SAY_WHAT_BREAKFAST',
                                                        'limit_reached' :'SAY_ILL_CHOOSE_BREAKFAST'})

                smach.StateMachine.add( "SAY_WHAT_BREAKFAST",
                                        states.Say(robot, [ "What would you like to have for breakfast?", 
                                                            "Please tell me your breakfast order.", 
                                                            "What do you want to eat?"], block=True),
                                        transitions={   'spoken' :'GET_ORDER'})

                smach.StateMachine.add( "GET_ORDER",
                                        wakeStates.GetOrder(robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes),
                                        transitions={   'succeeded' :   'SAY_REPEAT_ORDER',
                                                        'failed'    :   'SAY_INCORRECT_ORDER'})

                smach.StateMachine.add( "SAY_REPEAT_ORDER",
                                        wakeStates.ConfirmOrder(robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes),
                                        transitions={   'done'      :'HEAR_IF_CORRECT'})

                smach.StateMachine.add( "HEAR_IF_CORRECT",
                                        states.HearYesNo(robot),
                                        transitions={   'heard_yes' :'SAY_ALRIGHT',
                                                        'heard_no'  :'COUNTER',
                                                        'heard_failed':"COUNTER"})

                smach.StateMachine.add( "SAY_INCORRECT_ORDER",
                                        states.Say(robot, [ "I didn't get that.",
                                                            "I missunderstood something," ], block=False),
                                        transitions={   'spoken'    :'COUNTER'})

                smach.StateMachine.add( "SAY_ILL_CHOOSE_BREAKFAST",
                                        states.Say(robot, "I couldn't understand the breakfast order. I'll choose something for you.", block=False),
                                        transitions={   'spoken'    :'PICK_DEFAULT_ORDER'})

                smach.StateMachine.add( "PICK_DEFAULT_ORDER",
                                        wakeStates.PickDefaultOrder(breakfastCerealDes, breakfastFruitDes, breakfastMilkDes),
                                        transitions={   'done'      :'SAY_REPEAT_ORDER'})

                smach.StateMachine.add( "SAY_DEFAULT_ORDER",
                                        states.Say(robot, "I will bring you a "+knowledge.default_fruit+", "+knowledge.default_cereal+" and "+knowledge.default_milk),
                                        transitions={   'spoken'      :'container_succeeded'})

                smach.StateMachine.add( "SAY_ALRIGHT",
                                        states.Say(robot, "Alright!"),
                                        transitions={   'spoken'      :'container_succeeded'})


            smach.StateMachine.add( 'TAKE_ORDER_CONTAINER',
                                    takeOrderContainer,
                                    transitions={   'container_succeeded':'GOTO_KITCHEN_CONTAINER'})



            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 GOTO_KITCHEN_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            goToKitchenContainer = smach.StateMachine(outcomes = ['container_succeeded', 'container_failed'])
            with goToKitchenContainer:

                smach.StateMachine.add( "SAY_PREPARING",
                                        states.Say(robot, [ "I'm going to the kitchen" ], block=False),
                                        transitions={   'spoken' :'GOTO_KITCHEN'})

                smach.StateMachine.add( 'GOTO_KITCHEN',
                                        states.NavigateToSymbolic(robot, {
                                            EdEntityDesignator(robot, id=knowledge.kitchen_nav_goal['in']) : "in" }, 
                                            EdEntityDesignator(robot, id=knowledge.kitchen_nav_goal['lookat'])),
                                        transitions={   'arrived':'container_succeeded',
                                                        'unreachable':'SAY_UNREACHABLE',
                                                        'goal_not_defined':'SAY_UNREACHABLE'})

                smach.StateMachine.add( 'SAY_UNREACHABLE',
                                        states.Say(robot, "I can't get to the kitchen"),
                                        transitions={   'spoken'    :'container_failed'})


            smach.StateMachine.add( 'GOTO_KITCHEN_CONTAINER',
                                    goToKitchenContainer,
                                    transitions={   'container_succeeded':'PREP_BREAKFAST_CONTAINER',
                                                    'container_failed': 'PREP_BREAKFAST_CONTAINER'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 PREP_BREAKFAST_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            prepBreakfastContainer = smach.StateMachine(outcomes = ['container_succeeded', 'container_failed'])
            with prepBreakfastContainer:

                smach.StateMachine.add( "SAY_PREPARING",
                                        states.Say(robot, [ "I'm going to prepare your breakfast. La la la la la" ], block=False),
                                        transitions={   'spoken' : 'SELECT_ITEM'})

                smach.StateMachine.add( "SELECT_ITEM",
                                        wakeStates.SelectItem(  robot, 
                                                                options=knowledge.generic_items, 
                                                                asked_items= [ breakfastCerealDes, breakfastMilkDes, breakfastFruitDes ], 
                                                                generic_item=selected_gen_item_des, 
                                                                specific_item=selected_spec_item_des,
                                                                item_nav_goal=current_item_nav_goal_at,
                                                                item_lookat_goal=current_item_nav_goal_lookat),
                                        transitions={   'selected' : 'GOTO_ITEM',
                                                        'all_done' : 'EVALUATE_RESULTS'})

                smach.StateMachine.add( 'GOTO_ITEM',
                                        wakeStates.NavigateToSymbolic(  robot, 
                                                                    current_item_nav_goal_at,
                                                                    current_item_nav_goal_lookat ),
                                        transitions={   'arrived':'FIND_ITEM',
                                                        'unreachable':'SAY_ITEM_UNREACHABLE',
                                                        'goal_not_defined':'SAY_ITEM_UNREACHABLE'})

                smach.StateMachine.add( 'SAY_ITEM_UNREACHABLE',
                                        states.Say(robot, "I'm afraid I can't get to the item"),
                                        transitions={   'spoken'    :'SELECT_ITEM'})

                smach.StateMachine.add( 'FIND_ITEM',
                                        wakeStates.FindItem(robot, 
                                                            sensor_range=2.0, 
                                                            on_object_des=current_item_nav_goal_lookat,
                                                            type_des=selected_spec_item_des, 
                                                            result_des=item_designator),
                                        transitions={   'item_found':'PICK_UP_ITEM',
                                                        'not_found' :'SAY_ITEM_NOT_FOUND'})

                smach.StateMachine.add( 'SAY_ITEM_NOT_FOUND',
                                        states.Say(robot, "I'm afraid I can't find the item"),
                                        transitions={   'spoken'    :'SELECT_ITEM'})

                # TODO: check this!!!
                smach.StateMachine.add( 'PICK_UP_ITEM',
                                        states.Grab(robot, item_designator, armDesignator),
                                        transitions={   'done'      :'GOTO_TABLE',
                                                        'failed'    :'SELECT_ITEM'})

                smach.StateMachine.add( 'GOTO_TABLE',
                                        states.NavigateToSymbolic(robot, {
                                            EdEntityDesignator(robot, id=knowledge.table_nav_goal['in']) : "in",
                                            EdEntityDesignator(robot, id=knowledge.table_nav_goal['in_front_of']) : "in_front_of" }, 
                                            EdEntityDesignator(robot, id=knowledge.table_nav_goal['lookat'])),
                                        transitions={   'arrived'           :'SCAN_TABLE',
                                                        'unreachable'       :'SAY_UNREACHABLE',
                                                        'goal_not_defined'  :'SAY_UNFINDABLE'})

                smach.StateMachine.add( 'SCAN_TABLE',
                                        wakeStates.ScanTableTop(robot, EdEntityDesignator(robot, id=knowledge.dinner_table)),
                                        transitions={   'done'      :'PLACE_ITEM',
                                                        'failed'    :'SAY_UNFINDABLE'})

                smach.StateMachine.add( 'SAY_UNFINDABLE',
                                        states.Say(robot, "I can't find the "+knowledge.dinner_table),
                                        transitions={   'spoken'    :'PLACE_ITEM'})

                smach.StateMachine.add( 'SAY_UNREACHABLE',
                                        states.Say(robot, "I can't get to the "+knowledge.dinner_table),
                                        transitions={   'spoken'    :'PLACE_ITEM'})

                smach.StateMachine.add( 'PLACE_ITEM',
                                        states.Place(robot, item_designator, place_position, armDesignator),
                                        transitions={   'done'      :'ADD_POSITIVE_RESULT',
                                                        'failed'    :'SELECT_ITEM'})

                smach.StateMachine.add( 'ADD_POSITIVE_RESULT',
                                        wakeStates.addPositive( results_designator=prep_eval_des, 
                                                                item_designator=selected_gen_item_des),
                                        transitions={   'done'      :'SELECT_ITEM'})

                smach.StateMachine.add( 'EVALUATE_RESULTS',
                                        wakeStates.Evaluate(knowledge.generic_items, prep_eval_des),
                                        transitions={   'all_succeeded'     :'container_succeeded',
                                                        'all_failed'        :'container_failed',
                                                        'partly_succeeded'  :'container_succeeded'})


            smach.StateMachine.add( 'PREP_BREAKFAST_CONTAINER',
                                    prepBreakfastContainer,
                                    transitions={   'container_succeeded'   :'END_CHALLENGE',
                                                    'container_failed'      :'END_CHALLENGE'})

            # Pour cereal in the boal: find boal, grab cereal, move arm with cereal to predefined position, make rotating movement (pour), 
            # rotate back, put cereal back on table. 

            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 DELIVER_BREAKFAST_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            # deliverBreakfastContainer = smach.StateMachine(outcomes = ['container_succeeded', 'container_failed'])
            # with deliverBreakfastContainer:

            #     smach.StateMachine.add( 'GOTO',
            #                             states.NavigateToSymbolic(robot, 
            #                                 {EdEntityDesignator(robot, id=knowledge.bed_nav_goal['near']) : "near", 
            #                                  EdEntityDesignator(robot, id=knowledge.bed_nav_goal['at_bedside']) : "at_bedside", 
            #                                  EdEntityDesignator(robot, id=knowledge.bed_nav_goal['in']) : "in" }, 
            #                                  EdEntityDesignator(robot, id=knowledge.bed_nav_goal['lookat'])),
            #                             transitions={   'arrived':'container_succeeded',
            #                                             'unreachable':'SAY_COULD_NOT_PREPARE',
            #                                             'goal_not_defined':'SAY_COULD_NOT_PREPARE'})

            #     smach.StateMachine.add( "SAY_COULD_NOT_PREPARE",
            #                             states.Say(robot, [ "I'm sorry but i could not prepare your breakfast." ], block=False),
            #                             transitions={   'spoken' :'container_succeeded'})

            #     smach.StateMachine.add( 'SAY_UNREACHABLE',
            #                             states.Say(robot, "I can't get there"),
            #                             transitions={   'spoken'    :'container_failed'})

            # smach.StateMachine.add( 'DELIVER_BREAKFAST_CONTAINER',
            #                         deliverBreakfastContainer,
            #                         transitions={   'container_succeeded':'END_CHALLENGE',
            #                                         'container_failed': 'END_CHALLENGE'})


            smach.StateMachine.add( 'END_CHALLENGE',
                                    states.Say(robot,"That was all I can do. Goodbye!"),
                                    transitions={'spoken':'Done'})



# ------------------------ MAIN ------------------------


if __name__ == '__main__':
    rospy.init_node('wakemeup_exec')

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]
    else:
        print "[CHALLENGE MANIPULATION] Please provide robot name as argument."
        exit(1)

    if robot_name == 'amigo':
        robot = Amigo(wait_services=True)
    elif robot_name == 'sergio':
        robot = Sergio(wait_services=True)
    elif robot_name == 'mockbot':
        robot = Mockbot(wait_services=True)
    else:
        print "Don't recognize that robot name: " + robot_name

    ''' If necessary: set initial state '''
    rospy.loginfo("Sys.argv = {0}, Length = {1}".format(sys.argv,len(sys.argv)))

    ''' Setup state machine'''
    machine = WakeMeUp(robot)

    if  len(sys.argv) > 2:
        print wakeStates.bcolors.WARNING + "Overriding initial_state to '" + sys.argv[2] +  "'" + wakeStates.bcolors.ENDC

        initial_state = [sys.argv[2]]
        machine.set_initial_state(initial_state)

    # for using smach viewer
    introserver = smach_ros.IntrospectionServer('server_name', machine, '/SM_ROOT_PRIMARY')
    introserver.start()

    try:
        machine.execute()
        # startup(WakeMeUp, robot_name=robot_name)
    except Exception, e:
        print "Exception occurred on state machine execution"

    introserver.stop()
