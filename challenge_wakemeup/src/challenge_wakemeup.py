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

            # Check if z coordinate is within specified range
            if entity.pose.position.z + entity.z_max < bed.pose.position.z + bed.z_max + knowledge.find_person['under_z']:
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

        armDesignator = LockingDesignator(ArmDesignator(robot.arms))

        # Waking up the operator
        bed_des = EdEntityDesignator(robot, id=knowledge.bed)

        entityOnBedDesignator = EdEntityDesignator(robot, 
            center_point = robot.ed.get_entity(knowledge.bed).pose.position, 
            radius = knowledge.find_person['within_range'], 
            criteriafuncs = [is_large_enough, probably_exists, is_just_above_bed, is_not_bed, is_not_prior])

        wakeup_time_marker = VariableDesignator(resolve_type=type(rospy.Time.now()))         # Time to repeat wakeup loop

        # Taking breakfast order
        spec = Designator("I want <fruit_snack> with <cereal> and <milk> for breakfast")
        choices = Designator(knowledge.breakfast_order_options)
        answer = VariableDesignator(resolve_type=GetSpeechResponse)

        breakfastCerealDes = VariableDesignator("")       # designator containing chosen cereal name
        breakfastFruitDes = VariableDesignator("")        # designator containing chosen fruit name
        breakfastMilkDes = VariableDesignator("")         # designator containing chosen milk name
        loop_counter_des = VariableDesignator(0)         # counter for general looping (because smach iterator sucks)

        # Getting the order from the kitchen
        waypoint_kitchen = EdEntityDesignator(robot, id="wakemeup_kitchen_table")
        

        # ------------------------ STATE MACHINE ------------------------

        with self:
            smach.StateMachine.add('GET_NEWSPAPER',
                                states.HandoverFromHuman(robot, armDesignator, grabbed_entity_label="newspaper", timeout=knowledge.give_newspaper_timeout),
                                transitions={   'succeeded':'INITIALIZE',
                                                'failed'   :'INITIALIZE'})

            smach.StateMachine.add('INITIALIZE',
                                states.Initialize(robot),
                                transitions={   'initialized':'START_CHALLENGE',
                                                'abort':'Aborted'})

            smach.StateMachine.add( 'START_CHALLENGE',
                                    states.StartChallengeRobust(robot, 'initial_pose'),
                                    transitions={   'Done'              :'SAY_OPERATOR_AWAKE',
                                                    'Aborted'           :'Aborted',
                                                    'Failed'            :'SAY_OPERATOR_AWAKE'})

            smach.StateMachine.add( "SAY_OPERATOR_AWAKE",
                                    states.Say(robot, "Lets see if my operator is awake", block=False), 
                                    transitions={   "spoken"            :"GO_TO_BED"})

            smach.StateMachine.add('GO_TO_BED',
                                    states.NavigateToSymbolic(robot, 
                                        {EdEntityDesignator(robot, id=knowledge.bed_nav_goal['near']) : "near", 
                                         EdEntityDesignator(robot, id=knowledge.bed_nav_goal['in']) : "in" }, 
                                         EdEntityDesignator(robot, id=knowledge.bed_nav_goal['lookat'])),
                                    transitions={   'arrived'           :'WAKEUP_CONTAINER',
                                                    'unreachable'       :'GO_TO_BED',
                                                    'goal_not_defined'  :'SAY_NO_BED'})

            smach.StateMachine.add( "SAY_NO_BED",
                                    states.Say(robot, "The bed is not defined in my world model", block=False), 
                                    transitions={   "spoken"            :"END_CHALLENGE"})


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
                                        wakeStates.LookAtBedTop(robot, knowledge.bed, wakeup_light_color=knowledge.wakeup_light_color),
                                        transitions={   'succeeded':'SET_TIME_MARKER'})

                smach.StateMachine.add( "SET_TIME_MARKER",
                                        states.SetTimeMarker(robot,wakeup_time_marker),
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
                                        transitions={    'awake':'SAY_AWAKE',
                                                         'not_awake':'CHECK_TIME'})

                smach.StateMachine.add( "CHECK_TIME",
                                        states.CheckTime(robot, wakeup_time_marker, knowledge.alarm_duration),
                                        transitions={   'ok'        :'WAKEUP_MESSAGE',
                                                        'timeout'   :'SAY_AWAKE'})

                smach.StateMachine.add( "SAY_AWAKE",
                                        states.Say(robot, ["Finally, you're awake, I will hand you your newspaper"], block=True),
                                        transitions={   'spoken' :'HANDOVER_NEWSPAPER'})

                smach.StateMachine.add( "HANDOVER_NEWSPAPER",
                                        states.HandoverToHuman(robot, armDesignator, timeout=knowledge.give_newspaper_timeout),
                                        transitions={   'succeeded':'container_succeeded',
                                                        'failed':'container_succeeded'})

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
            takeOrderContainer = smach.StateMachine(outcomes = ['container_successed', 'container_failed'])
            with takeOrderContainer:

                smach.StateMachine.add( "SAY_WHAT_BREAKFAST",
                                        states.Say(robot, [ "What would you like to have for breakfast?", 
                                                            "Please tell me your breakfast order.", 
                                                            "What do you want to eat?"], block=True),
                                        transitions={   'spoken' :'GET_ORDER'})

                smach.StateMachine.add( "GET_ORDER",
                                        wakeStates.GetOrder(robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes),
                                        transitions={   'succeeded' :   'SAY_REPEAT_ORDER',
                                                        'failed':       'SAY_INCORRECT_ORDER'})

                smach.StateMachine.add( "SAY_INCORRECT_ORDER",
                                        states.Say(robot, [ "I didn't get that.",
                                                            "I missunderstood something," ], block=False),
                                        transitions={   'spoken' :'SAY_WHAT_BREAKFAST'})

                smach.StateMachine.add( "SAY_REPEAT_ORDER",
                                        wakeStates.RepeatOrderToPerson(robot, breakfastCerealDes, breakfastFruitDes, breakfastMilkDes),
                                        transitions={   'done' :'container_successed'})

            smach.StateMachine.add( 'TAKE_ORDER_CONTAINER',
                                    takeOrderContainer,
                                    transitions={   'container_successed':'PREP_BREAKFAST_CONTAINER',
                                                    'container_failed': 'SAY_ILL_CHOOSE_BREAKFAST'})

            smach.StateMachine.add( "SAY_ILL_CHOOSE_BREAKFAST",
                                    states.Say(robot, "I couldn't understand the breakfast order. I'll choose something for you.", block=False),
                                    transitions={   'spoken' :'PREP_BREAKFAST_CONTAINER'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 PREP_BREAKFAST_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            prepBreakfastContainer = smach.StateMachine(outcomes = ['container_successed', 'container_failed'])
            with prepBreakfastContainer:

                smach.StateMachine.add( "SAY_PREPARING",
                                        states.Say(robot, [ "I'm preparing your breakfast! La la la la la." ], block=False),
                                        transitions={   'spoken' :'GOTO_KITCHEN'})

                smach.StateMachine.add('GOTO_KITCHEN',
                                    states.NavigateToWaypoint(robot, waypoint_kitchen),
                                    transitions={   'arrived':'container_successed',
                                                    'unreachable':'container_successed',
                                                    'goal_not_defined':'container_successed'})

            smach.StateMachine.add( 'PREP_BREAKFAST_CONTAINER',
                                    prepBreakfastContainer,
                                    transitions={   'container_successed':'DELIVER_BREAKFAST_CONTAINER',
                                                    'container_failed': 'DELIVER_BREAKFAST_CONTAINER'})


            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            #                                 DELIVER_BREAKFAST_CONTAINER
            # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

            # container for this stage
            deliverBreakfastContainer = smach.StateMachine(outcomes = ['container_successed', 'container_failed'])
            with deliverBreakfastContainer:

                smach.StateMachine.add('GOTO_BEDSIDE',
                                    states.NavigateToSymbolic(robot, 
                                        {EdEntityDesignator(robot, id=knowledge.bed_nav_goal['near']) : "near", 
                                         EdEntityDesignator(robot, id=knowledge.bed_nav_goal['in']) : "in" }, 
                                         EdEntityDesignator(robot, id=knowledge.bed_nav_goal['lookat'])),
                                    transitions={   'arrived':'SAY_COULD_NOT_PREPARE',
                                                    'unreachable':'SAY_COULD_NOT_PREPARE',
                                                    'goal_not_defined':'SAY_COULD_NOT_PREPARE'})

                smach.StateMachine.add( "SAY_COULD_NOT_PREPARE",
                                        states.Say(robot, [ "I'm sorry but i could not prepare your breakfast." ], block=False),
                                        transitions={   'spoken' :'container_successed'})

            smach.StateMachine.add( 'DELIVER_BREAKFAST_CONTAINER',
                                    deliverBreakfastContainer,
                                    transitions={   'container_successed':'END_CHALLENGE',
                                                    'container_failed': 'END_CHALLENGE'})


            smach.StateMachine.add('END_CHALLENGE',
                                   states.Say(robot,"My work here is done, goodbye!"),
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
