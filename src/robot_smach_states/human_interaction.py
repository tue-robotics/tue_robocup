#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import random
import util

from std_msgs.msg import String
import threading

from speech_interpreter.srv import AskUser

from utility_states import Wait_time, PlaySound

TTS_waittime = 0.000 #TODO: Needs to be measured and be made as long as possible.

@util.deprecated_replace_with("speech_interpreter")
class Dummy_Question(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['target_room_received','no_target_room_received'], 
                             input_keys=['rate','similar_answers_dictionary','answer', 'world_info','target','exploration_targets_longlist'],
                             output_keys=['room','exploration_targets','answer'])
        
        self.start_time = rospy.Time.now()
        self.answer = ""
        self.sentence = ""    
        self.question_answered = False   
        self.room = ""
    
    def execute(self, gl):
        
        '''self.room = 'livingroom'
        if self.room == 'livingroom':
            gl.exploration_targets = load_exploration_targets(gl.exploration_targets_longlist,'livingroom')
        elif self.room == 'kitchen':
            gl.exploration_targets = load_exploration_targets(gl.exploration_targets_longlist,'kitchen')
        elif self.room == 'laundry':
            gl.exploration_targets = load_exploration_targets(gl.exploration_targets_longlist,'laundry')
        elif self.room == 'bedroom':
            gl.exploration_targets = load_exploration_targets(gl.exploration_targets_longlist,'bedroom')
        elif self.room == 'corridor':
            gl.exploration_targets = load_exploration_targets(gl.exploration_targets_longlist,'corridor')
        elif self.room == 'hospital':
            gl.exploration_targets = load_exploration_targets(gl.exploration_targets_longlist,'hospital')
        elif self.room == 'autolab':
            gl.exploration_targets = load_exploration_targets(gl.exploration_targets_longlist,'autolab')
        else:
            return 'no_target_room_received' '''
        
        #gl.exploration_targets = load_exploration_targets(gl.exploration_targets_longlist,'livingroom')
        #gl.room = 'livingroom'

        gl.answer = 'kitchen'
        
        return 'target_room_received'

############################## Hierarchical StateMachine QuestionMachine  ###################################
@util.deprecated_replace_with("speech_interpreter")
class QuestionMachine(smach.StateMachine):
    
    #TODO: Add timeouts. DONE, in Timedout_QuestionMachine
    #TODO: let ears start listening *before* the question is posed.

    def __init__(self, 
                 default_option, sentence=None, robot = None,
                 options=None,  
                 maxtime=30,
                 max_confirm_asks=3,
                 recite_options=False):
        smach.StateMachine.__init__(self, 
                                   outcomes=["answered", "not_answered"],
                                   input_keys=["question_sentence", "options"],
                                   output_keys=["answer", "options"])
        self.robot = robot
        assert isinstance(default_option,str) or not default_option #Check if the default option is set or explicitly False/None
        
        with self:
            if sentence:
                self.userdata.question_sentence = sentence
            if options:
                self.userdata.options = options
                
            if default_option:
                '''Use the default option'''
                pass
            else:
                '''quit with not_answered'''
                pass
            
            smach.StateMachine.add("ASK", 
                                   Ask(self.robot, recite_options),
                                   transitions={'question_asked':'BEEP'},
                                   remapping={'options':'options', 
                                              'question_sentence':'question_sentence'})

            smach.StateMachine.add("BEEP", 
                                   PlaySound("~/ros/pkgs/tue-ros-pkg/dev/tue_executioner/robot_smach_states/beep-6.mp3", 
                                    blocking=True),
                                   transitions={'played':'WAIT_ANSWER',
                                                'error':'WAIT_ANSWER'})
            

            if default_option:
                smach.StateMachine.add("WAIT_ANSWER", 
                                       Wait_Answer_Ears(self.robot, maxtime),
                                       transitions={'invalid':'ASK',
                                                    'answer_received':'CONFIRM_ANSWER',
                                                    'no_answer':'SET_DEFAULT_OPTION',
                                                    'empty_answer':'WAIT_ANSWER'})
            else:
                smach.StateMachine.add("WAIT_ANSWER", 
                                       Wait_Answer_Ears(self.robot, maxtime),
                                       transitions={'invalid':'ASK',
                                                    'answer_received':'CONFIRM_ANSWER',
                                                    'no_answer':'not_answered', #THIS TRANSITION IS DIFFERENT
                                                    'empty_answer':'WAIT_ANSWER'})

            confirmation_iterator = smach.Iterator(outcomes=['confirmed', 'disconfirmed', 'no_confirm'],
                                                   input_keys=["question_sentence", "options", "answer"],
                                                   output_keys=["answer"],
                                                   it=lambda: range(max_confirm_asks), 
                                                   it_label='confirmation_try', 
                                                   exhausted_outcome='confirmed')
            
            with confirmation_iterator:
                confirmation_sm = smach.StateMachine(outcomes=['confirmed', 'disconfirmed', 'no_confirm'], 
                                                     input_keys=["question_sentence", "options", "answer"],
                                                     output_keys=['answer'])
                with confirmation_sm:
                    smach.StateMachine.add('CONFIRM_ANSWER_SUB',
                                           Confirm_Answer(self.robot),
                                           transitions={'done':'confirmed',
                                                        'disconfirmed':'disconfirmed',
                                                        'no_confirm':'no_confirm'})
                
                confirmation_iterator.set_contained_state('CONFIRMATION_SM', 
                                                          confirmation_sm, 
                                                          loop_outcomes=['no_confirm'], 
                                                          break_outcomes=['confirmed', 'disconfirmed'])
            
            smach.StateMachine.add('CONFIRM_ANSWER', 
                                   confirmation_iterator, 
                                   transitions={'confirmed':'REMEMBER_ANSWER', 
                                                'disconfirmed':'ASK', 
                                                'no_confirm':'not_answered'})

            @smach.cb_interface(outcomes=['done'], 
                                input_keys=['answer'], 
                                output_keys=['answer'])
            def remember_answer(userdata):
                
                # assert the knowledge corresponding to the answer (stored in 'options') to the reasoner
                fact = options[userdata.answer]
                robot.reasoner.assertz(fact)
                return 'done'
            smach.StateMachine.add('REMEMBER_ANSWER', smach.CBState(remember_answer),
                                    transitions={'done':'answered'})
            
            @smach.cb_interface(outcomes=['stored'], 
                                input_keys=['answer'], 
                                output_keys=['answer'])
            def set_default_option(userdata):
                robot.speech.speak("I did not get a correct answer timely, so I will assume you said {0}".format(default_option))
                userdata.answer = default_option
                return 'stored'
            
            smach.StateMachine.add('SET_DEFAULT_OPTION',
                                   smach.CBState(set_default_option),
                                   transitions={'stored':'REMEMBER_ANSWER'})
  
@util.deprecated_replace_with("speech_interpreter")
class Ask(smach.State):
    def __init__(self, robot, recite_options=False):
        smach.State.__init__(self,
                             outcomes=["question_asked"],
                             input_keys=["question_sentence", "options"])
        self.robot = robot
        self.recite_options = recite_options
    
    def execute(self, gl):
        rospy.loginfo("Clearing ears :-)")
        self.robot.ears.forget()
        self.robot.ears.start_listening() #Start listening before speaking, and thus hear yourself potentially.
        #amigo.Ears must handle this by selecting only those words spoken AFTER we stopped talking, as set by TTS_waittime
        self.robot.speech.speak(gl.question_sentence)
        if 'options' in gl.keys() and gl.options and self.recite_options: #an empty list evaluates to False or if no options are given, gl.options = None, which also evaluates to False
            self.robot.speech.speak("These are your options: "+self.robot.speech.buildList(gl.options, "or"))
        
        return "question_asked"
    
@util.deprecated_replace_with("Ask_query_true")
class tueMonitorState(smach.State):
    """The tueMonitorState takes a callback, a topic and a messagetype as arguments. 
       This callback takes userdata and a message of the given type.
       It must return True _and_ the message if the message is the correct trigger or
       return False and the message if the message is not the trigger we waited on"""
    def __init__(self, callback, topic, _type):
        smach.State.__init__(self,
                             outcomes=["valid", "invalid", "preempted"],
                             output_keys=["value"])
        
        self._trigger_cond = threading.Condition()
        self._max_checks = 1
        self._n_checks = 0
        
        self._cond_cb = callback
        self.topic = topic
        self.type = _type
        
        self.monitor_passed = None
        self.passed_value = None
        
    def _cb(self, msg, ud):
        rospy.logdebug("self._cb({0}, {1})".format(msg, ud.keys()))
        
        self._n_checks += 1
        try:
            output =  self._cond_cb(ud, msg) #This should return 2 values: if the condition is passed and the value that made it pass
            if len(output) != 2:
                rospy.logwarn("callback in tueMonitorState should return a boolean indicating if it passed and secondly the value that made the callback return that boolean")
                self.monitor_passed = output
            else:
                self.monitor_passed = output[0]
                self.passed_value = output[1]
            
            self._trigger_cond.acquire()
            self._trigger_cond.notify()
            self._trigger_cond.release()
        except:
            rospy.logerr("Error thrown while executing condition callback %s" % str(self._cond_cb))
            self._trigger_cond.acquire()
            self._trigger_cond.notify()
            self._trigger_cond.release()
    
    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()
    
    def execute(self, userdata):
        """Wait for the users' answer using a Subscriber in a separate thread 
        that signals that the original thread that the answer is received"""
        
        #I made my own implementation of smach_ros.MonitorState here.
        self._sub = rospy.Subscriber(self.topic, 
                                     self.type, 
                                     self._cb, 
                                     callback_args=userdata)

        self._trigger_cond.acquire()
        rospy.loginfo("tueMonitorState waiting for ROS message...")
        self._trigger_cond.wait()
        rospy.loginfo("message received")
        self._trigger_cond.release()

        self._sub.unregister()
        
        userdata.value = self.passed_value
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self.monitor_passed:
            rospy.loginfo("self.monitor_passed=True, returning valid")
            return 'valid'
        
        rospy.loginfo("self.monitor_passed=False, returning invalid")
        return 'invalid'

@util.deprecated_replace_with("Await_Text_Ears")
class Wait_Answer(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,
                             outcomes=["answer_received", "invalid", "no_answer_yet", "no_answer", 'empty_answer'],
                             input_keys=["question_sentence", "options", "answer"],
                             output_keys=["answer"])
        self.robot = robot
        
        self._trigger_cond = threading.Condition()
        self._max_checks = 1
        self._n_checks = 0
        
        self._cond_cb = self.wait_answer_callback
        
        self.answer = None
        self.result = None
        
    def _cb(self, msg, ud):
        rospy.logdebug("self._cb({0}, {1}".format(msg, ud.keys()))
        
        self._n_checks += 1
        try:
            self.result =  self._cond_cb(ud, msg)
            self._trigger_cond.acquire()
            self._trigger_cond.notify()
            self._trigger_cond.release()
                
            # if not self.wait_answer_callback(ud, msg) or (self._max_checks > 0 and self._n_checks >= self._max_checks):
                #------------------------------------------- self.result = False
                #---------------------------------- self._trigger_cond.acquire()
                #----------------------------------- self._trigger_cond.notify()
                #---------------------------------- self._trigger_cond.release()
            #------------------------------------------------------------- else:
                #---------- print "Setting the condition although it is NOT met"
                #-------------------------------------------- self.result = True
                #---------------------------------- self._trigger_cond.acquire()
                #----------------------------------- self._trigger_cond.notify()
                #---------------------------------- self._trigger_cond.release()
        except:
            rospy.logerr("Error thrown while executing condition callback %s" % str(self._cond_cb))
            self._trigger_cond.acquire()
            self._trigger_cond.notify()
            self._trigger_cond.release()
    
    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release()
    
    def wait_answer_callback(self, userdata, msg):
        rospy.loginfo("userdata: {0}, msg: {1}".format(userdata.keys(), msg.data))
        
        try:
            options = userdata.options# This attribute may not exist
        except Exception, e:
            rospy.logwarn("No options given in userdata. This could be intended though.")
            options = None
        answer = msg.data.lower()
        
        #time.sleep(10)
        
        self.answer = answer

         #If there are options defined, check if the answer is indeed an option
        #Else take any answer
        rospy.loginfo("Checking the answer now")
        if options:
            if answer in options:
                rospy.loginfo("Answer {0} in options ({1}), return True".format(answer, options))                
                options[answer]
                return True
            else:
                rospy.loginfo("Answer {0} NOT in options ({1}), return False".format(answer, options))
                return False
        else:
            rospy.loginfo("Answer {0} given, no options, return True".format(answer))
            return True
    
    def execute(self, userdata):
        """Wait for the users' answer using a Subscriber in a separate thread 
        that signals that the original thread that the answer is received"""
        #=======================================================================
        # rospy.loginfo("Wait_Answer.execute(userdata.keys={0})".format(userdata.keys()))
        # rospy.loginfo("userdata.options = {0}".format(userdata.options))
        # rospy.loginfo("self.get_registered_input_keys() = {0}".format(self.get_registered_input_keys()))
        # rospy.loginfo("self.get_registered_output_keys() = {0}".format(self.get_registered_output_keys()))
        # rospy.loginfo("Callback is {0}".format(self._cond_cb))
        #=======================================================================
        
        #I made my own implementation of smach_ros.MonitorState here.
        
        rospy.loginfo("Waiting a sec before starting to listen to speech output")
        rospy.sleep(1)
        
        self._sub = rospy.Subscriber("/speech/output", 
                                     String, 
                                     self._cb, 
                                     callback_args=userdata)

        self._trigger_cond.acquire()
        rospy.loginfo("Waiting for the users answer...")
        self._trigger_cond.wait(20.0) #TODO Loy: Make this timeout configurable via userdata
        rospy.loginfo("The user gave an answer")
        self._trigger_cond.release()

        self._sub.unregister()
        
        if self.answer:
            userdata.answer = self.answer
            self.robot.reasoner.assertz(self.answer)
        else:
            return 'empty_answer'
        
        if self.preempt_requested():
            self.service_preempt()
            return 'no_answer' #'preempted'

        #------- if self._max_checks > 0 and self._n_checks >= self._max_checks:
            #--------------------------------- return 'answer_received' #'valid'
        if self.result:
            rospy.loginfo("self.result=True, returning answer_received")
            return 'answer_received'
        
        rospy.loginfo("self.result=False, returning invalid")
        return 'invalid' #'invalid'

@util.deprecated_replace_with("Await_Text_Ears")
class Wait_Answer_Ears(smach.State):
    def __init__(self, robot, maxtime=30):
        smach.State.__init__(self,
                             outcomes=["answer_received", "invalid", "no_answer", "empty_answer"],
                             input_keys=["question_sentence", "options", "answer"],
                             output_keys=["answer"])
        self.robot = robot
        self.maxtime = maxtime

    def execute(self, userdata):
        """Wait for the users' answer polling robot.ears"""
        self.robot.ears.forget()
        self.robot.ears.start_listening()
        waited_time = 0
        sleep_interval = 0.25
        while waited_time < self.maxtime and not rospy.is_shutdown() and not self.preempt_requested():
            if self.preempt_requested():
                self.service_preempt()
                return 'no_answer'
            rospy.sleep(sleep_interval)
            waited_time += sleep_interval

            if "options" in userdata.keys() and userdata.options:
                userdata.answer = self.robot.ears.last_heard_words(userdata.options, TTS_waittime)
                if userdata.answer:
                    rospy.loginfo("Received answer '{0}'".format(userdata.answer))
            else:
                try:
                    userdata.answer = self.robot.ears.get_words_for_duration(TTS_waittime)[-1][0] #Last element of queue, get the text
                    rospy.loginfo("Answer: {0}".format(userdata.answer))
                except:
                    userdata.answer = None
                #return 'empty_answer'
            if userdata.answer:
                userdata.answer = userdata.answer.lower()
                if "options" in userdata.keys() and userdata.options: #check if exists and if not empty
                    if userdata.answer in userdata.options:
                        self.robot.ears.stop_listening()
                        return 'answer_received'
                    else:
                        self.robot.ears.stop_listening()
                        return 'invalid'
                else:
                    self.robot.ears.stop_listening()
                    return 'answer_received'

        self.robot.ears.stop_listening()
        return 'no_answer'    

@util.deprecated_replace_with("speech_interpreter")
class Confirm_Answer(smach.State):
    def __init__(self, robot, maxwait=10):
        smach.State.__init__(self,
                             outcomes=["done", "disconfirmed", "no_confirm"],
                             input_keys=["question_sentence", "options", "answer"],
                             output_keys=["answer"])
        self.robot = robot
        self.maxwait = maxwait
        
    def execute(self, userdata):
        """Wait for the users' answer polling robot.ears"""
        self.robot.speech.speak("I heard {0}, is that correct?".format(userdata.answer))
        #import ipdb; ipdb.set_trace()
        self.robot.ears.forget()
        self.robot.ears.start_listening()
        
        sleep_interval = 0.1
        waited_time = 0
        
        while waited_time < self.maxwait:
            try:
                words = self.robot.ears.get_words_for_duration(TTS_waittime)
                rospy.loginfo("I heard this for confirmation: {0}".format(words))
                confirmation = words[-1][0].lower()
                if confirmation in ['yes', 'ok']:
                    self.robot.ears.stop_listening()
                    return 'done'
                elif confirmation == 'no':
                    self.robot.ears.stop_listening()
                    return 'disconfirmed'
                else:
                    self.robot.ears.stop_listening()
                    return 'no_confirm'
            except IndexError:
                rospy.sleep(sleep_interval)
                waited_time += sleep_interval
                
        self.robot.ears.stop_listening()
        return 'no_confirm'

@util.deprecated_replace_with("speech_interpreter")
class Timedout_QuestionMachine(smach.StateMachine):
    def __init__(self, 
                 default_option, sentence, robot,
                 options=None,
                 maxtime=45,
                 max_confirm_asks=3):
        smach.StateMachine.__init__(self, 
                                   outcomes=["answered", "not_answered"],
                                   input_keys=["question_sentence", "options"],
                                   output_keys=["answer", "options"])
        self.robot = robot
        assert isinstance(default_option,str) or not default_option #Check if the default option is set or explicitly False/None
         
        with self:
            if sentence:
                self.userdata.question_sentence = sentence
            def child_term_cb(outcome_map):
                return True #whatever node has exited, just terminate the CC
        
            max_wait_for_answer_cc = smach.Concurrence(outcomes=['not_answered', 'answered'],
                                                       default_outcome='not_answered', 
                                                       output_keys=['answer'],
                                                       input_keys=['options','question_sentence'], 
                                                       outcome_map={'not_answered':{'ASK_QUESTION':'not_answered',
                                                                                 'QUESTION_TIMEOUT':'waited'},
                                                                    'answered':{'ASK_QUESTION':'answered'}}, 
                                                       child_termination_cb=child_term_cb)
                
            with max_wait_for_answer_cc:
                smach.Concurrence.add("ASK_QUESTION", 
                                      QuestionMachine(robot=self.robot,
                                                        sentence=sentence,
                                                        options=options,
                                                        default_option=default_option))
                smach.Concurrence.add('QUESTION_TIMEOUT', 
                                      Wait_time(robot, maxtime)) #TIMEOUT
                
            smach.StateMachine.add('ASK_QUESTION_TOP',
                                   max_wait_for_answer_cc)

@util.deprecated_replace_with("Await_Text_Ears")
class Await_Text(smach.State):
    def __init__(self, sentence=None, topic="/speech/output", waittime=30):
        smach.State.__init__(self,
                       outcomes=["text_heard", "timed_out", "preempted"],
                       input_keys=["sentence_to_wait_for", "waittime"])
        
        self._trigger_cond = threading.Condition()
        self.result = None
        self.sentence_to_wait_for = sentence
        self.waittime = waittime
        
    def _cb(self, msg, args):
        userdata, sentence = args #unpack the tuple with args
        
        rospy.logdebug("self._cb({0}, ({1}, {2})".format(msg, sentence, userdata.keys()))
        
        try:
            if msg.data.lower() == sentence.lower():
                self._trigger_cond.acquire()
                self._trigger_cond.notify()
                self._trigger_cond.release()
                
                self.result = msg.data
                
        except:
            rospy.logerr("Error thrown while executing condition callback %s" % str(self._cb))
            self._trigger_cond.acquire()
            self._trigger_cond.notify()
            self._trigger_cond.release()
    
    def request_preempt(self):
        rospy.loginfo("Preempt requested")
        smach.State.request_preempt(self)
        self._trigger_cond.acquire()
        self._trigger_cond.notify()
        self._trigger_cond.release
    
    def execute(self, userdata):
        
        '''Choose a sentence to wait for from either the initialization or from the userdata '''
        sentence = self.sentence_to_wait_for
        if "sentence_to_wait_for" in userdata.keys():
            sentence = userdata.sentence_to_wait_for
        
        self._sub = rospy.Subscriber("/speech/output", 
                                     String, 
                                     self._cb, 
                                     callback_args=(userdata, sentence))
        
        waittime = self.waittime
        if "waittime" in userdata.keys():
            waittime = userdata.waittime
        
        self._trigger_cond.acquire()
        rospy.loginfo("Waiting for '{1}' ({0} secs)...".format(waittime, sentence))
        
        self._trigger_cond.wait(waittime)
        rospy.loginfo("The user gave a command")
        self._trigger_cond.release()

        self._sub.unregister()
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if self.result:
            rospy.loginfo("self.result={0}, returning command_heard".format(self.result))
            return 'text_heard'
        
        rospy.loginfo("self.result={0}, returning timed_out".format(self.result))
        return 'timed_out'

class Await_Text_Ears(smach.State):
    def __init__(self, robot, sentence, maxtime=30):
        smach.State.__init__(self,
                             outcomes=["text_heard", "timed_out", "preempted"])
        self.robot = robot
        self.maxtime = maxtime
        self.sentence = sentence

    def execute(self, userdata):
        """Wait for the users' answer polling robot.ears"""
        #self.robot.ears.forget()
        self.robot.ears.start_listening()
        waited_time = 0
        sleep_interval = 0.25
        while waited_time < self.maxtime and not rospy.is_shutdown() and not self.preempt_requested():
            if self.preempt_requested():
                self.service_preempt()
                self.robot.ears.stop_listening()
                return 'preempted'
            
            rospy.sleep(sleep_interval)
            waited_time += sleep_interval

            if self.robot.ears.heard(self.sentence, sleep_interval+TTS_waittime):
                self.robot.ears.stop_listening()
                return "text_heard"

        self.robot.ears.stop_listening()
        return 'timed_out'    

class Say(smach.State):
    def __init__(self, robot, sentence=None, language="us", personality="kyle", voice="default", mood="excited", block=True):
        smach.State.__init__(self,
                       outcomes=["spoken"])
        self.robot = robot
        self.sentence = sentence
        self.language = language
        self.personality = personality
        self.voice = voice
        self.mood = mood
        self.block = block
        
    def execute(self, userdata):
        if not isinstance(self.sentence, str) and isinstance(self.sentence, list):
            sentence = random.choice(self.sentence)
        else:
            sentence = self.sentence
        self.robot.speech.speak(sentence, self.language, self.personality, self.voice, self.mood, self.block)
        return "spoken"

@util.deprecated_replace_with("Say_generated")
class Say_userdata_sentence(smach.State):
    def __init__(self, robot): #other option for sentence_selection is 'constructor'
        smach.State.__init__(self,
                       outcomes=["spoken"],
                       input_keys=["sentence"])
        self.robot = robot
        
    def execute(self, userdata):
        self.robot.speech.speak(userdata.sentence)
        return "spoken"
    
class Say_generated(smach.State): 
    def __init__(self, robot, sentence_creator=None, input_keys=[]):
        smach.State.__init__(self,
                       outcomes=["spoken"],
                       input_keys=input_keys)
        self.robot = robot
        self.sentence_creator = sentence_creator
        
    def execute(self, userdata):
        if callable(self.sentence_creator):
            sentence = self.sentence_creator(userdata)
        elif isinstance(self.sentence_creator, str):
            sentence = self.sentence_creator
        else:
            rospy.logerr("Say_lambda could not produce a sentence to say with {0}".format(self.sentence_creator))
        self.robot.speech.speak(sentence)
        return "spoken"

@util.deprecated_replace_with("Ask_yes_no")
class YesNoQuestion(smach.StateMachine):
    def __init__(self, robot, question):
        smach.StateMachine.__init__(self, outcomes=["yes", "no", 'unknown'])
        self.robot = robot

        with self:
            smach.StateMachine.add( "ASK",
                                    Say(robot, question),
                                    transitions={'spoken':"YES_NO_CONCURRENT"})

            def child_term_cb(outcome_map):
                return True #whatever node has exited, just terminate the CC
        
            yesno_cc = smach.Concurrence(   outcomes=['yes', 'no', 'unknown'],
                                            default_outcome='unknown', 
                                            outcome_map={'yes': {'YES'  :'text_heard'},
                                                         'no':  {'NO'   :'text_heard'}}, 
                                            child_termination_cb=lambda outcome: True) #Terminate the 
                
            with yesno_cc:
                smach.Concurrence.add(  "YES",
                                        Await_Text_Ears(robot, "yes"))
                smach.Concurrence.add(  "NO",
                                        Await_Text_Ears(robot, "no"))

                
            smach.StateMachine.add('YES_NO_CONCURRENT',
                                   yesno_cc)

class Ask_yes_no(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["yes", "preempted", "no"])

        self.robot = robot
        self.preempted = False
        self.ask_user_service_get_yes_no = rospy.ServiceProxy('interpreter/ask_user', AskUser)

    def execute(self, userdata=None):

        self.response = self.ask_user_service_get_yes_no("yesno", 3 , rospy.Duration(10))  # 3 tries, each max 10 seconds

        for x in range(0,len(self.response.keys)):
                if self.response.keys[x] == "answer":
                    response_answer = self.response.values[x]

        if response_answer == "true":
            return "yes"
        else:
            return "no"