#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import geometry_msgs.msg
import util
import util.msg_constructors
import util.reasoning_helpers as urh

import random
import copy
import math

from psi import Term, Compound, Conjunction
#from designators.designator import NoAnswerException

import actionlib

#TODO: Refactor these Navigate_*-classes so that get_loc_via reasoner is not a method of _abstract.
class Navigate_abstract(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["arrived", "unreachable", "preempted", "goal_not_defined"])

        self.robot = robot
    #import ipdb; ipdb.set_trace()
        assert hasattr(robot, 'base')

        self.preempted = False
        self.dynamic   = False
    
    def calc_dist(self, (xg,yg,phig)):
            #TODO: Take rotation into account as well
            loc = self.robot.base.location[0]
            xr,yr,zr = loc.x, loc.y, loc.z
            dist = math.sqrt(abs(xr-xg)**2 + abs(yr-yg)**2)
            return dist

    def get_goal(self, userdata):
        raise NotImplementedError("Navigate_abstract does not implement get_goal, use a subclass instead.")    

    def execute(self, userdata):
        if not self.dynamic:
            goal = self.get_goal(userdata)
            if goal:
                pos, orient = goal #unpack tuple
            else:
                rospy.logerr("No goal could be defined in {state} with userdata {ud}".format(state=self, ud=userdata))
                #self.robot.speech.speak("I don't know where to go. I'm very sorry.")
                return "goal_not_defined"

            outcome = self.robot.base.send_goal(pos, orient, block=True) #TODO Loy: in send_goal, assert that the right (duck)types are passed
            if outcome:
                return "arrived"
            else:
                if not self.preempted:
                    return "unreachable"
                else:
                    self.service_preempt()
                    self.preempted = False
                    return "preempted"
        else:
            rospy.loginfo("Navigating in Dynamic mode")
            unreachable = False
            while not self.preempted and not unreachable:
                #import ipdb; ipdb.set_trace()
                goal = self.get_goal(userdata)
                if goal:
                    rospy.loginfo("Goal: ({0.x}, {0.y})".format(goal[0]))
                    pos, orient = goal #unpack tuple
                else:
                    rospy.logerr("No goal could be defined in {state} with userdata {ud}".format(state=self, ud=userdata))
                    self.robot.speech.speak("I don't know where to go. I'm very sorry.")
                    return "goal_not_defined"

                outcome = self.robot.base.send_goal(pos, orient, time=0.5, block=False)
                if outcome:
                    pass #Keep tracking until told otherwise
                else:
                    if not self.preempted:
                        unreachable = True
                        return 'unreachable'
                    else:
                        self.preempted = False
                        self.service_preempt()
                        return "preempted"
            return "arrived"

    def request_preempt(self):
        """Quit navigating"""
        #self.request_preempt(self)
        rospy.logwarn("Preempting Navigate_abstract...")
        self.preempted = True
        self.robot.base.cancel_goal()

    def get_loc_via_reasoner(self, name):
        rospy.logdebug("Getting location of named goal '{0}'".format(name))

        #import ipdb; ipdb.set_trace()

        query = Compound("base_pose", name, Compound("pose_2d", "X", "Y", "Phi"))

        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(query)

        if not answers:
            rospy.logerr("No answers found for query {query}".format(query=query))
            return None
        else:
            possible_locations = [(float(answer["X"]), float(answer["Y"]), float(answer["Phi"])) for answer in answers]

            x,y,phi = min(possible_locations, key=self.calc_dist)
            rospy.logdebug("Found location for '{0}': {1}".format(name, (x,y,phi)))
            return self.robot.base.point(x,y), self.robot.base.orient(phi)

@util.deprecated
class Navigate_predefined(Navigate_abstract):
    def __init__(self, robot, goal):
        Navigate_abstract.__init__(robot)

        self.goal = goal

    def get_goal(self, userdata):
        return self.goal

class Navigate_exact(Navigate_abstract):
    """ Navigate to an exact x,y,phi location in /map.
    Overrides get_goal, the rest is inherited from Navigate_abstract"""
    def __init__(self, robot, x, y, phi):
        Navigate_abstract.__init__(self, robot)
        
        self.robot = robot
        self.point = self.robot.base.point(x,y)
        self.orient = self.robot.base.orient(phi)

        self.preempted = False
        
    def get_goal(self, userdata):
        return self.point, self.orient
        
class Navigate_named(Navigate_abstract):
    """Navigate to a location from a list of locations, indexed by name
    The constructor already sets the point and location, so we can use the inherited get_goal unmodified"""
    def __init__(self, robot, name):
        Navigate_abstract.__init__(self, robot)
        
        self.robot = robot
        assert hasattr(robot, 'reasoner')

        self.goal_name = name

    def get_goal(self, userdata):
        return self.get_loc_via_reasoner(self.goal_name)

@util.deprecated
class Navigate_generated_name(Navigate_abstract):
    """Move to a named location from the locations_list, with the name specified by the callback"""
    def __init__(self, robot, 
                       callback):
        Navigate_abstract.__init__(self, robot)
        
        self.robot = robot

        assert hasattr(robot, 'reasoner')

        assert callable(callback)
        self.callback = callback
        
    def get_goal(self, userdata):
        name = self.callback(userdata)
        
        return self.get_loc_via_reasoner(name)

@util.deprecated
class Navigate_generated_coords(Navigate_abstract):
    """Move to a named location from the locations_list, with the name specified by the callback"""
    def __init__(self, robot, 
                       callback):
        Navigate_abstract.__init__(self, robot)
        
        self.robot = robot

        assert hasattr(robot, 'reasoner')

        assert callable(callback)
        self.callback = callback
        
    def get_goal(self, userdata):
        goal = self.callback(userdata)
        if goal:
            return goal
        else:
            return None

@util.deprecated
class Navigate_userdata_loc(Navigate_abstract):
    """Move to a named location from the locations_list, with the name specified by userdata.location_name"""
    def __init__(self, robot):
        Navigate_abstract.__init__(self, robot)
        self.register_input_keys(['goallocation_name'])
        
        self.robot = robot
        
    def get_goal(self, userdata):
        name = userdata.goallocation_name
        
        return self.get_loc_via_reasoner(name)

class Navigate_to_queryoutcome(Navigate_abstract):
    """Move to the output of a query, which is passed to this state as a Term from the reasoner-module.
    
    The state can take some parameters that specify which keys of the dictionary to use for which data.
    By default, the binding-key "X" refers to the x-part of the goal, etc. 
    
    Optionally, also a sorter can be given that sorts the bindings according to some measure.
    """
    def __init__(self, robot, query, X="X", Y="Y", Phi="Phi", sorter=None):
        Navigate_abstract.__init__(self, robot)

        assert isinstance(query, Term)

        self.queryTerm = query
        self.X, self.Y, self.Phi = X, Y, Phi
        if not sorter:
            self.sorter = self.calc_dist
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        answers = self.robot.reasoner.query(self.queryTerm)

        if not answers:
            return None
            rospy.logerr("No answers found for query {query}".format(query=self.queryTerm))
        else:
            #From the summarized answer, 
            possible_locations = [( float(answer[self.X]), 
                                    float(answer[self.Y]), 
                                    float(answer[self.Phi])) for answer in answers]

            x,y,phi = min(possible_locations, key=self.sorter)
            rospy.logdebug("Found location for '{0}': {1}".format(self.queryTerm, (x,y,phi)))
            return self.robot.base.point(x,y), self.robot.base.orient(phi)

class Visit_query_outcome(Navigate_to_queryoutcome):
    def __init__(self, robot, query, 
        X="X", Y="Y", Phi="Phi", identifier="Object", 
        sorter=None, dynamic=False):

        Navigate_to_queryoutcome.__init__(self, robot, query, X="X", Y="Y", Phi="Phi", sorter=sorter)
        self.register_outcomes(["all_matches_tried"])

        self.identifier = identifier

        assert isinstance(query, Term)

        self.queryTerm = query
        self.decorated_query = Compound("none")

        self.X, self.Y, self.Phi = X, Y, Phi
        if not sorter:
            self.sorter = self.calc_dist

        self.current_identifier = None
        self.dynamic = dynamic
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """
        
        # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
        # is a mapping of variable to a constant, like a string or number
        #import ipdb; ipdb.set_trace()
        self.decorated_query = Conjunction(self.queryTerm, 
                        Compound("not", Compound("visited",     self.identifier)),
                        Compound("not", Compound("unreachable", self.identifier)))
        answers = self.robot.reasoner.query(self.decorated_query)#self.queryTerm)

        if not answers:
            rospy.logwarn("No answers found for query {query} that are not visited and are not unreachable.".format(query=self.queryTerm))
            return None
        else:
            #From the summarized answer, 
            possible_locations = [(float(answer[self.X]), float(answer[self.Y]), float(answer[self.Phi])) for answer in answers]

            x,y,phi = min(possible_locations, key=self.sorter)
            rospy.logdebug("Found location for '{0}': {1}".format(self.queryTerm, (x,y,phi)))

            location_index = possible_locations.index((x,y,phi))
            selected_answer = answers[location_index]

            answer_identifier = selected_answer[self.identifier]

            self.current_identifier = answer_identifier
            
            return self.robot.base.point(x,y), self.robot.base.orient(phi)

    def execute(self, userdata):
        outcome = super(Visit_query_outcome, self).execute(userdata)
        if outcome == "goal_not_defined":
            #Then check whether this is due to that the plain query has no answers or because all those ar evisited or unreachable?
            plain_matches = len(self.robot.reasoner.query(self.queryTerm))
            checked_matches = len(self.robot.reasoner.query(self.decorated_query))

            if plain_matches != 0 and checked_matches == 0:
                return "all_matches_tried"

        if self.current_identifier:
            if outcome == "arrived":
                visited_assertion = Compound("retractall", Compound("not_visited", self.current_identifier))
                self.robot.reasoner.query(visited_assertion)
                self.current_identifier = None

                rospy.loginfo("outcome=arrived, the following fact should be retracted = not_visited({0})".format(self.current_identifier))

            elif outcome == "failed":
                visited_assertion = Compound("retractall", Compound("not_reachable", self.current_identifier))
                self.robot.reasoner.query(visited_assertion)
                self.current_identifier = None

                rospy.loginfo("outcome=failed, the following fact should be retreacted = not_reachable({0})".format(self.current_identifier))

        else:
            rospy.logerr("current_identifier was None, should not happen.")
        return outcome

class Visit_query_outcome_3d(Visit_query_outcome):
    def __init__(self, robot, query, 
        X="X", Y="Y", Z="Z", identifier="Object", ROI_Location="ROI_Location", 
        sorter=None, dynamic=False,
        x_offset=None, y_offset=None,
        maxdist=20.0):

        # #OLD
        # self.X, self.Y, self.Z = X, Y, Z
        #NEW code: mapping
        mapping = {"X":X, "Y":Y, "Z":Z}
        self.mapping = mapping

        Visit_query_outcome.__init__(self, robot, query, X=mapping["X"], Y=mapping["Y"], sorter=sorter)
        self.register_outcomes(["all_matches_tried"])

        self.identifier = identifier

        assert isinstance(query, Term)

        self.queryTerm = query
        self.decorated_query = Compound("none")

        self.ROI_Location = ROI_Location

        if not sorter:
            self.sorter = self.calc_dist_3d

        self.current_identifier = None
        self.dynamic = dynamic

        self.x_offset, self.y_offset = x_offset, y_offset
        self.maxdist = maxdist

    def calc_dist_3d(self, (xg,yg,zg)):            
            loc = self.robot.base.location[0]
            xr,yr,zr = loc.x, loc.y, loc.z
            dist = math.sqrt(abs(xr-xg)**2 + abs(yr-yg)**2)
            return dist
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """
        
        rospy.loginfo("Trying to find answers for query {query} with identifier = {identifier}".format(query=self.queryTerm, identifier=self.identifier))

        self.decorated_query = Conjunction(self.queryTerm, 
                                Compound("not_visited",     self.identifier),
                                Compound("not_unreachable",   self.identifier))

    
        answers = self.robot.reasoner.query(self.decorated_query)

        if not answers:
            rospy.logwarn("No answers found for query {query} that are not visited and are not unreachable.".format(query=self.decorated_query))
            return None
        else:
            basepos = self.robot.base.location[0]
            basepos = (basepos.x, basepos.y, basepos.z)
            selected_answer = urh.select_answer(answers, 
                                                lambda answer: urh.xyz_dist(answer, basepos), 
                                                minmax=min, 
                                                criteria=[lambda answer: urh.xyz_dist(answer, basepos) < self.maxdist])
            x,y,z = urh.answer_to_tuple(selected_answer)
            location = selected_answer[self.ROI_Location]

            self.current_identifier = location
           
            look_point = geometry_msgs.msg.PointStamped()
            look_point.point = self.robot.base.point(x,y)
            pose = util.msg_constructors.Quaternion(z=1.0)

            base_pose_for_point = self.robot.base.get_base_pose(look_point, self.x_offset, self.y_offset)
            if base_pose_for_point.pose.position.x == 0 and base_pose_for_point.pose.position.y ==0:
                rospy.logerr("IK returned empty pose.")
                return look_point.point, pose  #outWhen the IK pose is empty, just try to drive to the point itself. Will likely also fail.

            return base_pose_for_point.pose.position, base_pose_for_point.pose.orientation

@util.deprecated
class Look_at_obstacle(smach.State):
    def __init__(self, robot, tracking=True, rate=2):
        smach.State.__init__(self, outcomes=["done", "preempted", "no_obstacle"])

        self.robot = robot
        assert hasattr(robot, 'base')

        self.tracking = tracking
        self.preempted = False

        self.rate = rate

    def execute(self, userdata):
        if self.tracking:
            while not self.preempt_requested():
                rospy.Rate(self.rate).sleep()
                obstaclepoint = self.robot.base.obstacle_position #is a PointStamped
                if obstaclepoint:
                    rospy.loginfo("Obstaclepoint"+str(obstaclepoint.point).replace('\n', ' '))
                    self.robot.head.send_goal(obstaclepoint.point) #get the point of the PointStamped
                elif obstaclepoint and obstaclepoint.point.x == -1 and obstaclepoint.point.y == -1 and obstaclepoint.point.z == -1:
                    #reset head
                    self.robot.head.reset_position()
                else:
                    #reset head
                    self.robot.head.reset_position()

            self.preempted = False #reset
            self.robot.head.reset_position()
            self.service_preempt()
            return "preempted" #We can only get at this statement if preempted is True
        else:
            obstaclepoint = self.robot.base.obstacle_position
            if obstaclepoint:
                self.robot.head.send_goal(obstaclepoint)
                return "done"
            else:
                return "no_obstacle"

    # def request_preempt(self):
    #     """Quit looking at object"""
    #     #self.request_preempt(self)
    #     rospy.logwarn("Quit looking at obstacle")
    #     self.preempted = True

class Navigate_goal_location(Navigate_abstract):
    # class used in gpsr 2013

    def __init__(self, robot, location):                    #!!!! location should be named: loc_from, loc_to or object_action
        Navigate_abstract.__init__(self, robot)
        
        self.robot = robot
       
        self.location = location                            #!!!! location should be named: loc_from, loc_to or object_action

    def get_goal(self, userdata):

        raw_goals = self.robot.reasoner.query(Compound("goal", "Action","Loc_from","Loc_to","Object_action","Object_room","Object_location"))
        
        goals = [(answer["Action"], answer["Loc_from"], answer["Loc_to"], answer["Object_action"],answer["Object_room"],answer["Object_location"]) for answer in raw_goals]       
        action,loc_from,loc_to,object_action,object_room,object_location = min(goals)

        action = action.get_string()
        loc_from = loc_from.get_string()
        loc_to = loc_to.get_string()
        object_action = object_action.get_string()

        if self.location == 'loc_from':
            self.object_action = loc_from
            return self.get_loc_via_reasoner(self.object_action)
        elif self.location == 'loc_to':
            self.object_action = loc_to
            return self.get_loc_via_reasoner(self.object_action)
        elif self.location == 'object_action':
            self.object_action = object_action
            return self.get_loc_via_reasoner(self.object_action)


class Navigate_Designator(smach.State):

    def __init__(self, robot, designator, dynamic=False):
        smach.State.__init__(self, outcomes=["arrived", "unreachable", "preempted", "goal_not_defined", "all_matches_tried"])
        
        self.robot = robot
       
        self.designator = designator

        self.dynamic = dynamic

    def execute(self, userdata):
        if not self.dynamic:
            try:
                goal = self.designator.next()
                pos, orient = goal #unpack tuple
            except NoAnswerException:
                rospy.logerr("No goal could be defined in state {0} using designator {1}".format(self, self.designator))
                return "goal_not_defined"
            except StopIteration, si:
                rospy.loginfo(si)
                return "all_matches_tried"

            outcome = self.robot.base.send_goal(pos, orient, block=True) #TODO Loy: in send_goal, assert that the right (duck)types are passed
            if outcome:
                self.designator.finish_current()
                return "arrived"
            else:
                if not self.preempted:
                    self.designator.block_current()
                    return "unreachable"
                else:
                    self.service_preempt()
                    self.preempted = False
                    return "preempted"
        else:
            rospy.loginfo("Navigating in Dynamic mode")
            unreachable = False
            while not self.preempted and not unreachable:
                #import ipdb; ipdb.set_trace()
                try:
                    goal = self.designator.update_current()
                    if goal:
                        rospy.loginfo("Goal: ({0.x}, {0.y})".format(goal[0]))
                        pos, orient = goal #unpack tuple
                except NoAnswerException:
                    rospy.logerr("No goal could be defined in state {0} using designator {1}".format(self, self.designator))
                    return "goal_not_defined"
                except StopIteration, si:
                    rospy.loginfo(si)
                    return "all_matches_tried"

                outcome = self.robot.base.send_goal(pos, orient, time=0.5, block=False)
                if outcome:
                    pass #Keep tracking until told otherwise
                else:
                    if not self.preempted:
                        unreachable = True
                        self.designator.block_current()
                        return 'unreachable'
                    else:
                        self.preempted = False
                        self.service_preempt()
                        return "preempted"
            self.designator.finish_current()
            return "arrived"

    def request_preempt(self):
        """Quit navigating"""
        #self.request_preempt(self)
        rospy.logwarn("Preempting Navigate_abstract...")
        self.preempted = True
        self.robot.base.cancel_goal()

#############################################################################################################
#
#                                        NEW GENERIC STATE (SJOERD)
#
#############################################################################################################

class NavigateGeneric(smach.State):
    def __init__(self, robot, goal_pose_2d=None, goal_name=None, goal_query=None, goal_sorter=None, look_at_path_distance=1.5):
        smach.State.__init__(self, outcomes=["arrived", "unreachable", "preempted", "goal_not_defined"])

        self.robot = robot
        self.goal_pose_2d = goal_pose_2d
        self.goal_name = goal_name
        self.goal_query = goal_query
        self.goal_sorter = goal_sorter
        self.look_at_path_distance = look_at_path_distance
        self.clearance_window_size = 0.6

        self.preempted = False
    
        assert hasattr(robot, 'base')

    def get_goal(self):

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
        #                                 DETERMINE GOAL
        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

        possible_locations = []

        if self.goal_pose_2d:
            x, y, phi = self.goal_pose_2d
            possible_locations += [(x, y, phi)]

        if self.goal_name:
            query = Compound("base_pose", self.goal_name, Compound("pose_2d", "X", "Y", "Phi"))
            answers = self.robot.reasoner.query(query)

            if not answers:
                rospy.logerr("No answers found for query {query}".format(query=query))
            else:
                possible_locations += [(float(answer["X"]), float(answer["Y"]), float(answer["Phi"])) for answer in answers]

        if self.goal_query:
            # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
            # is a mapping of variable to a constant, like a string or number
            answers = self.robot.reasoner.query(self.goal_query)

            if not answers:
                rospy.logerr("No answers found for query {query}".format(query=self.goal_query))                                
            else:
                #From the summarized answer, 
                possible_locations += [(   float(answer["X"]), 
                                           float(answer["Y"]), 
                                           float(answer["Phi"])) for answer in answers]       

        if not possible_locations:
            rospy.logerr("No goal could be defined in {state}".format(state=self))
            self.robot.speech.speak("I don't know where to go. I'm very sorry.")
            return None

        # Get the best possible location according to self.goal_sorter, or get the first one if sorter not specified
        # ToDo: include costmap query
        if not self.goal_sorter == None:
            x,y,phi = min(possible_locations, key=self.goal_sorter)
        else:
            x,y,phi = possible_locations[0]

        return self.robot.base.point(x,y), self.robot.base.orient(phi)

    def execute(self, userdata=None):

        # # # # # # # # # # # # # # # # # START NAVIGATING # # # # # # # # # # # # # # # # 

        # Before navigating, we first have to determine the goal. 
        # This must be checked for feasibility by querying the costmap
        # This way, we know later on that if move_base does not return a feasible path, the path is blocked
        
        # If the query to the costmap is not feasible, the next possible location must be queried.
        # The current goal can be inserted at the back of the queue.

        # If a query is feasible, the path that is returned can be executed. 
        
        # If a path becomes unfeasible during execution the move base interface will give feedback on this
        # 

        self.new_goal_required = True # Auxiliary variable to know if it's a new goal or a replan
        self.new_path_required = True
        previous_poses_to_goal = 0
        goal_status = 0

        # ToDo: timeout needed, rethink anyway
        while ((not self.preempted and not goal_status == actionlib.GoalStatus.SUCCEEDED) or self.new_goal_required):           

			# Significant re-plan
            if (self.robot.base.poses_to_goal > (previous_poses_to_goal + 5) and not previous_poses_to_goal == 0 and self.new_path_required):
                self.robot.speech.speak("Lets take a different path")
            previous_poses_to_goal = self.robot.base.poses_to_goal

            if self.new_goal_required:
                rospy.loginfo("Looking for a goal")
                self.new_goal_required = False
                goal = self.get_goal()
                if goal:
                    rospy.loginfo("Goal: ({0.x}, {0.y})".format(goal[0]))
                    pos, orient = goal #unpack tuple
                else:
                    return "goal_not_defined"

            if self.new_path_required:
                rospy.loginfo("Looking for a path")
                self.new_path_required = False
                self.robot.base.send_goal(pos, orient, time=0.5, block=False)

                if not self.robot.base.path:
                    # Clear costmap and try once more
                    rospy.logwarn("Clearing costmap around robot")
                    self.robot.speech.speak("I am going to clear the map around myself")
                    self.robot.base.clear_costmap(1.0)
                    rospy.sleep(rospy.Duration(1.0))
                    rospy.logwarn("Setting unknown space to free around robot")
                    self.robot.base.free_unknown_space(0.2)
                    rospy.sleep(rospy.Duration(1.0))
                    self.robot.base.send_goal(pos, orient, time=0.5, block=False)

                if not self.robot.base.path:
                    print "Could not find a path to goal"
                    # ToDo: get next pose
                    # ToDo: clear with bigger window? reset map?
                    self.robot.speech.speak("I could not find a path to the goal")
                    #self.robot.speech.speak("I am going to the next goal")
                    return "unreachable"
                
                # New additions are encouraged!
                sentences = ["Jetst gate lowes","Lets roll","Here we go","Hey ho lets go"]
                self.robot.speech.speak(random.choice(sentences))


            # If not preempted and goalstatus active, only keep looking at path
            if goal_status == actionlib.GoalStatus.ACTIVE:

                #import ipdb; ipdb.set_trace()        
                if self.preempted:
                    self.preempted = False
                    self.service_preempt()
                    print 'preempted'
                    return "preempted"

                if self.look_at_path_distance > 0:
                    robot_pos, robot_orient = self.robot.base.get_location()
                    # if the nr of poses to goal is not set, there is something wrong
                    if self.robot.base.poses_to_goal != -1:
                        lookat_point = None
                        for i in range(len(self.robot.base.path) - self.robot.base.poses_to_goal, len(self.robot.base.path)):                    
                            dx = robot_pos.x - self.robot.base.path[i].pose.position.x
                            dy = robot_pos.y - self.robot.base.path[i].pose.position.y
                            if dx*dx + dy*dy > self.look_at_path_distance*self.look_at_path_distance and not lookat_point:
                                lookat_point = self.robot.base.path[i].pose.position.x, self.robot.base.path[i].pose.position.y

                        if lookat_point:
                            rospy.logdebug("look at: " + str(lookat_point))
                            self.robot.head.send_goal(self.robot.head.point(lookat_point[0], lookat_point[1], 0), keep_tracking=True, timeout=0.0)
                        else:
                            self.robot.head.reset_position(timeout=0.0)
                    else:
                        rospy.logwarn("nr of poses to goal is not set (equals {0})".format(self.robot.base.poses_to_goal))


            if goal_status == actionlib.GoalStatus.ABORTED:

                rospy.logwarn("Move base goal aborted, obstacle at {0}".format(self.robot.base.obstacle_position))

                # if there is no obstacle on the global path while the robot is not able to reach its goal
                # the local planner is probably in unknown space, so clear this
                if (self.robot.base.obstacle_position.point.x == -1 and self.robot.base.obstacle_position.point.y == -1 and self.robot.base.obstacle_position.point.z == -1):
                    # size must be large enough to clear unknown space around the robot
                    self.robot.speech.speak("I am not sure if the space in front of me is clear")
                    self.robot.speech.speak("Watch out, I am going to clear it")
                    self.robot.base.free_unknown_space(0.2)
                    # Sleep for a second to ensure the map is cleared and obstacles are inserted again
                    rospy.sleep(rospy.Duration(1.0))
                    self.new_path_required = True
                # otherwise the goal is unreachable or base is in obstacle, needs clearing
                else:    
                    # Compute distance to determine whether the base is in/near an obstacle or the goal is unreachable
                    dx = self.robot.base.base_pose.pose.position.x - self.robot.base.obstacle_position.point.x
                    dy = self.robot.base.base_pose.pose.position.y - self.robot.base.obstacle_position.point.y
                    distance = math.sqrt(dx*dx+dy*dy)
                    rospy.logwarn("Distance = {0}".format(distance))
                    if distance < 0.1:
                        self.robot.speech.speak("I have the funny feeling that I am inside an obstacle")
                        # Size must be at least the diameter of the robot
                        self.robot.speech.speak("I am going to clear the map around myself")
                        self.robot.base.clear_costmap(1.0)
                        # Sleep for a second to ensure the map is cleared and obstacles are inserted again
                        rospy.sleep(rospy.Duration(1.0))
                        self.new_path_required = True
                    else:
                        self.robot.speech.speak("Oh no, I can not reach my precious goal")
                        self.new_goal_required = True
                        self.new_path_required = True
                       
            goal_status = self.robot.base.ac_move_base.get_state()
            rospy.logdebug("GoalStatus = {0}".format(goal_status))
            rospy.sleep(0.5)

        
        rospy.logwarn("new_goal_required = {0}".format(self.new_goal_required))
        rospy.logwarn("Exit status while loop = {0}".format(((not self.preempted and not actionlib.GoalStatus.SUCCEEDED) or self.new_goal_required)))
        rospy.logwarn("not self.preempted = {0}".format(not self.preempted))
        rospy.logwarn("not SUCCEEDED = {0}".format(not actionlib.GoalStatus.SUCCEEDED))
        
        if goal_status == actionlib.GoalStatus.SUCCEEDED:
		    self.robot.speech.speak("I have reached my goal")
        
        self.robot.head.reset_position(timeout=0.0)
        return "arrived"

    def request_preempt(self):
        """Quit navigating"""
        #self.request_preempt(self)
        rospy.logwarn("Preempting NavigateGeneric...")
        self.preempted = True
        self.robot.base.cancel_goal()
