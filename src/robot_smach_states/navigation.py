#! /usr/bin/env python
import roslib; roslib.load_manifest('robot_smach_states')
import rospy
import smach
import geometry_msgs.msg
import util
import robot_skills.util.msg_constructors as msgs
import util.transformations
import util.reasoning_helpers as urh

import math

from psi import Term, Compound, Conjunction
#from designators.designator import NoAnswerException

import actionlib

#TODO: Refactor these Navigate_*-classes so that get_loc_via reasoner is not a method of _abstract.

class Navigate_abstract(smach.State):
    @util.deprecated_replace_with("NavigateGeneric")
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=["arrived", "unreachable", "preempted", "goal_not_defined"])

        self.robot = robot
        assert hasattr(robot, 'base')

        self.preempted = False
        self.dynamic   = False
    
    def calc_dist(self, (xg,yg,phig)):
            #TODO: Take rotation into account as well
            loc = self.robot.base.location.pose.position
            xr,yr,zr = loc.x, loc.y, loc.z
            dist = math.sqrt(abs(xr-xg)**2 + abs(yr-yg)**2)
            return dist

    def get_goal(self, userdata):
        """Must return a PoseStamped"""
        raise NotImplementedError("Navigate_abstract does not implement get_goal, use a subclass instead.")    

    def execute(self, userdata=None):
        if not self.dynamic:
            target_pose = self.get_goal(userdata)
            if not target_pose:
                rospy.logerr("No goal could be defined in {state} with userdata {ud}".format(state=self, ud=userdata))
                #self.robot.speech.speak("I don't know where to go. I'm very sorry.")
                return "goal_not_defined"
                
            outcome = self.robot.base.send_goal(target_pose, block=True) #TODO Loy: in send_goal, assert that the right (duck)types are passed
            if outcome:
                return "arrived"
            else:
                self.robot.base.reset_costmap()
                rospy.loginfo("Resetting costmap")
                outcome2 = self.robot.base.send_goal(target_pose, block=True) #TODO Loy: in send_goal, assert that the right (duck)types are passed
                if outcome2:
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
                target_pose = self.get_goal(userdata)
                if target_pose:
                    rospy.loginfo("Goal: ({0.x}, {0.y})".format(target_pose.pose.position))
                    pos, orient = target_pose #unpack tuple
                else:
                    rospy.logerr("No goal could be defined in {state} with userdata {ud}".format(state=self, ud=userdata))
                    self.robot.speech.speak("I don't know where to go. I'm very sorry.", block=False)
                    return "goal_not_defined"
    
                target_pose.header.frame_id = "/map"
                outcome = self.robot.base.send_goal(target_pose, time=0.5, block=False)
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

        query = Compound("waypoint", name, Compound("pose_2d", "X", "Y", "Phi"))

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
            
            point = self.robot.base.point(x,y)
            orient = self.robot.base.orient(phi)

            target_pose =  geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=point, 
                                                                                     orientation=orient))
            target_pose.header.frame_id = "/map"
            return target_pose

class Navigate_exact(Navigate_abstract):
    """ Navigate to an exact x,y,phi location in /map.
    Overrides get_goal, the rest is inherited from Navigate_abstract"""
    @util.deprecated_replace_with("NavigateGeneric")
    def __init__(self, robot, x, y, phi):
        Navigate_abstract.__init__(self, robot)
        
        self.robot = robot
        self.point = self.robot.base.point(x,y)
        self.orient = self.robot.base.orient(phi)

        self.preempted = False
        
    def get_goal(self, userdata):
        target_pose =  geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=self.point, 
                                                                                     orientation=self.orient))
        target_pose.header.frame_id = "/map"
        return target_pose 
        
class Navigate_named(Navigate_abstract):
    """Navigate to a location from a list of locations, indexed by name
    The constructor already sets the point and location, so we can use the inherited get_goal unmodified"""
    @util.deprecated_replace_with("NavigateGeneric")
    def __init__(self, robot, name):
        Navigate_abstract.__init__(self, robot)
        
        self.robot = robot
        assert hasattr(robot, 'reasoner')

        self.goal_name = name

    def get_goal(self, userdata):
        return self.get_loc_via_reasoner(self.goal_name)

class Navigate_generated_name(Navigate_abstract):
    """Move to a named location from the locations_list, with the name specified by the callback"""
    @util.deprecated_replace_with("NavigateGeneric")
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

class Navigate_to_queryoutcome(Navigate_abstract):
    """Move to the output of a query, which is passed to this state as a Term from the reasoner-module.
    
    The state can take some parameters that specify which keys of the dictionary to use for which data.
    By default, the binding-key "X" refers to the x-part of the goal, etc. 
    
    Optionally, also a sorter can be given that sorts the bindings according to some measure.
    """
    @util.deprecated_replace_with("NavigateGeneric")
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

            target_pose =  geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=self.robot.base.point(x,y), 
                                                                                     orientation=self.robot.base.orient(phi)))
            target_pose.header.frame_id = "/map"
            return target_pose

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
                        
            target_pose =  geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=self.robot.base.point(x,y), 
                                                                                     orientation=self.robot.base.orient(phi)))
            target_pose.header.frame_id = "/map"
            return target_pose

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
                visited_assertion = Compound("assert", Compound("visited", self.current_identifier))
                self.robot.reasoner.query(visited_assertion)
                self.current_identifier = None

                rospy.loginfo("outcome=arrived, the following fact should be asserted = visited({0})".format(self.current_identifier))

            elif outcome == "unreachable":
                visited_assertion = Compound("assert", Compound("unreachable", self.current_identifier))
                self.robot.reasoner.query(visited_assertion)
                self.current_identifier = None

                rospy.loginfo("outcome=unreachable, the following fact should be asserted = unreachable({0})".format(self.current_identifier))

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
            loc = self.robot.base.location.pose.position
            xr,yr,zr = loc.x, loc.y, loc.z
            dist = math.sqrt(abs(xr-xg)**2 + abs(yr-yg)**2)
            return dist
        
    def get_goal(self, userdata):
        """self.get_goal gets the answer to this query and lets it parse it into a list of binding-dictionaries. """
        
        rospy.loginfo("Trying to find answers for query {query} with identifier = {identifier}".format(query=self.queryTerm, identifier=self.identifier))

        self.decorated_query = Conjunction(self.queryTerm, 
                                Compound("not", Compound("visited",     self.identifier)),
                                Compound("not", Compound("unreachable", self.identifier)))

    
        answers = self.robot.reasoner.query(self.decorated_query)

        if not answers:
            rospy.logwarn("No answers found for query {query} that are not visited and are not unreachable.".format(query=self.decorated_query))
            return None
        else:
            basepos = self.robot.base.location.pose.position
            basepos = (basepos.x, basepos.y, basepos.z)
            selected_answer = urh.select_answer(answers, 
                                                lambda answer: urh.xyz_dist(answer, basepos), 
                                                minmax=min, 
                                                criteria=[lambda answer: urh.xyz_dist(answer, basepos) < self.maxdist])
            x,y,z = urh.answer_to_tuple(selected_answer)
            rospy.loginfo("[navigation.py] NAVIGATING TO (X = {0}, Y = {1}, Z = {2}".format(x,y,z))

            location = selected_answer[self.ROI_Location]

            self.current_identifier = location
            
            self.robot.reasoner.query(Compound("retractall", (Compound("current_poi","POI", Compound("point_3d","X","Y","Z")))))
            self.robot.reasoner.query(Compound("assert", (Compound("current_poi",self.current_identifier, Compound("point_3d",x,y,z)))))
            
            look_point = geometry_msgs.msg.PointStamped()
            look_point.point = self.robot.base.point(x,y)
            pose = util.msg_constructors.Quaternion(z=1.0)

            base_poses_for_point = self.robot.base.get_base_goal_poses(look_point, self.x_offset, self.y_offset)

            if base_poses_for_point:
                base_pose_for_point = base_poses_for_point[0]
            else:
                rospy.logerr("IK returned empty pose.")
                return look_point.point, pose  #outWhen the IK pose is empty, just try to drive to the point itself. Will likely also fail.
            
            if base_pose_for_point.pose.position.x == 0 and base_pose_for_point.pose.position.y ==0:
                rospy.logerr("IK returned empty pose.")
                return look_point.point, pose  #outWhen the IK pose is empty, just try to drive to the point itself. Will likely also fail.

            return base_pose_for_point

class Look_at_obstacle(smach.State):
    @util.deprecated_replace_with("NavigateGeneric")
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
                    self.robot.head.send_goal(obstaclepoint) #get the point of the PointStamped
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

    @util.deprecated_replace_with("NavigateGeneric")
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
    # class used in gpsr 2013
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
                target_pose =  geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=pos, 
                                                                                     orientation=orient))
                target_pose.header.frame_id = "/map"
            except NoAnswerException:
                rospy.logerr("No goal could be defined in state {0} using designator {1}".format(self, self.designator))
                return "goal_not_defined"
            except StopIteration, si:
                rospy.loginfo(si)
                return "all_matches_tried"

            outcome = self.robot.base.send_goal(target_pose, block=True) #TODO Loy: in send_goal, assert that the right (duck)types are passed
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
                        target_pose =  geometry_msgs.msg.PoseStamped(pose=geometry_msgs.msg.Pose(position=pos, 
                                                                                             orientation=orient))
                        target_pose.header.frame_id = "/map"
                except NoAnswerException:
                    rospy.logerr("No goal could be defined in state {0} using designator {1}".format(self, self.designator))
                    return "goal_not_defined"
                except StopIteration, si:
                    rospy.loginfo(si)
                    return "all_matches_tried"

                outcome = self.robot.base.send_goal(target_pose, time=0.5, block=False)
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

class ResetCostmap(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])

        self.robot = robot

    def execute(self):

        self.robot.base.reset_costmap()

        return 'done'

''' New implementation: hierarchical state machine '''
class Determine_goal(smach.State):
    def __init__(self, robot, goal_pose_2d=None, goal_name=None, goal_query=None, lookat_point_3d=None, lookat_query=None, refresh_freq=0):
        smach.State.__init__(self,outcomes=['succeeded','failed','aborted'],
                            input_keys=['goal'],
                            output_keys=['goal'])                    
        #!!!! location should be named: loc_from, loc_to or object_action        

        self.robot = robot
        self.goal_pose_2d = goal_pose_2d
        self.goal_name = goal_name
        self.goal_query = goal_query
        self.lookat_point_3d = lookat_point_3d
        self.lookat_query = lookat_query
        self.refresh_freq = refresh_freq
        rospy.logdebug("Goal name = {0}".format(self.goal_name))
        rospy.logdebug("Goal query = {0}".format(self.goal_query))
        rospy.logdebug("Lookat query = {0}".format(self.lookat_query))

        self.possible_locations_initialized = False
        self.possible_locations = []

    def set_spindle_out_of_laser_reach(self):
        """Reset the spindle when it could be position such that the arms might hang in the laser range. This should be cleared out by s self-filter"""
        if self.robot.spindle.current_position < 0.35: #TODO: Hardcoded threshold, should read from spindle or something
            return self.robot.spindle.send_goal(0.35, waittime=0.5)
        return True

    def execute(self, userdata):
        self.set_spindle_out_of_laser_reach() #Clear arms out of laser reach

        if self.possible_locations_initialized == False or self.refresh_freq > 0:

            self.possible_locations = []

            if self.goal_pose_2d:
                rospy.logwarn("Goalpose2d")
                x, y, phi = self.goal_pose_2d
                self.possible_locations += [(x, y, phi)]

            if self.goal_name:
                query = Compound("waypoint", self.goal_name, Compound("pose_2d", "X", "Y", "Phi"))
                answers = self.robot.reasoner.query(query)  

                if not answers:
                    rospy.logerr("No answers found for query {query}".format(query=query))
                else:
                    rospy.loginfo("Determined possible goals for name {0}: {1}".format(self.goal_name, answers))
                    self.possible_locations += [(float(answer["X"]), float(answer["Y"]), float(answer["Phi"])) for answer in answers]

            if self.goal_query:
                rospy.logwarn("Goalquery")
                # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
                # is a mapping of variable to a constant, like a string or number
                answers = self.robot.reasoner.query(self.goal_query)

                if not answers:
                    rospy.logerr("No answers found for query {query}".format(query=self.goal_query))                                
                else:
                    rospy.loginfo("Determined possible goals for query {0}: {1}".format(self.goal_query, answers))
                    #From the summarized answer, 
                    self.possible_locations += [(   float(answer["X"]), 
                                               float(answer["Y"]), 
                                               float(answer["Phi"])) for answer in answers]

            if self.lookat_query:
                rospy.logwarn("lookat_query")
                '''
                !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                !!! lookat_query IS MODIFIED FOR JOS AND TIM FOR OPEN CHALLENGE !!!
                !!!!!!!!!!!!!! DO NOT CHANGE UNTIL AFTER ROBOCUP 2013 !!!!!!!!!!!!!
                !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                '''
                # Gets result from the reasoner. The result is a list of dictionaries. Each dictionary
                # is a mapping of variable to a constant, like a string or number
                rospy.logdebug("Query = {0}".format(self.lookat_query))
                # answers = self.robot.reasoner.query(self.lookat_query)
                # rospy.logdebug("Query answers = {0}".format(answers))
                # if not answers:
                #     rospy.logerr("No answers found for query {query}".format(query=self.lookat_query))                                
                # else:
                #     rospy.logerr("Answers found for query {query}: {answers}".format(query=self.lookat_query, answers=answers))
                #     #From the summarized answer, 
                #     possible_ROIs = [(   float(answer["X"]), 
                #                           float(answer["Y"]), 
                #                           float(answer["Z"])) for answer in answers]

                #     # Get base poses for every query outcome and put these in possible locations
                #     for ROI in possible_ROIs:
                answers = self.robot.reasoner.query(self.lookat_query)

                look_point = self.robot.base.point(1, 0, stamped=True)
                look_point.point.x = float(answers[0]["X"])
                look_point.point.y = float(answers[0]["Y"])
                look_point.point.z = float(answers[0]["Z"])

                # ToDo: Parameterize offsets
                base_poses_for_point = self.robot.base.get_base_goal_poses(look_point, 0.5, 0.0)

                if base_poses_for_point:
                    for base_goal_pose in base_poses_for_point:
                        # Convert to x, y, phi
                        phi = self.robot.base.phi(base_goal_pose.pose.orientation)#util.transformations.quaternion_to_euler_z(base_goal_pose.pose.orientation)
                        # Add to possible locations
                        self.possible_locations += [(base_goal_pose.pose.position.x,
                                                base_goal_pose.pose.position.y,
                                                phi)]

            if self.lookat_point_3d:
                rospy.logwarn("Lookat_point_3d")
                
                x,y,z = self.lookat_point_3d

                look_point = self.robot.base.point(1,0,stamped=True)
                #rospy.loginfo("look_point = {0}".format(look_point))
                look_point.point.x = x
                look_point.point.y = y
                look_point.point.z = z

                base_poses_for_point = self.robot.base.get_base_goal_poses(look_point, 0.7, 0.0)

                if base_poses_for_point:
                    for base_goal_pose in base_poses_for_point:
                        # Convert to x, y, phi
                        phi = self.robot.base.phi(base_goal_pose.pose.orientation)#util.transformations.quaternion_to_euler_z(base_goal_pose.pose.orientation)
                        # Add to possible locations
                        self.possible_locations += [(base_goal_pose.pose.position.x,
                                                base_goal_pose.pose.position.y,
                                                phi)]

            # set init to true such that the possible locations array won't be filled a second time when this state is called
            self.possible_locations_initialized = True

            # if the list is not filled it failed to add a possible location
            if not self.possible_locations:
                rospy.logerr("No goal could be defined in {state}".format(state=self))
                self.robot.speech.speak("I don't know where to go. I'm very sorry.")
                return 'failed'

        # abort if there are no poses left to go to
        if not self.possible_locations:
            self.possible_locations_initialized = False
            return 'aborted'
        
        #import ipdb; ipdb.set_trace() 

        # get the info of the poses by querying the costmap
        query_result_tuples = self.robot.base.query_costmap([geometry_msgs.msg.Point(x=xyphi[0],y=xyphi[1],z=0.0) for xyphi in self.possible_locations])

        # merge the poses with their queried info
        possible_locations_with_info = [{'x':xyphi[0], 'y':xyphi[1], 'phi':xyphi[2], 'info':point_info} for xyphi, point_info in zip(self.possible_locations, query_result_tuples)]
        
        # filter out unfeasible poses
        if self.robot.base.use_2d:
            possible_locations_with_info = [d for d in possible_locations_with_info if d['info'].cost < 253]
        else:
            possible_locations_with_info = [d for d in possible_locations_with_info if d['info'].cost < 1]

        # if no feasible poses are left, abort immediately
        if not possible_locations_with_info:
            self.possible_locations_initialized = False
            return 'aborted'

        # get the normalized cost factor   
        max_cost = max([d['info'].cost for d in possible_locations_with_info])
        # make sure that we don't divide by zero!
        if max_cost == 0:
            max_cost = 1     

        # let's see if the cost is under or above the normalized cost threshold
        under_cost_threshold = [d for d in possible_locations_with_info if d['info'].cost/max_cost <= 0.2]
        above_cost_threshold = [d for d in possible_locations_with_info if d['info'].cost/max_cost > 0.2]

        def distance(p1x,p1y,p2x,p2y):
            sq1 = (p1x-p2x)**2
            sq2 = (p1y-p2y)**2
            return math.sqrt(sq1 + sq2)
        
        robot_position = self.robot.base.location.pose.position
        robot_x = robot_position.x
        robot_y = robot_position.y
        # sort those poses under the threshold on distance to get the closest pose first
        under_cost_threshold = sorted(under_cost_threshold, key=lambda d: distance(d['x'], d['y'], robot_x, robot_y))
        # the other ones can be sorted on their cost
        #above_cost_threshold = sorted(above_cost_threshold, key=lambda d: d['info'].cost)
        above_cost_threshold = sorted(above_cost_threshold, key=lambda d: distance(d['x'], d['y'], robot_x, robot_y))

        # merge both lists
        self.possible_locations = under_cost_threshold + above_cost_threshold

        # detach the point infos and make it a tuple again
        self.possible_locations = [(d['x'],d['y'],d['phi']) for d in self.possible_locations]

        # execute the best pose
        x,y,phi = self.possible_locations[0]

        userdata.goal = geometry_msgs.msg.Pose()
        userdata.goal.position = self.robot.base.point(x,y)
        userdata.goal.orientation = self.robot.base.orient(phi)
        rospy.loginfo("Determine_goal, goal = {0}".format(userdata.goal).replace("\n", " "))

        self.possible_locations.pop(0)

        return 'succeeded'

class Get_plan(smach.State):
    def __init__(self, robot, goal_area_radius=0.1):
        smach.State.__init__(self,outcomes=['succeeded','unreachable'],
                            input_keys=['goal'])

        self.robot = robot
        self.goal_area_radius = goal_area_radius
        self.last_reset = rospy.Time.now() - rospy.Duration(10.0) #Subtract 10 seconds to enable one clear map in the first ten seconds

    def execute(self, userdata):

        rospy.loginfo("Looking for a path")
        rospy.loginfo("Get_plan, goal = {0}".format(userdata.goal).replace("\n", " "))
        

        target_pose =  geometry_msgs.msg.PoseStamped(pose=userdata.goal)
        target_pose.header.frame_id = "/map"
        self.robot.base.send_goal(target_pose, time=0.5, block=False, goal_area_radius=self.goal_area_radius)

        #rospy.logdebug("Path found = {0}".format(self.robot.base.path))

        # if not self.robot.base.path:
        #     # Clear costmap and try once more
        #     # ToDo: @BasC: is this the way to go?
        #     rospy.logwarn("Clearing costmap around robot")
        #     self.robot.speech.speak("I am going to clear the map around myself")
        #     self.robot.base.clear_costmap(1.0)
        #     rospy.sleep(rospy.Duration(1.0))
        #     rospy.logwarn("Setting unknown space to free around robot")
        #     self.robot.base.free_unknown_space(0.2)
        #     rospy.sleep(rospy.Duration(1.0))
        #     self.robot.base.send_goal(userdata.goal.position, userdata.goal.orientation, time=0.5, block=False, goal_area_radius=self.goal_area_radius)

        # Ultimate fallback: reset entire map
        if (not self.robot.base.path and (rospy.Time.now() - self.last_reset) > rospy.Duration(10.0) ):
            #self.robot.speech.speak("I cannot seem to figure it out, i am going to reset my entire costmap", block=False)
            rospy.loginfo("I cannot seem to figure it out, i am going to reset my entire costmap")
            self.robot.base.reset_costmap()

        if not self.robot.base.path:
            rospy.loginfo("Could not find a path to goal")
            # ToDo: get next pose
            # ToDo: clear with bigger window? reset map?
            #self.robot.speech.speak("I could not find a path to the goal")
            rospy.loginfo("I could not find a path to the goal")
            #self.robot.speech.speak("I am going to the next goal")
            return "unreachable"
        
        # New additions are encouraged!
        sentences = ["Jetst gate lowes","Lets roll","Here we go","Hey ho, lets go","Consider it done","Moving out","On the move"]#,"Yeehah","Vamos","On iee va","Iemer geraade aus"]
        #self.robot.speech.speak(random.choice(sentences), block=False)

        return 'succeeded'

class Execute_path(smach.State):
    def __init__(self, robot, look_at_path_distance=2.7, refresh_freq=0):
        smach.State.__init__(self,outcomes=['arrived','aborted','waiting','refresh', 'preempted'])

        self.robot = robot
        self.look_at_path_distance = look_at_path_distance
        self.previous_poses_to_goal = 0
        self.refresh_freq = refresh_freq

    def execute(self, userdate):

        print "Executing!"

        while not rospy.is_shutdown():
            goal_status = self.robot.base.ac_move_base.get_state()
            rospy.logdebug("GoalStatus = {0}".format(goal_status))

            if goal_status == actionlib.GoalStatus.PREEMPTED or goal_status == actionlib.GoalStatus.RECALLED:
                return 'preempted'

            if goal_status == actionlib.GoalStatus.SUCCEEDED:
                sentences = ["I have arrived","Here i am","I have reached my goal","I have arrived at my destination"]
                #self.robot.speech.speak(random.choice(sentences))
                return 'arrived'

            if goal_status == actionlib.GoalStatus.ABORTED:
                return 'aborted'

            if self.robot.base.replan_timeout != 0.0:
                wait_time = int(self.robot.base.replan_timeout)
                #self.robot.speech.speak("I found a new path but this is much longer, I will wait for another {0} seconds before i will take it".format(wait_time), block=False)
                rospy.loginfo("I found a new path but this is much longer, I will wait for another {0} seconds before i will take it".format(wait_time))
                # Set previous pose to goal to zero to dismiss the "different path" statement
                self.previous_poses_to_goal = 0
                return 'waiting'

            # Significant re-plan (20 pct longer)
            if (float(self.robot.base.poses_to_goal) > (float(self.previous_poses_to_goal) * 1.2) and self.previous_poses_to_goal != 0):
                #self.robot.speech.speak("Lets take a different path", block=False)
                rospy.loginfo("Lets take a different path")
            self.previous_poses_to_goal = self.robot.base.poses_to_goal

            if self.look_at_path_distance > 0:
                robot_pos = self.robot.base.location.pose.position
                # if the nr of poses to goal is not set, there is something wrong
                if self.robot.base.poses_to_goal != -1:
                    lookat_point = None
                    for i in range(len(self.robot.base.path) - self.robot.base.poses_to_goal, len(self.robot.base.path)):                    
                        dx = robot_pos.x - self.robot.base.path[i].pose.position.x
                        dy = robot_pos.y - self.robot.base.path[i].pose.position.y
                        if dx*dx + dy*dy > self.look_at_path_distance*self.look_at_path_distance and not lookat_point:
                            lookat_point = self.robot.base.path[i].pose.position.x, self.robot.base.path[i].pose.position.y

                    if lookat_point:
                        rospy.logdebug("Look at {0}".format(lookat_point))
                        self.robot.head.send_goal(msgs.PointStamped(lookat_point[0], lookat_point[1], 0), keep_tracking=False, timeout=0.0,
                            min_pan=-0.9,max_pan=0.9,min_tilt=0.0,max_tilt=0.8)
                    else:
                        # reset head position to look down
                        #rospy.logwarn("Reset head to look down")
                        self.robot.head.look_down()
                else:
                    rospy.logwarn("nr of poses to goal is not set (equals {0})".format(self.robot.base.poses_to_goal))

            # Wait 0.5 seconds to avoid looping too fast

            if self.refresh_freq > 0:
                rospy.sleep(1 / float(self.refresh_freq))
                return 'refresh'

            rospy.sleep(0.5)

        return 'preempted'

class Waiting_to_execute(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['done','preempted'])

        self.robot = robot
        self.previous_replan_timeout = 0
        self.z_offset_direction = -1
        self.z_offset_timer = 0
        self.timeout = 20

    def execute(self, userdata):

        start_execute = rospy.Time.now() 

        while (self.robot.base.path_blocked and not rospy.is_shutdown()): 
            
            if self.robot.base.ac_move_base.get_state() == actionlib.GoalStatus.PREEMPTED or self.robot.base.ac_move_base.get_state() == actionlib.GoalStatus.RECALLED:
                return 'preempted'

            if (rospy.Time.now() - start_execute) > rospy.Duration(self.timeout):
                return 'done'

            # search_movement: the robot will look at all 8 corners of a cube around the obstacle, and will then look at the obstacle again
            #    cube_size: the size of the cube around the obstacle
            #    step_time: the max amount of time per head movement (1 head movement = look at 1 corner)
            self.robot.head.search_movement(self.robot.base.obstacle_position, cube_size=1.0, step_time=2.0, min_pan=-1.57,max_pan=1.57,min_tilt=0.0,max_tilt=0.8)

            #self.previous_replan_timeout = self.robot.base.replan_timeout
            # Wait 0.5 seconds to avoid looping too fast
            rospy.sleep(rospy.Duration(0.5))

        if self.robot.base.reached_blocked_timeout:
            #self.robot.speech.speak("This is not going to work, i will take the other route", block=True)
            rospy.loginfo("This is not going to work, i will take the other route")
            #self.previous_replan_timeout = 0.0
            return 'done'
        else:
            #self.robot.speech.speak("My original path is clear again so i can take that anyway", block=True)         
            rospy.loginfo("My original path is clear again so i can take that anyway")

        return 'done'

class Recover(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self,outcomes=['new_path_required','new_goal_required'])

        self.robot = robot

    def execute(self, userdata):

        rospy.logwarn("Move base goal aborted, obstacle at {0}".format(self.robot.base.obstacle_position))
        self.robot.head.send_goal(self.robot.base.obstacle_position)

        # if there is no obstacle on the global path while the robot is not able to reach its goal
        # the local planner is probably in unknown space, so clear this
        if (self.robot.base.obstacle_position.point.x == -1 and self.robot.base.obstacle_position.point.y == -1 and self.robot.base.obstacle_position.point.z == -1):
            # size must be large enough to clear unknown space around the robot
            #self.robot.speech.speak("I am not sure if the space in front of me is clear. Watch out, I am going to clear it", block=False)
            rospy.logdebug("Freeing unknown space and clearing costmap around robot")
            self.robot.base.free_unknown_space(2.0)
            self.robot.base.clear_costmap(1.2)
            return 'new_path_required'
        # otherwise the goal is unreachable or base is in obstacle, needs clearing
        else:    
            # Compute distance to determine whether the base is in/near an obstacle or the goal is unreachable
            dx = self.robot.base.base_pose.pose.position.x - self.robot.base.obstacle_position.point.x
            dy = self.robot.base.base_pose.pose.position.y - self.robot.base.obstacle_position.point.y
            distance = math.sqrt(dx*dx+dy*dy)
            rospy.logwarn("Distance = {0}".format(distance))
            if distance < 0.1:
                #self.robot.speech.speak("I have the funny feeling that I am inside an obstacle. I am going to clear the map around myself", block=False)
                rospy.loginfo("I have the funny feeling that I am inside an obstacle. I am going to clear the map around myself")
                self.robot.base.clear_costmap(1.2)
                return 'new_path_required'
            else:
                #self.robot.speech.speak("Oh no, I can not reach my precious goal", block=False)
                rospy.loginfo("Oh no, I can not reach my precious goal")
                return 'new_goal_required'

# refresh_freq      Frequency of re-check of determine_goal (0 means only check at beginning)
class NavigateGeneric(smach.StateMachine):
    def __init__(self, robot, goal_pose_2d=None, goal_name=None, goal_query=None, lookat_point_3d=None, lookat_query=None, look_at_path_distance=1.5, goal_area_radius=0.1, refresh_freq=0):
        smach.StateMachine.__init__(self,outcomes=['arrived','unreachable','preempted','goal_not_defined'])

        self.robot = robot
        self.goal_pose_2d = goal_pose_2d
        self.goal_name = goal_name
        self.goal_query = goal_query
        self.lookat_point_3d = lookat_point_3d
        self.lookat_query = lookat_query
        self.look_at_path_distance = look_at_path_distance
        self.goal_area_radius = goal_area_radius
        self.clearance_window_size = 0.7
        self.refresh_freq = refresh_freq

        self.preempted = False

        self.userdata.goal = None

        assert hasattr(robot, 'base')

        self.determine_goal = Determine_goal(self.robot, 
                goal_pose_2d = self.goal_pose_2d, 
                goal_name = self.goal_name, 
                goal_query = self.goal_query,
                lookat_point_3d = self.lookat_point_3d, 
                lookat_query = self.lookat_query,
                refresh_freq = self.refresh_freq)

        @smach.cb_interface(outcomes=['done'])
        def init_navigate_generic(userdata):
            self.determine_goal.robot = robot
            self.determine_goal.goal_pose_2d = goal_pose_2d
            self.determine_goal.goal_name = goal_name
            self.determine_goal.goal_query = goal_query
            self.determine_goal.lookat_point_3d = lookat_point_3d
            self.determine_goal.lookat_query = lookat_query
            rospy.logdebug("Goal name = {0}".format(self.goal_name))
            rospy.logdebug("Goal query = {0}".format(self.goal_query))
            rospy.logdebug("Lookat query = {0}".format(self.lookat_query))

            self.determine_goal.possible_locations_initialized = False
            self.determine_goal.possible_locations = []
            return "done"

        with self:

            smach.StateMachine.add('INITIALIZE_NAVIGATE_GENERIC', smach.CBState(init_navigate_generic),
                                    transitions={   'done':'DETERMINE_GOAL'})
        
            smach.StateMachine.add('DETERMINE_GOAL', self.determine_goal,
                transitions={'failed'           : 'goal_not_defined',
                             'aborted'          : 'DETERMINE_GOAL_TRY_AGAIN',
                             'succeeded'        : 'GET_PLAN'})

            smach.StateMachine.add('GET_PLAN', Get_plan(self.robot, self.goal_area_radius),
                transitions={'unreachable'      : 'DETERMINE_GOAL',
                             'succeeded'        : 'EXECUTE'})

            smach.StateMachine.add('EXECUTE', Execute_path(self.robot, self.look_at_path_distance, self.refresh_freq),
                transitions={'arrived'          : 'arrived',
                             'preempted'        : 'preempted',
                             'waiting'          : 'WAITING',
                             'refresh'          : 'DETERMINE_GOAL',
                             'aborted'          : 'RECOVER'})

            smach.StateMachine.add('WAITING', Waiting_to_execute(self.robot),
                transitions={'done'             : 'EXECUTE',
                             'preempted'        : 'preempted'})

            smach.StateMachine.add('RECOVER', Recover(self.robot),
                transitions={'new_path_required': 'GET_PLAN',
                             'new_goal_required': 'DETERMINE_GOAL'})

            smach.StateMachine.add('DETERMINE_GOAL_TRY_AGAIN', Determine_goal(self.robot, 
                goal_pose_2d = self.goal_pose_2d, 
                goal_name = self.goal_name, 
                goal_query = self.goal_query,
                lookat_point_3d = self.lookat_point_3d, 
                lookat_query = self.lookat_query),
                transitions={'failed'           : 'goal_not_defined',
                             'aborted'          : 'unreachable',
                             'succeeded'        : 'GET_PLAN_TRY_AGAIN'})

            smach.StateMachine.add('GET_PLAN_TRY_AGAIN', Get_plan(self.robot, self.goal_area_radius),
                transitions={'unreachable'      : 'DETERMINE_GOAL_TRY_AGAIN',
                             'succeeded'        : 'EXECUTE_TRY_AGAIN'})

            smach.StateMachine.add('EXECUTE_TRY_AGAIN', Execute_path(self.robot, self.look_at_path_distance, self.refresh_freq),
                transitions={'arrived'          : 'arrived',
                             'preempted'        : 'preempted',
                             'waiting'          : 'WAITING_TRY_AGAIN',
                             'refresh'          : 'DETERMINE_GOAL_TRY_AGAIN',
                             'aborted'          : 'RECOVER_TRY_AGAIN'})

            smach.StateMachine.add('WAITING_TRY_AGAIN', Waiting_to_execute(self.robot),
                transitions={'done'             : 'EXECUTE_TRY_AGAIN',
                             'preempted'        : 'preempted'})

            smach.StateMachine.add('RECOVER_TRY_AGAIN', Recover(self.robot),
                transitions={'new_path_required': 'GET_PLAN_TRY_AGAIN',
                             'new_goal_required': 'DETERMINE_GOAL_TRY_AGAIN'})
