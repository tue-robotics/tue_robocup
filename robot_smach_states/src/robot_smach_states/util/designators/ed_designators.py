# System
import inspect
import pprint

# ROS
import rospy

# TU/e Robotics
from robot_skills.util.entity import Entity
from robot_skills.util.kdl_conversions import VectorStamped
from robot_smach_states.util.designators.core import Designator
from robot_smach_states.util.designators.checks import check_resolve_type


__author__ = 'loy'


class EdEntityCollectionDesignator(Designator):
    """
    Resolves to a collection of Ed entities

    >>> from robot_skills.mockbot import Mockbot
    >>> robot = Mockbot()
    >>> entities = EdEntityCollectionDesignator(robot)
    >>> check_resolve_type(entities, [Entity]) #This is more a test for check_resolve_type to be honest :-/
    """

    def __init__(self, robot, type="", center_point=None, radius=0, id="", parse=True, criteriafuncs=None,
                 type_designator=None, center_point_designator=None, id_designator=None, debug=False, name=None):
        """Designates a collection of entities of some type, within a radius of some center_point, with some id,
        that match some given criteria functions.
        @param robot the robot to use for Ed queries
        @param type the type of the entity to resolve to (default: any type)
        @param center_point combined with radius: a sphere to search an entity in
        @param radius combined with center_point: a sphere to search an entity in
        @param id the ID of the object to get info about
        @param parse whether to parse the data string associated with the object model or entity
        @param type_designator same as type but dynamically resolved trhough a designator. Mutually exclusive with type
        @param center_point_designator same as center_point but dynamically resolved through a designator.
                Mutually exclusive with center_point
        @param id_designator same as id but dynamically resolved through a designator. Mutually exclusive with id"""
        super(EdEntityCollectionDesignator, self).__init__(resolve_type=[Entity], name=name)
        self.ed = robot.ed
        if type != "" and type_designator is not None:
            raise TypeError("Specify either type or type_designator, not both")
        if center_point is not None and center_point_designator is not None:
            raise TypeError("Specify either center_point or center_point_designator, not both")
        elif center_point is None and center_point_designator is None:
            center_point = VectorStamped()
        if id != "" and id_designator is not None:
            raise TypeError("Specify either id or id_designator, not both")

        self.type = type
        self.center_point = center_point
        self.radius = radius
        self.id = id
        self.parse = parse
        self.criteriafuncs = criteriafuncs or []

        if type_designator:
            check_resolve_type(type_designator, str)
        self.type_designator = type_designator

        if center_point_designator:
            check_resolve_type(center_point_designator, VectorStamped)
        self.center_point_designator = center_point_designator

        if id_designator:
            check_resolve_type(id_designator, str)
        self.id_designator = id_designator

        self.debug = debug

    def _resolve(self):
        _type = self.type_designator.resolve() if self.type_designator else self.type
        _center_point = self.center_point_designator.resolve() if self.center_point_designator else self.center_point
        _id = self.id_designator.resolve() if self.id_designator else self.id
        _criteria = self.criteriafuncs

        entities = self.ed.get_entities(_type, _center_point, self.radius, _id, self.parse)
        if self.debug:
            import ipdb;
            ipdb.set_trace()
        if entities:
            for criterium in _criteria:
                entities = filter(criterium, entities)
                criterium_code = inspect.getsource(criterium)
                rospy.loginfo("Criterium {0} leaves {1} entities".format(criterium_code, len(entities)))
                rospy.logdebug("Remaining entities: {}".format(pprint.pformat([ent.id for ent in entities])))
            if entities:
                self._current = entities
                return self.current

        rospy.logerr("No entities found in {0}".format(self))
        return None


class EdEntityDesignator(Designator):
    """
    Resolves to an entity from an Ed query
    """

    def __init__(self, robot, type="", center_point=None, radius=0, id="", parse=True, criteriafuncs=None,
                 weight_function=None, type_designator=None, center_point_designator=None, id_designator=None,
                 debug=False, name=None):
        """Designates an entity of some type, within a radius of some center_point, with some id,
        that match some given criteria functions.
        @param robot the robot to use for Ed queries
        @param type the type of the entity to resolve to (default: any type)
        @param center_point combined with radius: a sphere to search an entity in
        @param radius combined with center_point: a sphere to search an entity in
        @param id the ID of the object to get info about
        @param parse whether to parse the data string associated with the object model or entity
        @param criteriafuncs a list of functions that take an entity and return a bool (True if criterium met)
        @param weight_function returns a weight for each entity, the one with the lowest weight will be selected
        (could be a distance calculation)
        @param type_designator same as type but dynamically resolved trhough a designator. Mutually exclusive with type
        @param center_point_designator same as center_point but dynamically resolved trhough a designator.
        Mutually exclusive with center_point
        @param id_designator same as id but dynamically resolved through a designator. Mutually exclusive with id"""
        super(EdEntityDesignator, self).__init__(resolve_type=Entity, name=name)

        assert not type or type_designator is None, "Specify either type or type_designator, not both"
        assert center_point is None or center_point_designator is None, \
            "Specify either center_point or center_point_designator, not both"
        assert not id or id_designator is None, "Specify either id or id_designator, not both"

        self.robot = robot
        self.ed = robot.ed
        self.type = type
        if center_point is None and center_point_designator is None:
            center_point = VectorStamped()
        self.center_point = center_point
        self.radius = radius
        self.id = id
        self.parse = parse
        self.criteriafuncs = criteriafuncs or []
        self.weight_function = weight_function or (lambda entity: 0)

        if type_designator:  # the resolve type of type_designator can be either str or list
            check_resolve_type(type_designator, str, list)
        self.type_designator = type_designator

        if center_point_designator:  # the resolve type of type_designator can be either VectorStamped
            check_resolve_type(center_point_designator, VectorStamped)
        self.center_point_designator = center_point_designator

        if id_designator:  # the resolve type of id_designator must be str
            check_resolve_type(id_designator, str)
        self.id_designator = id_designator

        self.debug = debug

    def lockable(self):
        return LockToId(self.robot, self)

    def _resolve(self):
        if self.debug:
            import ipdb;
            ipdb.set_trace()
        _type = self.type_designator.resolve() if self.type_designator else self.type
        _center_point = self.center_point_designator.resolve() if self.center_point_designator else self.center_point
        _id = self.id_designator.resolve() if self.id_designator else self.id
        _criteria = self.criteriafuncs

        if self.type_designator and not _type:
            rospy.logwarn("type_designator {0} failed to resolve: {1}".format(self.type_designator, _type))
        if self.center_point_designator and not _center_point:
            rospy.logwarn("center_point_designator {0} failed to resolve: {1}".format(self.center_point_designator,
                                                                                      _center_point))
        if self.id_designator and not _id:
            rospy.logwarn("id_designator {0} failed to resolve: {1}".format(self.id_designator, _id))

        if isinstance(_type, list):
            _type = ""  # Do the check not in Ed but in code here
            typechecker = lambda entity: entity.type in _type
            _criteria += [typechecker]

        entities = self.ed.get_entities(_type, _center_point, self.radius, _id, self.parse)
        if entities:
            for criterium in _criteria:
                criterium_code = inspect.getsource(criterium)
                filtered_entities = []
                for entity in entities:
                    try:
                        if criterium(entity):
                            filtered_entities += [entity]
                    except Exception as exp:
                        rospy.logerr("{id} cannot be filtered with criterum '{crit}': {exp}".format(id=entity.id,
                                                                                                    crit=criterium_code,
                                                                                                    exp=exp))

                entities = filtered_entities
                rospy.loginfo("Criterium {0} leaves {1} entities".format(criterium_code, len(entities)))
                rospy.logdebug("Remaining entities: {}".format(pprint.pformat([ent.id for ent in entities])))

            if entities:
                weights = [self.weight_function(entity) for entity in entities]
                names = [entity.id for entity in entities]

                if len(entities) >= 1:
                    rospy.loginfo('choosing best entity from this list (name->weight):\n\t%s', zip(names, weights))
                    return min(entities, key=self.weight_function)
                else:
                    return entities[0]

        rospy.logerr("No entities found in {0}".format(self))
        return None


class EntityByIdDesignator(Designator):
    def __init__(self, robot, id, parse=True, name=None):
        """
        Designate an entity by its ID. Resolves to the entity with that ID

        :param robot: Robot who's worldmodel to use
        :param id: ID of the entity. If no such ID, resolves to None
        :param parse: Whether to parse the Entity's data-field
        :param name: Name of the designator for introspection purposes
        """
        super(EntityByIdDesignator, self).__init__(resolve_type=Entity, name=name)
        self.ed = robot.ed
        self.id_ = id
        self.parse = parse

    def _resolve(self):
        entities = self.ed.get_entities(id=self.id_, parse=self.parse)
        if entities:
            return entities[0]
        else:
            return None

    def __repr__(self):
        return "EntityByIdDesignator(id={}, name={})".format(self.id_, self.name)


class ReasonedEntityDesignator(Designator):
    def __init__(self, robot, query, name=None):
        """
        Designate an entity by its ID. Resolves to the entity with that ID

        :param robot: Robot who's worldmodel and reasoner to use. Robot must have a reasoner
        :param query: query to the reasoner. The first answer is cast to string and used as ID
        :param name: Name of the designator for introspection purposes
        """
        super(ReasonedEntityDesignator, self).__init__(resolve_type=Entity, name=name)
        assert hasattr(robot, "reasoner")
        self.robot = robot
        self.querystring = query

        self._locker = None

    def _resolve(self):
        first_answer = self.robot.reasoner.query_first_answer(self.reasoner_query)
        if not first_answer:
            return None
        rospy.loginfo("first_answer is:", str(first_answer))

        entities = self.ed.get_entities(id=str(first_answer), parse=True)
        if entities:
            return entities[0]
        else:
            return None

    def lockable(self):
        if not self._locker:
            self._locker = LockToId(self.robot, self)
        return self._locker


class LockToId(Designator):
    """An Entity...Designator's resolve() method may return a different Entity everytime.
    For some cases, this may be unwanted because a process has to be done with the same Entity for tha action to be
    consistent.
    In that case, a designator resolving to a different object every time is not usable.
    A LockToId will resolve to the same Entity after a call to .lock() and
    will only resolve to a different Entity after an unlock() call.
    This is done by remembering the Entity's ID"""

    def __init__(self, robot, to_be_locked, name=None):
        """ Constructor

        :param robot: robot object
        :param to_be_locked: designator to be locked
        :param name: (optional) might come in handy for debugging
        """
        super(LockToId, self).__init__(resolve_type=to_be_locked.resolve_type, name=name)
        self.robot = robot
        self.to_be_locked = to_be_locked
        self._locked_to_id = None
        self._locked = False

    def lock(self):
        self._locked = True

    def unlock(self):
        self._locked_to_id = None
        self._locked = False

    def _resolve(self):
        if self._locked:  # If we should resolve to a remembered thing
            if not self._locked_to_id:  # but we haven't remembered anything yet
                entity = self.to_be_locked.resolve()  # Then find  out what we should remember
                if entity:  # If we can find what to remember
                    self._locked_to_id = entity.id  # remember its ID.
            else:  # If we do remember something already, recall that remembered ID:
                return self.robot.ed.get_entity(id=self._locked_to_id)
        else:
            entity = self.to_be_locked.resolve()
            rospy.loginfo("{0} resolved to {1}, but is *not locked* to it".format(self, entity))
            return entity

    def __repr__(self):
        return "LockToId({})._locked = {}".format(self.to_be_locked, self._locked)


if __name__ == "__main__":
    import doctest
    doctest.testmod()
