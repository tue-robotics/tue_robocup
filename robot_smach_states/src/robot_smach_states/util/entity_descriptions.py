"""
Whether an entity, or another description, fits a description.

A description is what a step of a task says about the object it needs, without naming one entity: {'type': 'coke'}
asks for a coke, {'category': 'drink'} for any drink. HERO picks the entity that fits when it sees what is there.

There are two rules, so that every action decides "does this fit" the same way:

- entity_matches: while the task runs, does a seen entity fit the description? Used by Find.
- description_satisfies: while the task is set up, before anything is seen, does the object that an earlier step
  will hold fit what a later step asks for? Used by pick-up, hand-over and place to decide whether they still need
  to fetch the object.
- person_matches: does a seen person fit the attributes of a description ("the waving person")? Used by FindPeople.

Descriptions are dicts, or objects with the same attributes (the action server's EntityDescription). Names are
compared without regard to case, because the action server lowercases the whole task.
"""


def _get(description, key):
    """The value of a key of a description, as a dict or as an object; None when it has none"""
    if description is None:
        return None
    if isinstance(description, dict):
        value = description.get(key)
    else:
        value = getattr(description, key, None)
    return value or None


def _same(a, b):
    return a is not None and b is not None and str(a).lower() == str(b).lower()


def _category_of(knowledge, object_type):
    try:
        return knowledge.get_object_category(object_type)
    except Exception:  # unknown name, or knowledge without object categories
        return None


def entity_matches(entity_type, description, knowledge):
    """
    Whether an entity of this type fits the description.

    - A type asks for exactly that type: an entity of type 'coke' fits {'type': 'coke'}; an entity of type 'drink'
      (the detector could only tell the category) does not.
    - A category asks for an object of it, or an entity that is only known by the category: entities of type 'coke'
      and of type 'drink' both fit {'category': 'drink'}; one of type 'apple' does not.
    - A description without type or category fits nothing.

    >>> from types import SimpleNamespace
    >>> knowledge = SimpleNamespace(get_object_category={'coke': 'drink', 'apple': 'food'}.get)
    >>> [entity_matches(t, {'category': 'drink'}, knowledge) for t in ('coke', 'drink', 'apple', '')]
    [True, True, False, False]
    >>> [entity_matches(t, {'type': 'coke'}, knowledge) for t in ('coke', 'Coke', 'drink')]
    [True, True, False]

    :param entity_type: the type of the entity in the world model; empty when it has none
    :param description: dict (or object) with 'type' and/or 'category'
    :param knowledge: the 'common' knowledge of the environment (get_object_category)
    :return: True when the entity fits
    """
    wanted_type = _get(description, "type")
    if wanted_type is not None:
        return _same(entity_type, wanted_type)
    wanted_category = _get(description, "category")
    if wanted_category is not None:
        return bool(entity_type) and (_same(entity_type, wanted_category) or
                                      _same(_category_of(knowledge, entity_type), wanted_category))
    return False


def description_satisfies(held, wanted, knowledge):
    """
    Whether an object known by the description 'held' certainly fits the description 'wanted'. Used while a task is
    set up, when it is not yet known which entity an earlier step will pick up, only how it was described.

    - held {'type': 'coke'} fits wanted {'type': 'coke'} and wanted {'category': 'drink'} (coke is a drink);
    - held {'category': 'drink'} fits wanted {'category': 'drink'}, but not wanted {'type': 'coke'}: it may be
      another drink, so the coke still has to be fetched;
    - nothing fits a wanted description without type or category, and an empty held description fits nothing.

    >>> from types import SimpleNamespace
    >>> knowledge = SimpleNamespace(get_object_category={'coke': 'drink', 'apple': 'food'}.get)
    >>> description_satisfies({'type': 'coke'}, {'category': 'drink'}, knowledge)
    True
    >>> description_satisfies({'category': 'drink'}, {'type': 'coke'}, knowledge)
    False
    >>> description_satisfies({'category': 'drink'}, {'category': 'food'}, knowledge)
    False

    :param held: dict (or object) with 'type' and/or 'category' of the object an earlier step provides
    :param wanted: dict (or object) with 'type' and/or 'category' that this step asks for
    :param knowledge: the 'common' knowledge of the environment (get_object_category)
    :return: True when every object fitting 'held' also fits 'wanted'
    """
    held_type = _get(held, "type")
    if held_type is not None:
        return entity_matches(held_type, wanted, knowledge)
    held_category = _get(held, "category")
    wanted_type, wanted_category = _get(wanted, "type"), _get(wanted, "category")
    return held_category is not None and wanted_type is None and _same(held_category, wanted_category)


# ---------------------------------------------------------------------------------------------------------------------
# People
#
# A person is described by attributes with values from the rulebook's lists (GPSR command generator): "the waving
# person" is {'gesture': 'waving'}, "the person in the red shirt who is sitting" is {'clothing_color': 'red',
# 'pose': 'sitting'}. People perception (people_recognition) gives each person tags, a posture and shirt colours; this
# is where those are compared with the attributes.

#: Every attribute of a person, with its allowed values (the planner's schema offers exactly these)
PERSON_ATTRIBUTES = {
    "gesture": ("waving", "raising_left_arm", "raising_right_arm", "pointing_left", "pointing_right"),
    "pose": ("sitting", "standing", "lying"),
    "clothing_color": ("black", "blue", "gray", "orange", "red", "white", "yellow"),
    "garment": ("blouse", "coat", "jacket", "shirt", "sweater", "t_shirt"),
}

#: Attributes that perception cannot see yet: they are accepted, but not checked
UNCHECKABLE_PERSON_ATTRIBUTES = ("garment",)

# Tags of people_recognition per gesture or pose. One camera image cannot tell waving from a raised arm, so waving is
# either arm raised. L and R are the person's own arms; PointingLeft/PointingRight are directions as the person sees it.
_GESTURE_TAGS = {"waving": {"lwaving", "rwaving"}, "raising_left_arm": {"lwaving"},
                 "raising_right_arm": {"rwaving"}, "pointing_left": {"pointingleft"},
                 "pointing_right": {"pointingright"}}
_POSE_TAGS = {"sitting": {"lsitting", "rsitting"}, "lying": {"llaying", "rlaying"}}
_COLOR_NAMES = {"gray": {"gray", "grey"}}


def _lower_set(values):
    return {str(v).lower() for v in (values or [])}


def person_matches(person_properties, attributes):
    """
    Whether a person fits the attributes of a description, as far as perception can tell.

    An attribute perception cannot see (a garment, or a pose when no posture was measured) is not checked: the person
    may fit, and the attribute is returned as unchecked, so that HERO can say what it could not check. A person of whom
    perception told nothing does not fit an attribute that could be checked.

    >>> from types import SimpleNamespace
    >>> waving = SimpleNamespace(tags=['RWaving'], posture='sitting', shirt_colors=['grey', 'blue'])
    >>> person_matches(waving, {'gesture': 'waving', 'clothing_color': 'gray'})
    (True, [])
    >>> person_matches(waving, {'gesture': 'raising_left_arm'})
    (False, [])
    >>> person_matches(waving, {'pose': 'sitting', 'garment': 'jacket'})
    (True, ['garment'])

    :param person_properties: the person_properties of a person entity (ed_python PersonProperties: tags, posture,
        shirt_colors); None when perception gave none
    :param attributes: dict attribute -> value, see PERSON_ATTRIBUTES; None values are ignored
    :return: (fits, unchecked): fits is False as soon as a checked attribute does not fit; unchecked lists the
        attributes that could not be checked
    """
    unchecked = []
    for name, value in (attributes or {}).items():
        if value is None:
            continue
        value = str(value).lower()
        if name in UNCHECKABLE_PERSON_ATTRIBUTES or name not in PERSON_ATTRIBUTES:
            unchecked.append(name)
            continue
        if person_properties is None:  # perception told nothing about this person: do not take them for "the waving one"
            return False, unchecked
        tags = _lower_set(getattr(person_properties, "tags", None))
        if name == "gesture":
            fits = bool(tags & _GESTURE_TAGS.get(value, set()))
        elif name == "pose":
            posture = str(getattr(person_properties, "posture", "") or "").lower()
            if posture:
                fits = posture == value
            elif value in _POSE_TAGS:
                fits = bool(tags & _POSE_TAGS[value])
            else:  # 'standing' needs a measured posture
                unchecked.append(name)
                continue
        else:  # clothing_color
            fits = bool(_lower_set(getattr(person_properties, "shirt_colors", None)) &
                        _COLOR_NAMES.get(value, {value}))
        if not fits:
            return False, unchecked
    return True, unchecked
