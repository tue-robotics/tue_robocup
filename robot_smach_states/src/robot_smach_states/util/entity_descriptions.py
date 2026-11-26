"""
Whether an entity, or another description, fits a description.

A description is what a step of a task says about the object it needs, without naming one entity: {'type': 'coke'}
asks for a coke, {'category': 'drink'} for any drink. HERO picks the entity that fits when it sees what is there.

There are two rules, so that every action decides "does this fit" the same way:

- entity_matches: while the task runs, does a seen entity fit the description? Used by Find.
- description_satisfies: while the task is set up, before anything is seen, does the object that an earlier step
  will hold fit what a later step asks for? Used by pick-up, hand-over and place to decide whether they still need
  to fetch the object.

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
