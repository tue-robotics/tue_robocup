"""
Translate the labels of an object detector into the names of the object list of an environment.

A detector has its own words for things: COCO says "bottle" and "dining table", the object list says "coke" and
"dinner_table". The object list (``environments/<env>/common.py``) is the truth; every label a detector can give is
translated once, where it enters the world model, into one of:

- an object of the list ("cup"),
- a category of the list ("bottle" -> "drink": some drink, not known which),
- drop: not an object to keep (a person, or the furniture the objects stand on).

A label that matches nothing stays an object without a type.

The labels are matched against the names in ``common.py`` and these optional fields:

- ``objects[i]['aliases']``: other names of the object ("cola" for "coke").
- ``locations[i]['aliases']``: labels of the furniture ("dining table" for "dinner_table"); furniture is dropped.
- ``category_aliases``: ``{category: [labels]}`` for labels that only tell the category ("bottle" for "drink").
- ``detector_ignore``: other labels to drop ("person").

For example, with this object list::

    objects = [{'name': 'coke', 'category': 'drink', 'aliases': ['cola']},
               {'name': 'cup', 'category': 'container'}]
    locations = [{'name': 'dinner_table', 'room': 'dining_room', 'aliases': ['dining table']}]
    category_aliases = {'drink': ['bottle']}
    detector_ignore = ['person']

the labels of a detector mean:

    ============== =================== ==================================================
    detector label meaning             why
    ============== =================== ==================================================
    "cup"          object cup          an object name
    "cola"         object coke         an alias of coke
    "bottle"       category drink      a category alias: some drink, not known which
    "drink"        category drink      a category name
    "dining table" drop                an alias of the furniture dinner_table
    "person"       drop                in detector_ignore
    "laptop"       (none)              matches nothing: an object without a type
    ============== =================== ==================================================

The map depends only on the object list, not on the model (e.g. YOLO), so the same map works for every model;
``coverage`` tells which labels of a given model match what.
"""
import re

OBJECT = "object"
CATEGORY = "category"
DROP = "drop"

_SEPARATORS = re.compile(r"[\s_\-]+")


class LabelMapError(Exception):
    """The object list gives one label two different meanings, e.g. "can" is an alias of both coke and red_bull"""


class Target(object):
    """
    What a detector label means.

    :ivar kind: OBJECT, CATEGORY or DROP
    :ivar name: the object or category name of the object list; empty for DROP

    >>> Target(CATEGORY, "drink")
    category(drink)
    >>> Target(DROP, "ignored")   # a drop has no name
    drop
    """

    def __init__(self, kind, name=""):
        self.kind = kind
        self.name = name if kind != DROP else ""

    def __eq__(self, other):
        """Two targets are equal when they mean the same: same kind and same name"""
        return isinstance(other, Target) and (self.kind, self.name) == (other.kind, other.name)

    def __hash__(self):
        """Hash of (kind, name), consistent with __eq__"""
        return hash((self.kind, self.name))

    def __repr__(self):
        """'kind(name)', or only the kind when there is no name: shown by the check command"""
        return "{}({})".format(self.kind, self.name) if self.name else self.kind


def normalize(label):
    """
    The form in which labels and names are compared: lower case, with every run of spaces, '-' and '_' as one '_', and
    no separators at the start or the end. COCO writes "dining table", the object list writes "dining_table"; both
    become "dining_table". The C++ side (ed_sensor_integration, src/kinect/labels.cpp, normalizeLabel) does the same.

    >>> normalize(" Dining  Table ")
    'dining_table'
    >>> normalize("hair-gel")
    'hair_gel'
    """
    return _SEPARATORS.sub("_", label.strip().lower()).strip("_")


def _categories(common):
    """
    The categories of the object list, in order of first appearance: the categories of the objects, then the
    categories that only appear in ``category_aliases``.

    >>> from types import SimpleNamespace
    >>> _categories(SimpleNamespace(objects=[{'name': 'coke', 'category': 'drink'},
    ...                                      {'name': 'apple', 'category': 'food'},
    ...                                      {'name': 'water', 'category': 'drink'}],
    ...                             category_aliases={'fabric': ['towel']}))
    ['drink', 'food', 'fabric']
    """
    categories = []
    for o in getattr(common, "objects", []):
        category = o.get("category")
        if category and category not in categories:
            categories.append(category)
    for category in getattr(common, "category_aliases", {}):
        if category not in categories:
            categories.append(category)
    return categories


def _entries(common):
    """
    Every label the object list defines, with what it means and where it comes from (for error messages).

    One entry per object name and object alias, per category name and category alias, per location name and location
    alias, and per detector_ignore label. Old environments list locations as plain names; those are accepted too.

    A category named like one of its objects ("bowl" in category "bowl", in rgo2015) does not claim that name: the
    label means the object, which is found through its category as well.

    :return: generator of (label, Target, where), e.g. ("cola", object(coke), "object 'coke'")
    """
    object_names = set()
    for o in getattr(common, "objects", []):
        object_names.add(normalize(o["name"]))
        for label in [o["name"]] + list(o.get("aliases", [])):
            yield label, Target(OBJECT, o["name"]), "object '{}'".format(o["name"])
    for category in _categories(common):
        names = [] if normalize(category) in object_names else [category]
        for label in names + list(getattr(common, "category_aliases", {}).get(category, [])):
            yield label, Target(CATEGORY, category), "category '{}'".format(category)
    for location in getattr(common, "locations", []):
        if isinstance(location, str):  # old environments list only the names
            location = {"name": location}
        for label in [location["name"]] + list(location.get("aliases", [])):
            yield label, Target(DROP), "location '{}'".format(location["name"])
    for label in getattr(common, "detector_ignore", []):
        yield label, Target(DROP), "detector_ignore"


def build_label_map(common):
    """
    The meaning of every label the object list defines, as a dict from normalized label to Target. This is the map
    the check command prints and ED's rgbd plugin loads.

    A label that would mean two different things is an error, because the robot would not know what it sees: an alias
    "can" on both coke and red_bull, or "cup" as an object name and as a category alias. Two pieces of furniture with
    the same label are fine, because both mean drop: COCO calls every table "dining table".

    >>> from types import SimpleNamespace
    >>> common = SimpleNamespace(objects=[{'name': 'cup', 'category': 'container'}],
    ...                          locations=[{'name': 'dinner_table', 'aliases': ['dining table']},
    ...                                     {'name': 'salon_table', 'aliases': ['dining table']}],
    ...                          category_aliases={'drink': ['bottle']})
    >>> sorted(build_label_map(common).items())
    [('bottle', category(drink)), ('container', category(container)), ('cup', object(cup)), ('dining_table', drop), ('dinner_table', drop), ('drink', category(drink)), ('salon_table', drop)]

    :param common: the 'common' knowledge of an environment (module or object with the fields of the module docstring)
    :return: dict normalized label -> Target
    :raises LabelMapError: listing every label with two different meanings
    """
    label_map, origin, errors = {}, {}, []
    for label, target, where in _entries(common):
        key = normalize(label)
        if not key:
            continue
        if key in label_map and label_map[key] != target:
            errors.append("'{}' is {} ({}) and {} ({})".format(key, label_map[key], origin[key], target, where))
            continue
        label_map.setdefault(key, target)
        origin.setdefault(key, where)
    if errors:
        raise LabelMapError("Ambiguous labels in the object list:\n  " + "\n  ".join(errors))
    return label_map


def lookup(label_map, label):
    """
    What one detector label means. The label is normalized first, so the spelling of the model does not matter.

    >>> label_map = {'dining_table': Target(DROP), 'bottle': Target(CATEGORY, 'drink')}
    >>> lookup(label_map, 'Dining Table')
    drop
    >>> lookup(label_map, 'laptop') is None
    True

    :return: the Target of the label; None when it matches nothing (an object without a type)
    """
    return label_map.get(normalize(label))


def coverage(label_map, common, model_labels):
    """
    What a model can tell with this object list: the meaning of each of its labels, and the objects it cannot name.
    The check command prints this, so that the aliases can be fixed during the setup days.

    >>> from types import SimpleNamespace
    >>> common = SimpleNamespace(objects=[{'name': 'cup', 'category': 'container'},
    ...                                   {'name': 'coke', 'category': 'drink'}],
    ...                          category_aliases={'drink': ['bottle']}, detector_ignore=['person'])
    >>> coverage(build_label_map(common), common, ['person', 'bottle', 'cup', 'laptop'])
    ([('person', drop), ('bottle', category(drink)), ('cup', object(cup)), ('laptop', None)], ['coke'])

    Here coke is undetectable by name: a detection of it can only be a "bottle", so it can be found as a drink but
    never as "the coke".

    :param model_labels: the labels of the model, in class order
    :return: (rows, undetectable) with rows a list of (label, Target or None) per model label, and undetectable the
        objects that no label of the model names
    """
    rows = [(label, lookup(label_map, label)) for label in model_labels]
    named = {target.name for _, target in rows if target is not None and target.kind == OBJECT}
    undetectable = [o["name"] for o in getattr(common, "objects", []) if o["name"] not in named]
    return rows, undetectable


def read_model_labels(path):
    """
    The labels of a model, read from its class-name file, the same files yolo_onnx_ros reads:

    - ``coco.names``: one label per line, in class order::

          person
          bicycle

    - ``coco.yaml`` (any file ending in .yaml or .yml): the 'names:' section, as '<index>: <label>' or '- <label>'
      lines; the section ends at the next top-level key::

          names:
            0: person
            1: bicycle
          download: ...

    Both examples give ['person', 'bicycle'].

    :param path: the class-name file
    :return: the labels in class order
    """
    with open(path) as f:
        lines = f.read().splitlines()
    if not str(path).endswith((".yaml", ".yml")):
        return [line.strip() for line in lines if line.strip()]

    labels, in_names = [], False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("names:"):
            in_names = True
            continue
        if not in_names or not stripped or stripped.startswith("#"):
            continue
        if not line[0].isspace() and not stripped.startswith("-"):
            break  # next top-level key
        if stripped.startswith("-"):
            labels.append(stripped[1:].strip().strip("'\""))
        elif ":" in stripped:
            labels.append(stripped.split(":", 1)[1].strip().strip("'\""))
    return labels


def to_yaml(label_map, environment):
    """
    The label map as the YAML file ED's rgbd plugin reads (ed_sensor_integration, LabelMap::load): a list of
    {label, kind, name}, sorted by label, with name left out for a drop. For example::

        environment: impuls
        labels:
          - {label: "bottle", kind: category, name: "drink"}
          - {label: "cup", kind: object, name: "cup"}
          - {label: "dining_table", kind: drop}

    :param label_map: dict normalized label -> Target, from build_label_map
    :param environment: name of the environment, written in the file so that it shows where the map comes from
    :return: the text of the file
    """
    lines = ["# Generated by robocup_knowledge detector_label_map from environment '{}'. Do not edit: change the"
             .format(environment),
             "# object list (environments/{}/common.py) instead.".format(environment),
             "environment: {}".format(environment),
             "labels:"]
    for key in sorted(label_map):
        target = label_map[key]
        entry = "  - {{label: \"{}\", kind: {}".format(key, target.kind)
        if target.name:
            entry += ", name: \"{}\"".format(target.name)
        lines.append(entry + "}")
    return "\n".join(lines) + "\n"
