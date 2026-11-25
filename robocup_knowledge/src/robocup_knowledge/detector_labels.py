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

The map depends only on the object list, not on the model, so the same map works for every model; ``coverage``
tells which labels of a given model match what.
"""
import re

OBJECT = "object"
CATEGORY = "category"
DROP = "drop"

_SEPARATORS = re.compile(r"[\s_\-]+")


class LabelMapError(Exception):
    """The object list gives one label two different meanings"""


class Target(object):
    """What a label means: ``kind`` is OBJECT, CATEGORY or DROP; ``name`` is the object or category name"""

    def __init__(self, kind, name=""):
        self.kind = kind
        self.name = name if kind != DROP else ""

    def __eq__(self, other):
        return isinstance(other, Target) and (self.kind, self.name) == (other.kind, other.name)

    def __hash__(self):
        return hash((self.kind, self.name))

    def __repr__(self):
        return "{}({})".format(self.kind, self.name) if self.name else self.kind


def normalize(label):
    """
    The form in which labels and names are compared: lower case, with spaces, '-' and '_' as one '_'.
    The C++ side (ed_sensor_integration, label_map.cpp) does the same.

    >>> normalize(" Dining  Table ")
    'dining_table'
    """
    return _SEPARATORS.sub("_", label.strip().lower()).strip("_")


def _categories(common):
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
    """Yield (label, Target, where) for every name and alias of the object list"""
    object_names = set()
    for o in getattr(common, "objects", []):
        object_names.add(normalize(o["name"]))
        for label in [o["name"]] + list(o.get("aliases", [])):
            yield label, Target(OBJECT, o["name"]), "object '{}'".format(o["name"])
    for category in _categories(common):
        # A category named like an object ("bowl" in category "bowl") does not claim that name: the label means the
        # object, which is found through its category as well
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
    :param common: the 'common' knowledge of an environment (module or object with the fields above)
    :return: dict normalized label -> Target
    :raises LabelMapError: when a label would mean two different things, e.g. an object and a category. Several
        pieces of furniture sharing a label is fine: they all mean drop.
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
    :return: the Target of a detector label; None when it matches nothing (an object without a type)
    """
    return label_map.get(normalize(label))


def coverage(label_map, common, model_labels):
    """
    What a model can tell with this object list.

    :param model_labels: the labels of the model, in class order
    :return: (rows, undetectable) with rows a list of (label, Target or None) per model label, and undetectable the
        objects no label of the model names (they can at most be found through their category)
    """
    rows = [(label, lookup(label_map, label)) for label in model_labels]
    named = {target.name for _, target in rows if target is not None and target.kind == OBJECT}
    undetectable = [o["name"] for o in getattr(common, "objects", []) if o["name"] not in named]
    return rows, undetectable


def read_model_labels(path):
    """
    :param path: a class-name file of a detector: one name per line (coco.names), or a YAML file with a 'names:'
        section as '0: name' or '- name' lines (coco.yaml), like yolo_onnx_ros reads them
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
    :return: the label map as the YAML ED's rgbd plugin reads: a list of {label, kind, name}, sorted by label
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
