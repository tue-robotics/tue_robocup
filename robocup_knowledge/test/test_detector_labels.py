import os
import tempfile
import unittest
from types import SimpleNamespace

from robocup_knowledge import knowledge_loader
from robocup_knowledge.detector_labels import (CATEGORY, DROP, OBJECT, LabelMapError, Target, build_label_map,
                                               coverage, lookup, normalize, read_model_labels, to_yaml)

ENVIRONMENTS = os.path.join(os.path.dirname(knowledge_loader.__file__), "environments")

COMMON = SimpleNamespace(
    objects=[{"name": "coke", "category": "drink", "aliases": ["cola"]},
             {"name": "cup", "category": "container"},
             {"name": "apple", "category": "fruit"}],
    locations=[{"name": "dinner_table", "aliases": ["dining table"]},
               {"name": "salon_table", "aliases": ["dining table"]},
               {"name": "couch_long", "aliases": ["couch"]}],
    category_aliases={"drink": ["bottle", "wine glass"]},
    detector_ignore=["person"])


class TestNormalize(unittest.TestCase):
    def test_case_and_separators(self):
        self.assertEqual("dining_table", normalize("Dining table"))
        self.assertEqual("dining_table", normalize(" dining__table "))
        self.assertEqual("hair_gel", normalize("hair-gel"))
        self.assertEqual("cup", normalize("CUP"))


class TestLabelMap(unittest.TestCase):
    def setUp(self):
        self.map = build_label_map(COMMON)

    def test_objects_by_name_and_alias(self):
        self.assertEqual(Target(OBJECT, "cup"), lookup(self.map, "cup"))
        self.assertEqual(Target(OBJECT, "coke"), lookup(self.map, "Cola"))

    def test_category_by_name_and_alias(self):
        self.assertEqual(Target(CATEGORY, "drink"), lookup(self.map, "bottle"))
        self.assertEqual(Target(CATEGORY, "drink"), lookup(self.map, "wine glass"))
        self.assertEqual(Target(CATEGORY, "fruit"), lookup(self.map, "fruit"))

    def test_furniture_and_ignored_labels_are_dropped(self):
        for label in ("dining table", "couch", "dinner_table", "person"):
            self.assertEqual(Target(DROP), lookup(self.map, label), label)

    def test_unknown_label_has_no_type(self):
        self.assertIsNone(lookup(self.map, "laptop"))

    def test_furniture_may_share_a_label(self):
        # dinner_table and salon_table both have the alias "dining table": both mean drop
        self.assertEqual(Target(DROP), lookup(self.map, "dining_table"))

    def test_a_label_with_two_meanings_is_an_error(self):
        clash = SimpleNamespace(objects=[{"name": "cup", "category": "container"}],
                                category_aliases={"container": ["cup"]})
        with self.assertRaises(LabelMapError):
            build_label_map(clash)
        clash = SimpleNamespace(objects=[{"name": "coke", "category": "drink", "aliases": ["can"]},
                                         {"name": "red_bull", "category": "drink", "aliases": ["can"]}])
        with self.assertRaises(LabelMapError):
            build_label_map(clash)

    def test_object_named_like_its_category_wins(self):
        same = SimpleNamespace(objects=[{"name": "bowl", "category": "bowl"}])
        self.assertEqual(Target(OBJECT, "bowl"), lookup(build_label_map(same), "bowl"))

    def test_environment_without_the_new_fields(self):
        plain = SimpleNamespace(objects=[{"name": "cup", "category": "container"}],
                                locations=[{"name": "dinner_table"}])
        label_map = build_label_map(plain)
        self.assertEqual(Target(OBJECT, "cup"), lookup(label_map, "cup"))
        self.assertIsNone(lookup(label_map, "dining table"))


class TestCoverage(unittest.TestCase):
    def test_rows_and_undetectable_objects(self):
        label_map = build_label_map(COMMON)
        rows, undetectable = coverage(label_map, COMMON, ["person", "bottle", "cup", "laptop"])
        self.assertEqual([("person", Target(DROP)), ("bottle", Target(CATEGORY, "drink")),
                          ("cup", Target(OBJECT, "cup")), ("laptop", None)], rows)
        self.assertEqual(["coke", "apple"], undetectable)


class TestFiles(unittest.TestCase):
    def test_read_names_and_yaml(self):
        with tempfile.TemporaryDirectory() as folder:
            names = os.path.join(folder, "coco.names")
            with open(names, "w") as f:
                f.write("person\ndining table\n\n")
            self.assertEqual(["person", "dining table"], read_model_labels(names))
            yaml = os.path.join(folder, "coco.yaml")
            with open(yaml, "w") as f:
                f.write("path: x\nnames:\n  0: person\n  1: dining table\ndownload: y\n")
            self.assertEqual(["person", "dining table"], read_model_labels(yaml))

    def test_yaml_lists_every_label(self):
        text = to_yaml(build_label_map(COMMON), "test")
        self.assertIn('- {label: "bottle", kind: category, name: "drink"}', text)
        self.assertIn('- {label: "person", kind: drop}', text)
        self.assertIn('- {label: "cola", kind: object, name: "coke"}', text)


class TestEnvironments(unittest.TestCase):
    def test_every_environment_has_an_unambiguous_map(self):
        """The check of the setup days, for every environment with a common.py"""
        old = os.environ.get("ROBOT_ENV")
        failures = []
        try:
            for environment in sorted(os.listdir(ENVIRONMENTS)):
                if not os.path.isfile(os.path.join(ENVIRONMENTS, environment, "common.py")):
                    continue
                os.environ["ROBOT_ENV"] = environment
                try:
                    build_label_map(knowledge_loader.load_knowledge("common"))
                except LabelMapError as e:
                    failures.append("{}: {}".format(environment, e))
        finally:
            if old is None:
                os.environ.pop("ROBOT_ENV", None)
            else:
                os.environ["ROBOT_ENV"] = old
        self.assertEqual([], failures)


if __name__ == "__main__":
    unittest.main()
