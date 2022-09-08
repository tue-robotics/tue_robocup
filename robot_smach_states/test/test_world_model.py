import unittest
import PyKDL as kdl
import rospy

from ed_sensor_integration_msgs.srv import UpdateResponse
from ed.entity import Entity
from ed.volume import BoxVolume

from robot_skills import Mockbot
from robot_skills.classification_result import ClassificationResult
from robot_smach_states.world_model import SegmentObjects
import robot_smach_states.util.designators as ds


# noinspection PyAbstractClass
class MockDesignator(ds.EntityByIdDesignator):
    def resolve(self):
        return Entity(
            identifier="foo",
            object_type=None,
            frame_id="map",
            pose=kdl.Frame(),
            shape=None,
            volumes={"on_top_of": BoxVolume(
                kdl.Vector(-1.0, -1.0, -1.0),
                kdl.Vector(1.0, 1.0, 1.0),
            )},
            super_types=[],
            last_update_time=rospy.Time(),
        )


class TestSegmentObjects(unittest.TestCase):
    def setUp(self):
        self.robot = Mockbot()
        self.storage_designator = ds.VariableDesignator([], resolve_type=[ClassificationResult])

    def test_non_writeable(self):
        """
        Check for non-writeable designator
        """
        with self.assertRaises(TypeError):
            SegmentObjects(
                robot=self.robot,
                segmented_entity_ids_designator=self.storage_designator,
                entity_to_inspect_designator=MockDesignator(self.robot, "foo"),
            )

    def test_happy_flow_no_object(self):
        """
        Check happy flow: no objects
        """
        state = SegmentObjects(
            robot=self.robot,
            segmented_entity_ids_designator=self.storage_designator.writeable,
            entity_to_inspect_designator=MockDesignator(self.robot, "foo"),
        )
        self.assertEqual(state.execute(), "done", "SegmentObjects did not return 'done'")

    def test_happy_flow_with_objects(self):
        """
        Check happy flow: return object and check if it has been written
        """
        def _update_kinect(*args, **kwargs):
            return UpdateResponse(new_ids=["foobar"])
        self.robot.ed.update_kinect = _update_kinect

        def _classify(*args, **kwargs):
            return [ClassificationResult("foobar", "bar", 0.5, None)]
        self.robot.ed.classify = _classify

        state = SegmentObjects(
            robot=self.robot,
            segmented_entity_ids_designator=self.storage_designator.writeable,
            entity_to_inspect_designator=MockDesignator(self.robot, "foo"),
        )
        self.assertEqual(state.execute(), "done", "SegmentObjects did not return 'done'")
        self.assertEqual(self.storage_designator._current[0].uuid, "foobar")

    @unittest.skip("test_overwrite: is not (yet) implemented correctly in SegmentObjects")
    def test_overwrite(self):
        """
        Checks if a writeable designator is actually cleared if nothing new is there
        """
        storage_designator = self.storage_designator.writeable
        storage_designator._current = [None]
        state = SegmentObjects(
            robot=self.robot,
            segmented_entity_ids_designator=storage_designator,
            entity_to_inspect_designator=MockDesignator(self.robot, "foo"),
        )
        self.assertEqual(state.execute(), "done", "SegmentObjects did not return 'done'")
        self.assertEqual(
            len(storage_designator._current),
            0,
            "Writeable designator was not written to empty when nothing was detected"
        )


if __name__ == '__main__':
    unittest.main()
