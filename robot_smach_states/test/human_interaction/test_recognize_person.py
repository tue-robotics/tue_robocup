# System
import unittest

# TU/e Robotics
from image_recognition_msgs.msg import CategoryProbability, Recognition

# Robot Smach States
from robot_smach_states.human_interaction.recognize_person import get_operator_name, NoDetections, NoRecognitionMatch

BEST_LABEL = "best"
BEST_PROBABILITY = 6.0
WORST_LABEL = "worst"
WORST_PROBABILITY = 2.0


class TestGetPersonName(unittest.TestCase):

    def test_normal(self):
        best_match = CategoryProbability(BEST_LABEL, BEST_PROBABILITY)
        worst_match = CategoryProbability(WORST_LABEL, WORST_PROBABILITY)

        recognition1 = Recognition()
        recognition1.categorical_distribution.probabilities = [
            best_match,
            worst_match,
        ]
        self.assertEqual(get_operator_name(recognition1), BEST_LABEL)

        recognition2 = Recognition()
        recognition2.categorical_distribution.probabilities = [
            worst_match,
            best_match,
        ]
        self.assertEqual(get_operator_name(recognition2), BEST_LABEL)

    def test_below_threshold(self):
        worst_match = CategoryProbability(WORST_LABEL, WORST_PROBABILITY)
        recognition = Recognition()
        recognition.categorical_distribution.probabilities = [
            worst_match,
        ]
        with self.assertRaises(NoRecognitionMatch) as context:
            get_operator_name(
                recognition, threshold=recognition.categorical_distribution.probabilities[0].probability+0.1
            )
            self.assertIn("threshold" in context.exception)

    def test_empty_recognition(self):
        recognition = Recognition()
        with self.assertRaises(NoDetections) as context:
            get_operator_name(recognition)
            self.assertIn("Recognition does not contain probabilities", context.exception)


if __name__ == "__main__":
    unittest.main()
