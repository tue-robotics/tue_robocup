__author__ = 'loy'

from collections import namedtuple

# Generates a class with id, type and probability.
ClassificationResult = namedtuple("ClassificationResult", "uuid etype probability distribution")
