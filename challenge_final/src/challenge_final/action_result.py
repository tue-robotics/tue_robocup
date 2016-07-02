#!/usr/bin/python


class ActionResult(object):
    """ Class to store the result of an action
    """

    SUCCEEDED = 1
    FAILED = -1

    def __init__(self, result, message):
        """ Constructor

        :param result: result (ActionResult.SUCCEEDED etc.)
        :param message: human readable message
        """
        self.result = result
        self.message = message

    def __repr__(self):
        return self.message
