class Area(object):
    """ Represents an area of an entity

    """
    def __init__(self, name):
        """ Constructor

        :param name: name of this area
        """
        self.name = name


class BoxArea(Area):
    """ Represents a box shaped area """
    # ToDo
    pass


class NearArea(Area):
    """ Represents a box shaped area """
    # ToDo
    pass


class CompoundArea(Area):
    """ Represents a compound area """
    # ToDo
    pass


def areas_from_entity_info_data(s):
    """ Creates a dict mapping strings to Areas from the EntityInfo data string

    :param s: ed.msg.EntityInfo.data (string)
    :return: dict mapping strings to Areas
    """
    # ToDo
    return {}
