class Volume(object):
    """ Represents an area of an entity

    """
    def __init__(self, name):
        """ Constructor

        :param name: name of this volume
        """
        self.name = name


class BoxVolume(Volume):
    """ Represents a box shaped volume """
    # ToDo
    pass


class NearVolume(Volume):
    """ Represents a box shaped volume """
    # ToDo
    pass


class CompoundVolume(Volume):
    """ Represents a compound volume """
    # ToDo
    pass


def volumes_from_entity_info_data(s):
    """ Creates a dict mapping strings to Volumes from the EntityInfo data string

    :param s: ed.msg.EntityInfo.data (string)
    :return: dict mapping strings to Volumes
    """
    # ToDo
    return {}
