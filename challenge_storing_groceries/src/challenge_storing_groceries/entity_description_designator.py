import robot_smach_states.util.designators as ds


class EntityDescriptionDesignator(ds.Designator):
    """ EntityDescriptionDesignator. Resolves into a sentence to announce what the robot is about to grasp """
    def __init__(self, entity_designator, name=None):
        super(EntityDescriptionDesignator, self).__init__(resolve_type=str, name=name)
        self.entity_designator = entity_designator
        self.known_formats = "I'm trying to grab the {type}"
        self.unknown_formats = "I'm trying to grab this thing"

    def _resolve(self):
        entity = self.entity_designator.resolve()
        if not entity:
            return self.unknown_formats
        typ = entity.type
        if typ:
            sentence = self.known_formats.format(type=typ)
        else:
            sentence = self.unknown_formats
        return sentence
