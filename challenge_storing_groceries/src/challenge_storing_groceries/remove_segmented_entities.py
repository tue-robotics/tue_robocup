import smach

class RemoveSegmentedEntities(smach.State):
    """ Removes all entities that have no shape (except _root) """
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['done'])
        self.robot = robot

    def execute(self, userdata=None):

        entities = self.robot.ed.get_entities(parse=False)

        for e in entities:
            if not e.has_shape and e.id != '_root':
                self.robot.ed.remove_entity(e.id)

        return "done"

