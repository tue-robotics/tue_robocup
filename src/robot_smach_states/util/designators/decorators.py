#!/usr/bin/python

from robot_smach_states.util.designators import *

def functionDesignator(original_function):

    Class = type('Wrapped' + original_function.__name__, (Designator,), {})

    def init(self, *args, **kwargs):
        super(Class, self).__init__()
        self.args = args
        self.kwargs = kwargs

    def resolve(self):
        self._current = original_function(*self.args, **self.kwargs)
        return super(Class, self).resolve()

    Class.__init__ = init
    Class.resolve = resolve
    return Class

if __name__ == '__main__':

    # to test the decorator, let's define a designator with it
    @functionDesignator
    def FormattedSentenceDesignator(fmt, **kwargs):
        kwargs_resolved = {key:value.resolve() for key,value in kwargs.iteritems()}
        return fmt.format(**kwargs_resolved)

    print 'creating the designator'
    item_designator = Designator('random_string')
    d = FormattedSentenceDesignator("This is a {item}.", item=item_designator)
    print 'resolving the designator'
    print d.resolve()
    print d.current
