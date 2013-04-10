#! /usr/bin/env python

import roslib; roslib.load_manifest('tue_execution_pack')
import rospy
import smach

def buildup(pre, inputs, outputs):
            #missing = dict()
            totals = {0:pre}
            for index, (inp, outp) in enumerate(zip(inputs, outputs)):
                name, input_keys = inp
                missing_input = (set(input_keys)-set(pre+totals[index]))
                yield (name, missing_input)
                
                _, output_keys = outp
                totals[index+1] = list(pre) + list(output_keys)

def check_userdata(container):
        children = container.get_children()
        outputs = [(name, child.get_registered_output_keys()) for name, child in children.iteritems()]
        inputs =  [(name, child.get_registered_input_keys())  for name, child in children.iteritems()]
        for state, misses in buildup(container.userdata.keys(), inputs, outputs):
            if misses:
                rospy.logwarn("{0} misses userdata keys {1}".format(state, list(misses)))
        
        subcontainers = [(name, child) for name, child in children.iteritems() if isinstance(child, smach.Container)]
        for name, sub in subcontainers:
            #rospy.logwarn("--- {0} ---".format(name))
            check_userdata(sub)
            
if __name__ == "__main__":
    print "Call check_userdata(smach.Container) to check if any of its children is missing userdata for input"