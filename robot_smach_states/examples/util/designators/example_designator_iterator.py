import rospy

from robot_smach_states.designator_iterator import IterateDesignator
import robot_smach_states.util.designators as ds

if __name__ == "__main__":
    collection = ['a', 'b', 'c']

    collection_des = ds.Designator(collection)
    element_des = ds.VariableDesignator(resolve_type=str)

    iterator = IterateDesignator(collection_des, element_des.writeable)

    for i in range(8):
        rospy.loginfo("iterator.execute() \n {}".format(iterator.execute()))
        rospy.loginfo("element_des.resolve() \n {} \n".format(element_des.resolve()))
