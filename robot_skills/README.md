# robot_skills

`robot_skills` provide interfaces to all parts of a robot: its base, arms, head, perception, worldmodel, speech system, etc.

A robot is represented via a robot-object which can be used by the [robot_smach_states](https://github.com/tue-robotics/tue_robocup/tree/master/robot_smach_states)

This package contains a [`mockbot`](src/robot_skills/mockbot.py), which expose all the needed interfaces.

## Robot registration

To implement your own robot, create a robot specific skills package. To register your robot add the following to the `package.xml` of your package:
```xml
<export>
  <robot_skills robot="foo_skills.foo.Foo"/>
</export>
```
The value of the robot attribute has to be of the format: `<package_name>.<sub_module>.<class_name>`.

[`Unittests`](src/robot_skills/test_tools/test_robot.py) are included in this package. Implement this in your robot specific
skills package. In case this test succeeds, the robot could be used by (most) smach states from the [`robot_smach_states`](../robot_smach_states) package.  
