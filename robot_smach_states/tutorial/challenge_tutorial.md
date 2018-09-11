# Creating a TU/e RoboCup challenge

This tutorial explains how to setup a new RoboCup challenge. It assumes that you have finished the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) as well as the [Smach tutorials](http://wiki.ros.org/smach/Tutorials). Furthermore, it assumes that the simulator as well as 'middleware' are running. (ToDo: add links.)

## Setting up the boiler plate

### Setting up the package

* Browse to the `tue_robocup` folder on your machine (ToDo: either create it directly in workspace, create link or whole target meuk):
```
cd ~/ros/$ROS_DISTRO/repos/https_/github.com/tue-robotics/tue_robocup.git/
```
* Create the package:
```
catkin_create_pkg challenge_example robot_skills robot_smach_states
```
This creates a ROS package called `challenge_example`. It depends on the `robot_skills`
and the `robot_smach_states` package. This can be seen when inspecting the `package.xml` that is located inside your newly created `challenge_example` folder.
* Make and source your `~/.bashrc` to make sure the package is in your `ROS_PACKAGE_PATH`.:
```
tue-make
source ~/.bashrc
```
and go to your newly created package:
```
roscd challenge_example
```
to see if everything has worked.

### Create the ROS node

* In the `challenge_example` package, create a new `scripts` folder and enter it:
```
mkdir scripts
cd scripts
```
* Create you new node and make it executable:
```
touch challenge_example
chmod +x challenge_example
```
Using `ls` in your terminal you can see that the file is green.
* Open your file in Pycharm and add the following contents:

```python
#!/usr/bin/python

# ROS
import rospy


if __name__ == "__main__":

    rospy.init_node("state_machine_example")
    rospy.loginfo("Hello world")
```

* Now you can run the challenge:
```
rosrun challenge_example challenge_example
```

### Creating a robot api object
* Update your example script to:

```python
#!/usr/bin/python

# ROS
import rospy

# TU/e Robotics
from robot_skills.util.robot_constructor import robot_constructor


if __name__ == "__main__":

    rospy.init_node("state_machine_test")
    rospy.loginfo("Hello world")
    robot = robot_constructor("amigo")
    raw_input("Press any key to exit")

```

* The 'robot' object that is constructed forms the application programming interface (API) to all other ROS nodes of your system. This can be used to move the base, move the arms, and to say stuff. You can use this interactively using the `amigo-console`. In a new terminal, enter
```
amigo-console
```
and an iPython interface will open. Here, enter:
```
amigo.speech.speak("Hello world")
```
and the robot will start talking. By entering `amigo.` + tab you will see all the other robot parts. By entering `amigo.<part>.` + tab you can see the methods that are available in these parts. By entering `amigo.<part>.<method>?` the 'docstring' of this method will be printed. This is useful when you want to try something. Test this yourself by entering:
```
amigo.speech.speak?
```

### Creating your state machine
* Create a new folder for your 'library' in the source directory and add the `__init__.py` and `example.py` files:
```
roscd challenge_example
mkdir src
cd src
mkdir challenge_example
cd challenge_example
touch __init__.py
touch example.py
```
The `__init__.py` indicates that the `src/challenge_example` folder is a Python module.
* Now, we can add the following code in `example.py`:

```python
# ROS
import smach

# Robot smach states
import robot_smach_states as states


class Example(smach.StateMachine):
    """ This is my example state machine

    """
    def __init__(self, robot):
        """ Initialization method

        :param robot: robot api object
        """
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])

        with self:
            smach.StateMachine.add("HELLO_WORLD", states.Say(robot, "Hello world"), transitions={"spoken": "succeeded"})

```
* `import smach` imports the [smach](http://wiki.ros.org/smach) Python module so that we can use it
* `import robot_smach_states as states` imports [the smach states](https://github.com/tue-robotics/tue_robocup/tree/master/robot_smach_states) that have been developed in the TU/e Robotics lab
* `class Example(smach.StateMachine):` defines the statemachine that we are implementing. The `smach.StateMachine` between brackets shows that the `Example` class [inherits](https://en.wikipedia.org/wiki/Inheritance_(object-oriented_programming) from the `smach.StateMachine` class. (If you don't know what inheritance means you might want to study that first)
* `def __init__(self, robot):` defines the signature of the initialization method. Basically, each class needs such a function. In this case, we need to supply the Robot object that was instantiated previously in our script.
* `smach.StateMachine.__init__(self, outcomes=["succeeded", "failed", "aborted"])` calls the initialization method of the parent class. Here, we also define the outcomes of our statemachine to `succeeded`, `failed` and `aborted`.
* `states.Say(robot, "Hello world")` instantiates a `Say` state using our robot object and the sentence "Hello world"
* `smach.StateMachine.add` adds this state to our statemachine and calls it "HELLO_WORLD". Furthermore, it connects the one outcome of the `Say` state, being `spoken` to the `succeeded` outcome of our statemachine.

### Adding our state machine to our app
* Update `scripts/challenge_example` to:

```python
#!/usr/bin/python

# ROS
import rospy

# TU/e Robotics
from robot_skills.util.robot_constructor import robot_constructor

# Challenge example
from challenge_example.example import Example


if __name__ == "__main__":

    rospy.init_node("state_machine_test")
    rospy.loginfo("Hello world")
    robot = robot_constructor("amigo")
    state_machine = Example(robot)
    state_machine.execute()
    raw_input("Press any key to exit")

```
* `from challenge_example.example import Example` imports the `Example` class that we defined in `src/challenge_example/example.py` into this scope.
* `state_machine = Example(robot)` instantiates an Example statemachine using the robot object and calls it 'state_machine'.
* `state_machine.execute()` executes the example state machine.
