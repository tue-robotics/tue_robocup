# Creating a TU/e RoboCup challenge

This tutorial explains how to setup a new RoboCup challenge. It assumes that you have finished the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) as well as the [Smach tutorials](http://wiki.ros.org/smach/Tutorials). Furthermore, it assumes that [the simulator as well as 'middleware' are running](https://github.com/tue-robotics/tue-robotics.github.io/wiki/Getting-started).

## Setting up the boiler plate

### Setting up the package

* Browse to your [catkin workspace](http://wiki.ros.org/catkin/workspaces):
```
cd ~/ros/$ROS_DISTRO/system/src
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
* Open your file in [Pycharm](https://www.jetbrains.com/pycharm/) and add the following contents (N.B., you can easily install Pycharm on your system using `tue-get install pycharm`. N.N.B., open Pycharm from a terminal to make sure the [bashrc](https://superuser.com/questions/49289/what-is-the-bashrc-file) is sourced, open the `challenge_example` *folder*, and tick 'Add to currently opened projects' for convenience):

```python
#!/usr/bin/python

# ROS
import rospy


if __name__ == "__main__":

    rospy.init_node("state_machine_example")
    rospy.loginfo("Hello world")
```

* Now you can run the challenge (N.B.: as was mentioned in the introduction, make sure a roscore, simulator and middleware are running):
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

## Creating the state machine
### Create your state machine
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
* `class Example(smach.StateMachine):` defines the statemachine that we are implementing. The `smach.StateMachine` between brackets shows that the `Example` class <a href="https://en.wikipedia.org/wiki/Inheritance_(object-oriented_programming)">inherits</a>  from the `smach.StateMachine` class. (If you don't know what inheritance means you might want to study that first)
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

### Running our challenge
* Before we run our challenge, we need to do some final bookkeeping:
```
roscd challenge_example
touch setup.py
chmod +x setup.py
```
* Add the following contents to `setup.py` (look [here](https://docs.python.org/2/distutils/setupscript.html) for more info on Python setup):

```python
#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # #  don't do this unless you want a globally visible script
    # scripts=['bin/myscript'],
    packages=['challenge_example'],
    package_dir={'': 'src'}
)

setup(**d)
```
* Uncomment (remove the `#`) the line `catkin_python_setup()` in your `CMakeLists.txt`. This makes sure that the `setup.py` is called when `catkin build` or `catkin make` is invoked (see next step).
* run `tue-make challenge_example`. This 'builds' or makes the package (`challenge_example`)
* make sure your roscore, simulator and free mode are running
* run the example:
```
rosrun challenge_example challenge_example
```

### Exercise 1
Now it is time to add a state to the state machine. We want the robot to navigate to the position `x=0.8`, `y=0.27` in the "/map" frame, with rotation `rz=-1.57`. To do so, follow the steps described above and run the script:
* locate the `NavigateToPose` in the `robot_smach_states` package and inspect the API (Application Programming Interface). Specifically look for the arguments that must be provided to the `__init__` method. As you can see, the `NavigateToPose` class inherits from the `NavigateTo` class: to identify the outcomes of this Smach state, inspect the latter class as well.
* Add the `NavigateToPose` state to your state machine
* Reconnect all the transitions (if necessary)
* Run the example
* Add another state so that the robot announces (using speech) that it has arrived at its goal

### Designators
In our applications, we don't want to hardcode the coordinates as we did in Exercise 1. The applications need to be as flexible as possible so that the same code, e.g., to navigate somewhere, can be reused in all applications and all environments. Even during operation, the goal of a navigation action might change. Consider, e.g., the situation that the robot asks the user where it should go. In this case, we don't know the destination of the robot while coding the challenge.

To address this problem, the [designator](https://github.com/tue-robotics/tue_robocup/blob/master/robot_smach_states/src/robot_smach_states/util/designators/core.py) was developed. You can see a designator as a container. At a certain point, either at startup or at some other time during a challenge, something gets put into the container, either by the programmer or by some Smach state. Later on, some other Smach state needs the information that is in the container. Hereto, it calls the `resolve` method and uses the outcome to perform it's task.

There are various types of designators defined in `robot_skills.util.designators` for various purposes. In practice, however, custom designators are often defined to meet challenge-specific needs.

## Exercise 2
* Add the following Smach to your `example.py` (fix the import whenever necessary) that will choose a random furniture object in your environment to navigate to:
```python
class GenerateRandomTarget(smach.State):
    """ Smach state that puts a random target into the provided designator

    """
    def __init__(self, robot, goal_designator):
        """ Initialization method

        :param robot: (Robot) robot api object
        :param goal_designator:
        """
        super(GenerateRandomTarget, self).__init__(outcomes=["done"])
        self._robot = robot
        self._writeable_designator = states.util.designators.core.VariableWriter(goal_designator)

    def execute(self, ud):
        """ Executes the state. Chooses a random target and puts this in the designator

        :param ud: {}
        :return: "done"
        """
        # Get all entities
        entities = self._robot.ed.get_entities()

        # Filter the entity id's (we're only interested in furniture objects) and choose one
        # N.B.: we need to make sure it has an 'in front of' area
        while True:
            target_entity = random.choice([e for e in entities if e.is_a("furniture")])
            if "in_front_of" in target_entity.volumes:
                break

        # Set the current id
        self._writeable_designator.write(target_entity)

        return "done"
```
As is described in the comments, the line `self._writeable_designator.write(target_entity)` makes sure that the entity that has been chosen is 'stored' in the designator.
* To use this, state, instantiate a `VariableDesignator` in your example state machine:
```python
target_designator = states.util.designators.core.VariableDesignator(resolve_type=robot_skills.util.entity.Entity)
```
* Add the `GenerateRandomTarget` state to your state machine. Use the `target_designator` as an argument. Reconnect the transitions
* Add a `NavigateToSymbolic` state to your state machine
```Python
states.NavigateToSymbolic(robot, entity_designator_area_name_map={target_designator: "in_front_of"},
                            entity_lookat_designator=target_designator)
```
(this will make the robot move to the 'in_front_of' area of the designated furniture object while looking at the same furniture object). Reconnect the transitions once again.
* Run the example
