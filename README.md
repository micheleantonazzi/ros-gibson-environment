# Ros Gibson Environment

This is a ros package to use [Gibson](https://github.com/StanfordVL/GibsonEnv) 
environment as a simulation tool for robotics applications. 
In particular, this package correctly sets Gibson environment and connect it to ros. In addition, this package implements the navigation stacks for the Turtlebot 2 and the Husky robots.

## Package configuration

Before use *ros_gibson_environment*, please install Gibson, building it from source.
Follow the instruction [here](https://github.com/StanfordVL/GibsonEnv). 
After that, download the datasets containing the environments to virtualize.
You can find all links and instructions [here](https://github.com/StanfordVL/GibsonEnv/blob/master/gibson/data/README.md).
Finally, clone this repo in your catkin workspace and run ```catkin_make```.

## How it works

To run Gibson and attack it to ros, you can use the launch files called ```<robot_name>_gibson_simulator.launch```.
Using this file, it is possible to load any environment of any dataset customizing the parameters related to the nodes
called ```<robot_name>_gibson_simulator.py```.
These parameters are:
* *env_dataset*: the dataset name ('matterport' or 'stanford')
* *environment*: the name of the environment

**NB:** each environment has a particular starting position and orientation. These data are stored in 
```ros_gibson_environment/config/starting_positions.yaml```. I haven't set the correct positions for all environments, so if you use 
an environment without the correct position, please fix it and update the file. 
