# Ros Gibson Environment

This is a ros package to use [Gibson](http://gibsonenv.stanford.edu/) environment as a simulation tool for robotics applications. 
In particular, this package correctly sets Gibson environment and connect it to ros. In addition, it implements the navigation stacks for Turtlebot 2 and Husky robots.

## Package configuration

To use this package, please follow the instructions below:

* Install ROS Noetic on your system

* Install all Turtlebot 2 and Husky packages. To do this, clone in your catkin workspace the following repositories and the run ```catkin_make```

  ```bash
  git clone https://github.com/turtlebot/turtlebot.git
  git clone https://github.com/turtlebot/turtlebot_apps.git
  git clone https://github.com/turtlebot/turtlebot_msgs.git
  git clone https://github.com/yujinrobot/kobuki.git
  git clone https://github.com/yujinrobot/kobuki_msgs.git
  git clone https://github.com/husky/husky.git
  ```
  
* Download and install Gibson environment from source. Please use the forked version contained in this [repository](https://github.com/micheleantonazzi/GibsonEnv). In this updated version of Gibson, other funtionalities are implemented and the most common build issues are fixed.

  **NB:** it is not necessary to configure the *gibson-ros* module contained in Gibson's repository 
  
  
* The simulated Turtlebot 2 model has been modified, increasing the Kinect height. To do this inside Gibson, open the file ```GibsonEnv/gibson/assets/models/turtlebot/turtlebot.urdf``` and search the following tags.

  ```xml
  <joint name="camera_rgb_joint" type="fixed">    
  	<origin rpy="0 0 0" xyz="-0.087 -0.0125 0.287"/>
      <parent link="base_link"/>
      <child link="camera_rgb_frame"/>
  </joint>
  ```

* Also the Husky model has been modified. To increase the camera height, you have to open the Hisky's urdf file ```GibsonEnv/gibson/assets/models/husky.urdf```, find the following lines

  ```xml
  <joint name="eyes_joint" type="fixed">
      <!-- camera is located at front bar -->
      <!-- Default orientation (rpy="0 0 0"): looking at -z with y as up -->
      <origin rpy="1.57 0 -1.57" xyz="0.4 0 0.149"/>
      <parent link="base_link"/>
      <child link="eyes"/>
  </joint>
  ```

  and change 0.149 to 1.00.

* Download the datasets containing the environments to virtualize using Gibson. You can find all links and instructions [here](https://github.com/micheleantonazzi/GibsonEnv/blob/master/gibson/data/README.md).

* Finally, clone this repo in your catkin workspace and run ```catkin_make```.

## How it works

To run Gibson and attack it to ros, you can use the launch files called ```<robot_name>_gibson_simulator.launch```. Using this file, it is possible to load any environment of any dataset. To do this, you simply assign the world's name to the *environment* parameter related to the nodes called ```<robot_name>_gibson_simulator.py```. These nodes starts the Gibson simulation and conncet it to ros. In addition, they automatically show the semantic information if the selected environment has them. The urdf of the simulated robots are contained inside ```robot_models``` folder. These urdf file must be equal than those used by Gibson, otherwise modify one fo them.


**NB:** each environment has a particular starting position and orientation. These data are stored in 
```ros_gibson_environment/config/starting_positions.yaml```. I haven't set the correct positions for all environments, so if you use 
an environment without the correct position, please fix it and update the file. 
