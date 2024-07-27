<h1 align="center">Obstacle Avoidance Using VFH</h1>
<h6 align="center">Spring-2023 Robotics Course Final Project at Amirkabir University of Tech.</h6>


## Introduction
This project implements the VFH (Vector Field Histogram) algorithm using the ROS framework for a TurtleBot3 Waffle. The original paper of the VFH algorithm can be found [here](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/borenstein_VFHisto.pdf).\
In this project, the heading and the position of the robot are received from the `/odom` topic and a P-controller is adopted for the robot's movement. The surronding of the robot is scanned by a 360-degree laser scanner and this data is used to compute the polar obstacle density for each 5-degree sector. The resulting histogram is smoothen using the adjucent sectors values and then cut by a threshold. After that, valleys narrower than a certain width are discarded and a binary histogram (only containing 0 and 1 values) marking the obstacles and valleys is generated. Finally, the robot determines its best heading by considering both its goal position and any nearby valleys, ensuring both progress toward the goal and obstacle avoidance. 

## Demo
Here you can observe the robot operating in a maze map. A total of 4 goals are defined for the robot to follow.
<p align="center">
  <img src="https://github.com/NegarMov/Obstacle-Avoidance-Using-VFH/blob/master/video/VFH.gif" alt="Plotted Signals" width="500"/>
</p>
