# Robot_Path_Planning
[![Build Status](https://travis-ci.org/aracelis-git/Robot_Path_Planning.svg?branch=master)](https://travis-ci.org/aracelis-git/Robot_Path_Planning)

---

## Overview

This ROS package provides a way for a robot to navigate a room by itself through self training. Using Q-learning, the robot navigates the room given a set reward and a learning rate constant. 

## SIP

<a href="https://drive.google.com/open?id=1vPMsvtIHx467KGZdfskSs1heT60qLzy87K0tlMnckY4">SIP Logs</a>

<a href="https://drive.google.com/open?id=1AiFwYDx-4b8577_lUeq3aE7R47sUl6ZOjDB4g3GRvhU "> Sprint Log </a>

## License

Copyright 2017 Aldrin I. Racelis

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

## Dependencies

Turtlebot
Gazebo
Reinforcement Learning

This package has been tested with Ubuntu 14.04 and ROS Indigo.

## Issues or Bugs

Travis is not working correctly with the ROS testing. 

## Build

After cloning the repository, do the following in the root directory: 
	
	catkin_make
	roslaunch robot_path_planning robot_path.launch
	
## Tests

Unit tests made with gtest verified the code. To run the test, execute the following code in the root directory.

	catkin_make run_tests_robot_path_planning
	
The unit tests are integrated with Travis, as seen by the badge at the top of the readme.

## API 
	
