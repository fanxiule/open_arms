# Open Arms Project

**THIS PROJECT IS STILL UNDER DEVELOPMENT**

## Introduction

The Open Arms Project thrives to develop an open source low-cost educational robotic arm with 7 degrees of freedom. Members of the project are all from the University of Waterloo and this project was created as their fourth-year design project for their Mechanical Engineering program. 

## About the Package

This repository is the top-level package for the control system of the Open Arms Project. It contains the following sub-packages:

- open_arms_common: contains common values and constants used by other sub-packages
  
- open_arms_description: URDF of the robot model. URDF was used to generate the moveit configiguration

- open_arms_driver: contains ROS nodes that convert information from user inputs and ROS to information received by Arduino

- open_arms_firmware: Arduino libraries and sketches to control the robot

- open_arms_interface: users can use this package to interact with the robot

- open_arms_moveit_config: a moveit configuration package based on the MoveIt Motion Planning Framework

Refer to the README files under each sub-package for more details.

## Using the Package

All of the packages were developed on a machine with Ubuntu 16.04 and ROS Kinetic.

## Acknowledgement

The team received thremendous supports from Professor Baris Fidan, Dr. Brandon DeHart and Alexander Werner from the University of Waterloo, as well as RoboHub, a research facility from the University of Waterloo.

## Permission

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.