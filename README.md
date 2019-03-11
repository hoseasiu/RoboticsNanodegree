# RoboticsNanodegree

This is a repository containing code written for Udacity's Robotics Software Engineer Nanodegree. It contains projects developed for the Nanodegree. Projects are desinged to be run in a ROS Catkin workspace, and have been tested on ROS ____ (TODO - check which version Udacity uses).

## Project 1 - Build My Wolrd

This project is an introduction to Gazebo, and contains a Gazebo World and a few models designed for that world. Dogbone World is a house with a few rooms and two tables shaped roughly like dog bones.

## Project 2 - Go Chase It!

This project involves working with a robot model inside Gazebo, and developing sensor plugins and nodes for the model. The robot model is a simple two-wheeled robot with a camera asensor and a Hokuyo LIDAR. Launching world.launch inside my_robot creates the Gazebo world from Project 1 and puts the robot model in it. Launching ball_chaser.launch inside ball_chaser starts the two nodes - drive_bot and process_image that allow the robot to move, and to find a white ball in the field of view, respectively. Using both of these nodes, the robot can follow white object around and move towards. it.
