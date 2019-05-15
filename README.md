# RoboticsNanodegree

This is a repository containing code written for Udacity's Robotics Software Engineer Nanodegree. It contains projects developed for the Nanodegree. Projects are desinged to be run in a ROS Catkin workspace, and have been tested on ROS Kinetic.

## Project 1 - Build My Wolrd

This project is an introduction to Gazebo, and contains a Gazebo World and a few models designed for that world. Dogbone World is a house with a few rooms and two tables shaped roughly like dog bones.

## Project 2 - Go Chase It!

This project involves working with a robot model inside Gazebo, and developing sensor plugins and nodes for the model. The robot model is a simple two-wheeled robot with a camera asensor and a Hokuyo LIDAR. Launching world.launch inside my_robot creates the Gazebo world from Project 1 and puts the robot model in it. Launching ball_chaser.launch inside ball_chaser starts the two nodes - drive_bot and process_image that allow the robot to move, and to find a white ball in the field of view, respectively. Using both of these nodes, the robot can follow white object around and move towards. it.

## Project 3 - Where Am I?

The third project involves using the robot model's Hokuyo LIDAR to localize the robot in the world. An adaptive Monte Carlo localization (AMCL) algorithm was used, where the data source was the LIDAR's laser scan (/scan) topic. The robot starts out uncertain of its pose, but is able to update its pose estimate as it moves through the world when given a map of its environment.

## Project 4 - Map My World

This project involves simultaneous localization and mapping (SLAM) using an RGBD camera. The Real Time Appearance-Based Mapping package (RTAB-Map) is used to combine odometry measurements, laser scans, and visual feature tracking (via GFTT features) to perform mapping, localization, and loop closure. Here, we simulate the use of a laser scan by taking a single line of the RGBD camera, since our robot does not actually have a laser rangefinder.

For best results, move the robot at slow speeds so the odometry is more accurate, and return to the same visual field several times to allow for loop closure.

## Project 5 - Home Service Robot

In this final project, path planning is combined with localization on a known map to have the robot pick up and drop off virtual objects. For simplicity, a Turtlebot is used here rather than the custom robot used in the other projects (since the Turtlebot is circular and can rotate without translating). AMCL is used for localization, and Dijkstra's algorithm is used on a local cost map of the robot to plan its path given the map.

Two nodes work together to simulate the pickup and dropoff task. Pick_objects sends the two goal poses for the robot to go to, while add_markers simulates the object being transported. The latter places a marker in the first goal pose and tracks the robot's position, removing the marker when the robot reaches the first goal. Once the robot reaches the second goal, the marker appears there, completing the simulated trip.
