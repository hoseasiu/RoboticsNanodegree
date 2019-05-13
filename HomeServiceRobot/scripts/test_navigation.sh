#!/bin/sh
xterm -e "roslaunch my_robot turtlebot_dogboneHouseWorld.launch" &
sleep 5
xterm -e "roslaunch my_robot amcl_dogboneHouse_map.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"
sleep 5
