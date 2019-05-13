#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {

  // Initialize the pick_objects node (edited from simple_navigation)
  ros::init(argc, argv, "pick_objects");
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  move_base_msgs::MoveBaseGoal goal;
  
  goal.target_pose.header.frame_id = "map";		// frame_id changed from "base_link" to "map"
  goal.target_pose.header.stamp = ros::Time::now();

  // pickup zone
  goal.target_pose.pose.position.x = 3.0;
  goal.target_pose.pose.position.y = -9.0;
  goal.target_pose.pose.orientation.w = 1.0;
  
  ROS_INFO("Sending pickup zone goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot reached the pickup zone!");
  else
    ROS_INFO("The base failed to move.");
    
  // wait at the pickup zone for 5 seconds
  ros::Duration(5.0).sleep();

  // dropoff zone
  goal.target_pose.pose.position.x = 7.0;
  goal.target_pose.pose.position.y = 2.0;
  goal.target_pose.pose.orientation.w = 1.5;
  
  ROS_INFO("Sending dropoff zone goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot reached the dropoff zone!");
  else
    ROS_INFO("The base failed to move.");
    
  ros::Duration(5.0).sleep();
  
  return 0;
}
