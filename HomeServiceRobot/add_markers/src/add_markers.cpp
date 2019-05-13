#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

double pickupPosX = 3.0;
double pickupPosY = -9.0;
double dropoffPosX = 7.0;
double dropoffPosY = 2.0;
bool pickedUp = false;
bool droppedOff = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  double robotX = msg->pose.pose.position.x;
  double robotY = msg->pose.pose.position.y;
  
  if(!pickedUp){
		double distanceToPickup = sqrt((robotX - pickupPosX)*(robotX - pickupPosX) + (robotY - pickupPosY)*(robotY - pickupPosY));

		if (distanceToPickup < 0.3) {
		  pickedUp = true;
		}
  }
  
  else{
    double distanceToDropoff = sqrt((robotX - dropoffPosX)*(robotX - dropoffPosX) + (robotY - dropoffPosY)*(robotY - dropoffPosY));  
    if (distanceToDropoff < 0.3) {
      droppedOff = true;
    }
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);

  visualization_msgs::Marker marker;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "cube";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pickupPosX;
  marker.pose.position.y = pickupPosY;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  while (ros::ok())
  {
    // Publish the marker
    //while (marker_pub.getNumSubscribers() < 1)
    //{
    //  if (!ros::ok())
    //  {
    //    return 0;
    //  }
    //  ROS_WARN_ONCE("Please create a subscriber to the marker");
    //  sleep(1);
    //}
    
    if (!pickedUp)
    {
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
    }
    else if (!droppedOff)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
    }
    else
    {
      marker.pose.position.x = dropoffPosX;
      marker.pose.position.y = dropoffPosY;
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
    }
    
    ros::spinOnce();
  }
}
