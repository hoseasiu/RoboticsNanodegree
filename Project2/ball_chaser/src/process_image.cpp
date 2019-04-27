#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO (done): Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget cmd;
	cmd.request.linear_x = lin_x;
    cmd.request.angular_z = ang_z;

	bool result = client.call(cmd);

    if (!result){
        ROS_ERROR("Error: failed to execute drive_robot command");
	}
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int fully_saturated = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

	float white_center_of_mass = -1;
	float num_white_pixels = 0;

    // Loop through each pixel in the image and check if it's white
    for (int i = 0; i < img.height; i++) {
		for (int j = 0; j < img.step; j++) {
	        if (img.data[i*img.step+j] == fully_saturated &&
		   img.data[i*img.step+j+1] == fully_saturated &&
		   img.data[i*img.step+j+2] == fully_saturated) {
				white_center_of_mass = white_center_of_mass + j - img.step/2.0;
				num_white_pixels = num_white_pixels + 1.0;
	        }
		}
    }

	float turn_gain = -1.0/img.step*(M_PI/2.0);

	white_center_of_mass = white_center_of_mass/num_white_pixels;

	if (num_white_pixels > 0)				// ball found - drive towards it
	{
		ROS_INFO("Ball found at COM = %1.2f", (float)white_center_of_mass);
		drive_robot(0.5,turn_gain*white_center_of_mass);
	}
	else										// no ball found - request stop
	{
		drive_robot(0, 0);
	}
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
