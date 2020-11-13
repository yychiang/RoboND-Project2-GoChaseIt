#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving the bot in the specified velocity and direction.");

    // Request specified velocity and direction
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service and pass the specified velocity and direction.
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    
    int height = img.height; // image height, that is, number of rows
    int width = img.width; // image width, that is, number of columns
    int step  = img.step;  // Full row length in bytes
    // The definition of above parameters could be found at 
    // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html


    
    int positionId; // The position ID of each white pixel we find. 
    // This will be stored in  white_position vector.

    // TODO:
    // (1) Loop through each pixel in the image and check if there's a bright white one
    int sum = 0; //The sum of all column index of all white positions
    int size = 0;
    for (int i = 0; i < height * step; i += 3) {
        positionId = i % ( width * 3 ) / 3;
        if ( img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel ) {
            // Insert the positionId into white_position vector:
	    sum += positionId;
	    size++;	
        }         
    }
    // (2) Identify if the average of these pixels falls in the left, mid, or right side of the image
    int avg;
     
    

    if (size == 0){
        // Will request a stop when there's no white ball seen by the camera
        drive_robot(0.0, 0.0);  // This request a stop
    }
    else {
        avg = sum / size;

        // Depending on the white ball position, call the drive_bot function and pass velocities to it
        if (avg <= width / 3){
	    drive_robot(0.0, 0.5);  // This request should drive my_robot left
        }
        else if (avg >= 2 * width / 3){
	    drive_robot(0.0, -0.5); // This request drives my_robot right
        }
        else{
	    drive_robot(0.5, 0.0);  // This request drives my_robot robot forward
        }
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
