/*
1.    This client node sends requests to stop the robot once the robot crashes into the pillar.

2.    When the robot hits the pillar the acceleration drastically increases, and hence the client 
      immediately requests the server to stop the robot.
*/

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <sensor_msgs/Imu.h>

float linear_acceleration_magnitude = 0.0;
float crash_threshold = 1.0;
bool robot_crash_detection = false;

// imu callback function:
void imuCallback(const sensor_msgs::Imu::ConstPtr& message)
{ 
//   ROS_INFO("************ !!!!!!!!!! *****************");

  linear_acceleration_magnitude = sqrt(message->linear_acceleration.x * message->linear_acceleration.x +
                                    message->linear_acceleration.y * message->linear_acceleration.y +
                                    message->linear_acceleration.z * message->linear_acceleration.z);
//   ROS_INFO("the acceleration magnitude is: %.2f", linear_acceleration_magnitude);

//   ROS_INFO("************ !!!!!!!!!! *****************"); 

    // Check if the linear acceleration exceeds a certain threshold
    if (linear_acceleration_magnitude>crash_threshold)
    {
        // ROS_INFO("Robot has crashed into the pillar, stop navigating!");
        robot_crash_detection = true;
    } 
    else 
    {
        // ROS_INFO("Robot is in the safe zone - keep navigating!");
        
        robot_crash_detection = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_crash_stop_client");
    ros::NodeHandle nh;  

    
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("robot_start_stop"); // Service client

    ros::Subscriber subscriber_ = nh.subscribe("imu", 10, imuCallback); // Subscriber for "imu"

    
    std_srvs::SetBool srv; // Create a service message
    srv.request.data = true;  // Set request data as needed

    ros::Rate loop_rate(100); 
    while (ros::ok()) {
        if (robot_crash_detection) {
            

            if (client.call(srv)) // Call the service
            {
                if (srv.response.success)
                {
                    ROS_INFO("Service call successful: %s", srv.response.message.c_str());
                }
                else
                {
                    ROS_ERROR("Service call failed: %s", srv.response.message.c_str());
                }
            }
            else
            {
                ROS_ERROR("Failed to call service");
            }
            
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}