/*
1.  This node subscribers to the "scan topic" to get the closest distance from the pillar.
2.  This node publishes on the "cmd_vel topic" to publish the velocity values to run the robot.
3.  This node also acts as the Server, and it keeps listening to the client's requests,
    and as soon as the client requests to stop the robot, the robot will be stopped.

Note: 1.  We can manually generate the request to stop the robot using the following command:
                  rosservice call /robot_start_stop 'true'

      2. The below command will change the value of the  run_emergency_stop to false in the launch file:
                  roslaunch my_launch_file.launch run_emergency_stop:='false'

*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include <sensor_msgs/LaserScan.h>

geometry_msgs::Twist twist;
bool move = false;
float smallest_dist;
int closest_val_index = -1;

// Check if the client sent a request to stop the robot.
bool startStopCallback(std_srvs::SetBool::Request& req,
                       std_srvs::SetBool::Response& res)
{
  if (req.data) 
  {
    res.message = "Received request to stop the robot";
    move = true;
  } 
  else 
  {
    res.message = "Service is not requested, so robot will keep moving!";
    move = false;
  }
  res.success = true;
  return true;
}

// Get the smallest distance and the index of the pillar.
void Callback(const sensor_msgs::LaserScan::ConstPtr& message)
{
// The below lines compute the smallest distance to the Pillar:
const std::vector<float>& ranges = message->ranges;
smallest_dist = *std::min_element(ranges.begin(), ranges.end());

// This for loop is calculating the index at which we are getting the smallest distance.
// when the difference b/w ranges[i] and smallest distance <= 0, we get that index:

float prev_val = 1000;
for (int i = 0; i < ranges.size(); ++i) {
    if (abs(ranges[i] - smallest_dist) <= prev_val) {
        prev_val = abs(ranges[i] - smallest_dist);
        closest_val_index = i;
    }
}
  // ROS_INFO("Can you print something???????????????????????????????????");
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mrob_highlight_controller_pub_node");

  ros::NodeHandle nodeHandle;

  ros::Subscriber subscriber_ = nodeHandle.subscribe("scan", 1000, Callback);
  ros::Publisher publisher_ = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
  ros::ServiceServer service = nodeHandle.advertiseService("robot_start_stop", startStopCallback);
  
  // twist.linear.x = 0.9;  

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ROS_INFO("The smallest distance is: %.2f", smallest_dist);
    ROS_INFO("The closet index is: %d", closest_val_index);

    if (!move)
      {
      /*We will keep rotating the robot, unless the robot is perfectly aligned with the pillar.
      When laser sensor is between 0 to 5 and 355 to 360, robot is facing the pillar: */

      if (closest_val_index > 10 && closest_val_index < 350)
      { 
        if (closest_val_index > 5 && closest_val_index < 180)
        {
          twist.angular.z = 0.2;                  // Angular velocity about the z-axis (rotation)
          twist.linear.x = 0.0 * smallest_dist;  // Linear velocity in the x-axis (forward direction)
        }
        else
        {
        twist.angular.z = -0.2;                 // Angular velocity about the z-axis (rotation)
        twist.linear.x = 0.0 * smallest_dist;  // Linear velocity in the x-axis (forward direction)
      }
      }
      else
      {
        // ROS_INFO("Linear X: %.2f", twist.linear.x);
        twist.linear.x = 0.9 * smallest_dist;  // Linear velocity in the x-axis (forward direction)
        twist.linear.z = 0.0;
      }
      }
    else
    {
        ROS_INFO("!!!Crash finall detected with success!!!");
        twist.angular.z = 0.0;
        twist.linear.x = 0.0;  
    }
    publisher_.publish(twist);  
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
