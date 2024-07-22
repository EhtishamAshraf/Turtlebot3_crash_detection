/*
1.    This client node sends requests to stop the robot once the robot gets a bit close to the pillar.

2.    When the robot is within the threshold distance from the pillar the speed is set to "0" and
      immediately requests the server to stop the robot.
*/


#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <sensor_msgs/LaserScan.h>

float smallest_dist = 0.0;
bool pillar_near = false;

// laser scan callback function:
void Callback(const sensor_msgs::LaserScan::ConstPtr& message)
{
// The below lines compute the smallest distance to the Pillar:
  const std::vector<float>& ranges = message->ranges;
  float smallest_dist = *std::min_element(ranges.begin(), ranges.end());

  ROS_INFO("************ !!!!!!!!!! *****************");

  if (smallest_dist<0.5)
  {
    ROS_INFO("Robot is closed to the pillar - danger zone, stop navigating!");
    pillar_near = true;
  }
  else
  {
    ROS_INFO("Robot is in the safe zone - keep navigating!");
    pillar_near = false;
  }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_emergency_stop_client");
    ros::NodeHandle nh;  

    // Create a service client
    ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("robot_start_stop");

    ros::Subscriber subscriber_ = nh.subscribe("scan", 1000, Callback);

    // Create a service message
    std_srvs::SetBool srv;
    srv.request.data = true;  // Set request data as needed

    bool count = true;  

    // Loop to automatically request server to stop the robot
    ros::Rate loop_rate(10); 
    while (ros::ok()) {
        if (pillar_near) {
            // Call the service
            if (client.call(srv))
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
