#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   ROS_INFO("LaserScan (val,angle) = (%f,%f", msg->range_min, msg->angle_min);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh; 
  ros::Subscriber scan_sub = nh.subscribe("/scan1",1,counterCallback);
  ros::spin();
  return 0; 
}
