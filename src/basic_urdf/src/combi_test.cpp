#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>


void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   ROS_INFO("LaserScan_min (val,angle = (%f,%f)", msg->range_min, msg->angle_min);
   ROS_INFO("LaserScan_max (val,angle = (%f,%f)", msg->range_max, msg->angle_max);
   ROS_INFO("LaserScan (angle_incre = %f)", msg->angle_increment);
//     msg->time_increment);
//    ROS_INFO("LaserScan (intensities) = %f", msg->intensities);
   
//     for (size_t i = 0; i < msg->intensities.size(); i++) 
//     { // msg->intensities.size()를 사용
//         ROS_INFO("LaserScan (intensities[%zu]) = %f", i, msg->intensities[i]);
//     }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "combi_test");
  ros::NodeHandle nh;
  ros::Subscriber scan_sub = nh.subscribe("scan1",1,counterCallback);

  tf::TransformListener listener;
  tf::StampedTransform transform;

  ros::Rate rate(10.0);

  while (ros::ok())
  {
    try
        {
  	      listener.waitForTransform("base_link", "front_laser", ros::Time(0), ros::Duration(3.0));
          listener.lookupTransform("base_link","front_laser",ros::Time(0), transform);

         // try 블록 안에서 transform을 성공적으로 받은 후
          ROS_INFO("Transform: x: %f, y: %f, z: %f\n\r",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
          ROS_INFO("Transform_rotation: x: %f, y: %f, z: %f\n\r",transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z());
        }

    catch (tf::TransformException ex) 
        {
  	      ROS_ERROR("%s",ex.what());
  	      ros::Duration(1.0).sleep();
        }
    ros::spinOnce();
    rate.sleep();
  
  }
  return 0;
};
