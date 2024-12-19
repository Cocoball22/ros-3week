#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_front_laser");
  ros::NodeHandle nh;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  ros::Rate rate(10.0);

  while (nh.ok())
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
    // ros::spinOnce();
     rate.sleep();
  
  }
  return 0;
};
