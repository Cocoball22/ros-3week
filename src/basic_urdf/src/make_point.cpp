#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <cmath>

class make_point
{
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    sensor_msgs::PointCloud cloud;
public:
    make_point()
    {
        scan_sub = nh.subscribe("scan1", 1, &make_point::counterCallback, this);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/cloud",1);
    }   

    void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
            cloud.header.stamp = ros::Time::now();
            cloud.header.frame_id = "front_laser";

            unsigned int num_points = 1440;
            cloud.points.resize(num_points);

            try
            {
                listener.waitForTransform("base_link", "front_laser", ros::Time(0), ros::Duration(3.0));
                listener.lookupTransform("base_link","front_laser",ros::Time(0), transform);
            }
            catch(tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
  	            ros::Duration(1.0).sleep();
            }

            //we'll also add an intensity channel to the cloud
            cloud.channels.resize(1);
            cloud.channels[0].name = "intensities";
            cloud.channels[0].values.resize(num_points);

            //generate some fake data for our point cloud
            for(unsigned int i = 0; i < num_points; ++i)
            {
              float r = msg->ranges[i]; // 현재 거리값
              float angle_min = 0;
              float theta = angle_min + i * msg->angle_increment; // 현재 각도 계산

              cloud.points[i].x = r * cos(theta);
              cloud.points[i].y = r * sin(theta);
              cloud.points[i].z = 0;
              cloud.channels[0].values[i] = msg->intensities[i];
            }

            cloud_pub.publish(cloud);
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "make_point");
  make_point mp;

  ros::spin();
  return 0;
};
