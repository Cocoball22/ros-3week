#ifndef MAKE_POINT_HPP
#define MAKE_POINT_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h> // tf::TransformListener를 생성하는데 필요한 헤더파일 
#include <sensor_msgs/LaserScan.h> // 
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <cmath>

class MakePoint
{
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    tf::Matrix3x3 Rotation; 
    tf::Vector3 Translation;
    bool make_point();
    void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    //bool a = false;
    bool is_success = false;
    
public:
    MakePoint();
        ~MakePoint();
};
#endif

