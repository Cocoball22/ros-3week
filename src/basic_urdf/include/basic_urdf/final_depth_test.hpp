#ifndef FINAL_DEPTH_TEST_HPP
#define FINAL_DEPTH_TEST_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "opencv2/opencv.hpp"
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>
#include <cmath>

class final_depth_test
{
    private:
        ros::NodeHandle nh;
        ros::Subscriber depth_sub;
        ros::Publisher depth_pub, depth_pub_2;
        tf::StampedTransform tf_depth;
        tf::TransformListener listener;
        tf::Matrix3x3 rotation_depth; 
        tf::Vector3 translation_depth;
        bool is_success = false;
        bool final_depth();
        void camera_callback(const sensor_msgs::PointCloud2::ConstPtr& depth_msg);
        void non_tf(const sensor_msgs::PointCloud2::ConstPtr& msg);
        
    public:
        final_depth_test();
        ~final_depth_test();
    
};

class MakePoint
{
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    tf::StampedTransform transform; // tf::
    tf::TransformListener listener;
    tf::Matrix3x3 Rotation; 
    tf::Vector3 Translation;
    void make_point();
    void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    //bool a = false;
    bool is_success = false;
    
public:
    MakePoint();
        ~MakePoint();
};

#endif