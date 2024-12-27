#ifndef SCAN_DEPTH_CPP
#define SCAN_DEPTH_CPP

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "opencv2/opencv.hpp"
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>
#include <cmath>

class scan_depth
{
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub, depth_sub;
    ros::Publisher scan_pub, depth_tf_pub, depth_non_pub;
    tf::StampedTransform transform_scan, transform_depth; // 베이스 링크와 스캔 또는 뎁스 카메라의 좌표 변환
    tf::TransformListener listener_scan, listener_depth; // 변환된 좌표 값을 수신
    tf::Matrix3x3 rotation_scan, rotation_depth; // 3x3 회전 행렬을 다루기 위한 변수
    tf::Vector3 translation_scan, translation_depth;

  void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  void depth_callback(const sensor_msgs::PointCloud2::ConstPtr& depth_msg);
  bool succes_nonsucces = false;
  void listener_func();

public:
    scan_depth(/* args */);
    ~scan_depth();
};



#endif