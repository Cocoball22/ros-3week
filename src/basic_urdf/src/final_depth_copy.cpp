#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <cmath>

class final_depth_test
{
private:
    ros::NodeHandle nh;
    ros::Subscriber depth_sub;
    ros::Publisher depth_pub;
    tf::StampedTransform tf_depth;
    tf::TransformListener listener;
    tf::Matrix3x3 rotation_depth; 
    tf::Vector3 translation_depth;

public:
    final_depth_test()
    {
      depth_sub = nh.subscribe("/camera/depth/points",1, &final_depth_test::camera_callback, this);
      depth_pub = nh.advertise<sensor_msgs::PointCloud2>("/revised_depth_test", 1);

      try
      {
        listener.waitForTransform("base_link", "camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link","camera_depth_optical_frame", ros::Time(0), tf_depth); // 

        rotation_depth = tf::Matrix3x3(tf_depth.getRotation()); // 회전 정보를 쿼터니언으로 반환
        translation_depth = tf::Vector3(tf_depth.getOrigin().x(),tf_depth.getOrigin().y(),tf_depth.getOrigin().z()); // base_link 프레임을 기준

          
      }
      catch(tf::TransformException ex)
        {
          ROS_WARN("%s",ex.what());
          return; // 오류시 콜백 종료
        }
    }   

    void camera_callback(const sensor_msgs::PointCloud2::ConstPtr& depth_msg)
    {
      sensor_msgs::PointCloud2 cloud_msg;

      // Initialize OpenCV matrices
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "base_link";
        cloud_msg.height = depth_msg->height;
        cloud_msg.width = depth_msg->width;
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = true;

        depth_pub.publish(cloud_msg);
    }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "final_depth_test");

  final_depth_test fdt;

  ros::spin();

  return 0;
};
