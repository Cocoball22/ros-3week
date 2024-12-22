#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <cmath>

class final_depth
{
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub, depth_sub;
    ros::Publisher cloud_pub, depth_pub;
    tf::StampedTransform tf_scan,tf_depth;
    tf::TransformListener listener;
    tf::Matrix3x3 R,R_; 
    tf::Vector3 T,T_;
    cv::Mat RGBD_data;
    cv::Mat D_data;
public:
    final_depth()
    {
      scan_sub = nh.subscribe("/scan1", 1, &final_depth::scan_callback, this);
      depth_sub = nh.subscribe("/camera/depth/points",1, &final_depth::camera_callback, this);
      cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/revised_scan1",1);
      depth_pub = nh.advertise<sensor_msgs::PointCloud2>("/revised_depth", 1);

      try
      {
        listener.waitForTransform("base_link", "front_laser", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link","front_laser", ros::Time(0), tf_scan); // --> ros::Time(0),

        R = tf::Matrix3x3(tf_scan.getRotation()); // 회전 정보를 쿼터니언으로 반환
        T = tf::Vector3(tf_scan.getOrigin().x(),tf_scan.getOrigin().y(),tf_scan.getOrigin().z()); // base_link 프레임을 기준

        listener.waitForTransform("base_link", "camera_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link","camera_link", ros::Time(0), tf_depth); // --> ros::Time(0),

        R_ = tf::Matrix3x3(tf_depth.getRotation()); // 회전 정보를 쿼터니언으로 반환
        T_ = tf::Vector3(tf_depth.getOrigin().x(),tf_depth.getOrigin().y(),tf_depth.getOrigin().z()); // base_link 프레임을 기준
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

      // ROS_INFO("PointCloud2 Message Info: Height=%d, Width=%d, Data Size=%lu",depth_msg->height, depth_msg->width, depth_msg->data.size());

      
      // Initialize OpenCV matrices
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "base_link";
        cloud_msg.height = depth_msg->height;
        cloud_msg.width = depth_msg->width;
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = true;

        // PointCloud2 필드 정의 (XYZ)
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(depth_msg->height * depth_msg->width);

        // 매트릭스 초기화
        RGBD_data = cv::Mat(depth_msg->height, depth_msg->width, CV_32FC3);
        D_data = cv::Mat(depth_msg->height, depth_msg->width, CV_32F);
    
        // 데이터를 반복자와 OpenCV 매트릭스에 저장
        sensor_msgs::PointCloud2ConstIterator<float> D_iter_x(*depth_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> D_iter_y(*depth_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> D_iter_z(*depth_msg, "z");

        RGBD_data.setTo(0);
        D_data.setTo(0);

        for (int r_num = 0; r_num < depth_msg->height; ++r_num)
        {
            for (int c_num = 0; c_num < depth_msg->width ; ++c_num)
            {

                ROS_INFO("Row %d Col %d: \n",r_num,c_num);
                // Store the XYZ data in the OpenCV matrix
                RGBD_data.at<cv::Vec3f>(r_num, c_num)[0] = *D_iter_x;
                RGBD_data.at<cv::Vec3f>(r_num, c_num)[1] = *D_iter_y;
                RGBD_data.at<cv::Vec3f>(r_num, c_num)[2] = *D_iter_z;

                // Store the Z value as depth in a separate matrix
                D_data.at<float>(r_num, c_num) = RGBD_data.at<cv::Vec3f>(r_num,c_num)[2];

                ++D_iter_x;
                ++D_iter_y;
                ++D_iter_z;
            }
        }

     // 변환한 데이터를 다시 PointCloud2 메시지에 저장
        sensor_msgs::PointCloud2Iterator<float> out_iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_iter_z(cloud_msg, "z");
   
        tf::Vector3 transformed_point;
        cv::Vec3f point;

        for (int r = 0; r < RGBD_data.rows; ++r) 
        {
            for (int c = 0; c < RGBD_data.cols; ++c) 
            {
                point = RGBD_data.at<cv::Vec3f>(r, c);

                tf::Vector3 origin_point(point[0], point[1], point[2]);
                transformed_point = R_ * origin_point + T_;

                // ROS_INFO("Row %d Col %d: Original Point [%f, %f, %f] -> Transformed Point [%f, %f, %f] \n",
                //  r, c, origin_point.x(), origin_point.y(), origin_point.z(),
                //  transformed_point.x(), transformed_point.y(), transformed_point.z());

                *out_iter_x = transformed_point.x();
                *out_iter_y = transformed_point.y();
                *out_iter_z = transformed_point.z();

                ++out_iter_x;
                ++out_iter_y; 
                ++out_iter_z;
            }
        }

        depth_pub.publish(cloud_msg);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
      float x,y,z;
      float r,theta;
      
      tf::Vector3 laser_point, point_out; // 회전 적용
      sensor_msgs::PointCloud cloud;
      geometry_msgs::Point32 point_out_;
      int ranges_size = scan_msg->ranges.size();
     
      cloud.header.stamp = ros::Time::now();
      cloud.header.frame_id = "base_link";

      cloud.points.resize(ranges_size);

        for(int i = 0; i < ranges_size; i++)
        {
          r = scan_msg->ranges[i]; // 현재 거리값
      
          if (!(r >= scan_msg->range_min && r <= scan_msg->range_max))
          {
                continue; // 무효한 데이터를 건너뜁니다
          }

          theta = scan_msg->angle_min + (i * scan_msg->angle_increment); // 현재 각도 계산
          x = r * cos(theta);
          y = r * sin(theta);
          z = 0.0;
         
          laser_point.setValue(x, y, z);  // 한 줄로 값을 할당

          point_out = R * laser_point + T;

          point_out_.x = point_out.x();
          point_out_.y = point_out.y();
          point_out_.z = point_out.z();

          cloud.points[i] = point_out_;
        }
        
        cloud_pub.publish(cloud);
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "final_depth");

  final_depth fd;

  ros::spin();

  return 0;
};
