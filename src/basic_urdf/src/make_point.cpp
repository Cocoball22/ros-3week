#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/tf.h>
#include <cmath>

class make_point
{
private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher cloud_pub;
    tf::StampedTransform transform;
    tf::TransformListener listener;
    tf::Matrix3x3 R; 
    tf::Vector3 T;
    
public:
    make_point()
    {
      scan_sub = nh.subscribe("/scan1", 1, &make_point::counterCallback, this);
      cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/revised_scan1",1);

      try
      {
        listener.waitForTransform("base_link", "front_laser", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link","front_laser", ros::Time(0), transform); // --> ros::Time(0),

        R = tf::Matrix3x3(transform.getRotation()); // 회전 정보를 쿼터니언으로 반환
        T = tf::Vector3(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()); // base_link 프레임을 기준
        ROS_INFO("Transform matrix_1: [%f, %f, %f]\n\r", R[0][0], R[0][1], R[0][2]);
        ROS_INFO("Transform matrix_2: [%f, %f, %f]\n\r", R[1][0], R[1][1], R[1][2]);
        ROS_INFO("Transform matrix_3: [%f, %f, %f]\n\r", R[2][0], R[2][1], R[2][2]);
        ROS_INFO("Translation vector: [%f, %f, %f]\n\r", T.x(), T.y(), T.z());

      }
      catch(tf::TransformException ex)
        {
          ROS_WARN("%s",ex.what());
          return; // 오류시 콜백 종료
        }
    }   

    void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      float x,y,z;
      float r,theta;
      
      tf::Vector3 laser_point, point_out; // 회전 적용
      sensor_msgs::PointCloud cloud;
      geometry_msgs::Point32 point_out_;
      int ranges_size = msg->ranges.size();
     
      cloud.header.stamp = ros::Time::now();
      cloud.header.frame_id = "base_link";

      cloud.points.resize(ranges_size);

        for(int i = 0; i < ranges_size; i++)
        {
          r = msg->ranges[i]; // 현재 거리값
      
          if (!(r >= msg->range_min && r <= msg->range_max))
          {
                continue; // 무효한 데이터를 건너뜁니다
          }

          theta = msg->angle_min + (i * msg->angle_increment); // 현재 각도 계산
          x = r * cos(theta);
          y = r * sin(theta);
          z = 0.0;
          ROS_INFO("r: %f, theta: %f", r, theta);
          ROS_INFO("x: %f, y: %f, z: %f", x, y, z);

         
          laser_point.setValue(x, y, z);  // 한 줄로 값을 할당

          point_out = R * laser_point + T;

          point_out_.x = point_out.x();
          point_out_.y = point_out.y();
          point_out_.z = point_out.z();

          ROS_INFO(" point_out_ = (%f, %f, %f \n\r)",point_out_.x, point_out_.y, point_out_.z);
          cloud.points[i] = point_out_;
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
