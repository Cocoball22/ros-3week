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
    tf::TransformListener listener;
    tf::StampedTransform transform;
    tf::Matrix3x3 R;
    tf::Vector3 T, laser_point, point_out; // 회전 적용
    tf::Quaternion q;
 
    sensor_msgs::PointCloud cloud;
public:
    make_point()
    {
        scan_sub = nh.subscribe("scan1", 1, &make_point::counterCallback, this);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/cloud",1);
    }   

    void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
      float x,y,z;
      while (nh.ok())
      {
        try
        {
          listener.waitForTransform("base_link", "front_laser", ros::Time(0), ros::Duration(3.0));
          listener.lookupTransform("base_link","front_laser",ros::Time(0), transform);

          R = tf::Matrix3x3(transform.getRotation()); // 회전 정보를 쿼터니언으로 반환
          T = tf::Vector3(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()); // base_link 프레임을 기준

          cloud.header.stamp = ros::Time::now();
          cloud.header.frame_id = "front_laser";

          // for(unsigned int i = 0; i < msg->ranges.size(); i++)
          // {
          //    float r = msg->ranges[i]; // 현재 거리값
          //    float angle_min = msg->angle_min;
          //    float theta = angle_min + i * msg->angle_increment; // 현재 각도 계산

          //    x = r * cos(theta);
          //    y = r * sin(theta);
          //    z = 0;

          //   laser_point = tf::Vector3(x, y, z);  // 한 줄로 값을 할당
          //   //printf("%.3f, %.3f, %.3f \n",laser_point.x(),laser_point.y(),laser_point.z());
          // }
          
           float distance = msg->range_max  - msg->range_min;
           float angle_min = msg->angle_min;
           float theta = angle_min + i * msg->angle_increment; // 현재 각도 계산
        }
        catch(tf::TransformException ex)
        {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }

      
              
              // cloud.points[i].x = point_out.x() + tanslation.x();
              // cloud.points[i].y = point_out.y() + tanslation.y();
              // cloud.points[i].z = point_out.z() + tanslation.z();
              // point_out = rotation_matrix * laser_point;

         cloud_pub.publish(cloud);
      }
    }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "make_point");
  make_point mp;

  ros::spin();
  return 0;
};
