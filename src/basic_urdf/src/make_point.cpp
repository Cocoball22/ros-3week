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
      scan_sub = nh.subscribe("scan1", 1, &make_point::counterCallback, this);
      cloud_pub = nh.advertise<sensor_msgs::PointCloud>("revised_scan1",1);

      try
      {
        listener.waitForTransform("base_link", "front_laser", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link","front_laser", ros::Time(0), transform); // --> ros::Time(0),

        R = tf::Matrix3x3(transform.getRotation()); // 회전 정보를 쿼터니언으로 반환
        T = tf::Vector3(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()); // base_link 프레임을 기준
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
     
      cloud.header.stamp = ros::Time::now();
      cloud.header.frame_id = "base_link";

      // ROS_INFO("before clear Cloud points size: %lu", cloud.points.size());

      // cloud.points.clear();  // 한번 실행 후 clear
      // 현재 resize된 크기 출력
      // ROS_INFO("after clear Cloud points size: %lu", cloud.points.size());

      cloud.points.resize(msg->ranges.size());

      // 현재 resize된 크기 출력
      // ROS_INFO("resize Cloud points size: %lu", cloud.points.size());

        for(int i = 0; i < msg->ranges.size(); i++)
        {
          r = msg->ranges[i]; // 현재 거리값
          theta = msg->angle_min + (i * msg->angle_increment); // 현재 각도 계산

          if (std::isinf(r) || r < msg->range_min || r > msg->range_max) 
          {
                continue; // 무효한 데이터를 건너뜁니다
          }

          ROS_DEBUG(" r:%f, angle_min:%f, theta:%f \n\r",r,msg->angle_min,theta);

          x = r * cos(theta);
          y = r * sin(theta);
          z = 0.0;
          
          // // 만약 범위를 넘어가게 되면 0으로 초기화
          // if((r >= msg->range_min && r  <= msg->range_max) && (theta >= msg->angle_min && theta <= msg->angle_max))
          // {
          //     x = r * cos(theta);
          //     y = r * sin(theta);
          //     z = 0;
          // }
          // else
          // {
          //   x = 0;
          //   y = 0;
          //   z = 0;
          // }
        
         ROS_DEBUG(" x:%f, y:%f, z:%f \n\r",x,y,z);
         
          laser_point = tf::Vector3(x, y, z);  // 한 줄로 값을 할당

          point_out = R * laser_point + T;
          
         //  printf("%.2f, %.2f, %.2f\n\r",transformed_point.x(), transformed_point.y(),transformed_point.z());
          // 각 점을 직접 수정하려는 경우
          // cloud.points[i].x = point_out.x();
          // cloud.points[i].y = point_out.y();
          // cloud.points[i].z = point_out.z();
          // printf("%.2f, %.2f, %.2f\n\r",cloud.points[i].x , cloud.points[i].y ,cloud.points[i].z );

          point_out_.x = point_out.x();
          point_out_.y = point_out.y();
          point_out_.z = point_out.z();
          cloud.points[i] = point_out_;
          // cloud.points.push_back(point_out_); // 변환된 점들을 pointcloud 형태로 저장해 새로운 점 추가
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
