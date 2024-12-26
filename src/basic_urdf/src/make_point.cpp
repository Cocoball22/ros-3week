#include <ros/ros.h>
#include <tf/transform_listener.h> // tf::TransformListener를 생성하는데 필요한 헤더파일 
#include <sensor_msgs/LaserScan.h> // 
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
    tf::Matrix3x3 Rotation; 
    tf::Vector3 Translation;
    
public:
    make_point()
    {
      scan_sub = nh.subscribe("/scan1", 1, &make_point::counterCallback, this); 
      cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/revised_scan1",1); // 스캐닝 레이져의 pointcloud

      try
      {
        listener.waitForTransform("base_link", "front_laser", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("base_link","front_laser", ros::Time(0), transform); // --> ros::Time(0),

        Rotation = tf::Matrix3x3(transform.getRotation()); // 회전 정보를 쿼터니언으로 반환
        Translation = tf::Vector3(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()); // base_link 프레임을 기준
      
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
      
      tf::Vector3 laser_point, point_out; // 레이저센서에서 측정된 좌표값
      sensor_msgs::PointCloud cloud;      // 카메라, 스캐닝 레이저 거리 측적기 등 일반적으로 사용되는 센서에 대한 메시지, 3차원 포인트 컬랙션
      geometry_msgs::Point32 point_out_; // 점, 벡터, 포즈와 같은 일반적인 가하학적 기본 요소에 대한 메시지

      int ranges_size = msg->ranges.size(); // 메시지의 사이즈 정의 
     
      cloud.header.stamp = ros::Time::now(); // 센서 데이터 수집 시간
      cloud.header.frame_id = "base_link";   // 좌표 프레임 ID

      cloud.points.resize(ranges_size); // 벡터를 매번 동적할당을 하지않고 사이즈를 고정해서 사용
     

        for(int i = 0; i < ranges_size; i++) // 
        {
          r = msg->ranges[i]; // 측정된 거리값
      
          if (!(r >= msg->range_min && r <= msg->range_max))
          {
                continue; // 무효한 데이터를 건너뜁니다
          }

          theta = msg->angle_min + (i * msg->angle_increment); // 현재 각도 계산
          x = r * cos(theta); 
          y = r * sin(theta);
          z = 0.0;
         
          laser_point.setValue(x, y, z);  // 한 줄로 값을 할당

          point_out = Rotation * laser_point + Translation;

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
  ros::init(argc, argv, "make_point");

  make_point mp;

  ros::spin();

  return 0;
};
