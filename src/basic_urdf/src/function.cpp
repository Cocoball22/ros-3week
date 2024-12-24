#include "/home/cona/catkin_ws/src/basic_urdf/include/basic_urdf/test.hpp"

MakePoint::MakePoint()
{
   
    scan_sub = nh.subscribe("/scan1", 1, &MakePoint::counterCallback, this); 
    cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/revised_scan1",1); // 스캐닝 레이져의 pointcloud
    if(!is_success)
    {
      make_point();
    }
 
}

MakePoint::~MakePoint()
{
}

bool MakePoint::make_point()
{
  // bool a = false;

  //printf("111\n");
  try
  {

      ROS_INFO("222\n");
      listener.waitForTransform("base_link", "front_laser", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("base_link","front_laser", ros::Time(0), transform); // --> ros::Time(0),
      Rotation = tf::Matrix3x3(transform.getRotation()); // 회전 정보를 쿼터니언으로 반환
      Translation = tf::Vector3(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z()); // base_link 프레임을 기준
      is_success = true;

  }
  catch(tf::TransformException ex)
  {
    ROS_WARN("Error\n");
    
  }
    return is_success;
}

void MakePoint::counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  //printf("555\n");
  float x,y,z;
  float r,theta;
  
  tf::Vector3 laser_point, point_out; // 회전 적용
  sensor_msgs::PointCloud cloud;      // 카메라, 스캐닝 레이저 거리 측적기 등 일반적으로 사용되는 센서에 대한 메시지, 3차원 포인트 컬랙션
  geometry_msgs::Point32 point_out_; // 점, 벡터, 포즈와 같은 일반적인 가하학적 기본 요소에 대한 메시지
  int ranges_size = msg->ranges.size(); // 
 
  cloud.header.stamp = ros::Time::now(); // 센서 데이터 수집 시간
  cloud.header.frame_id = "base_link";   // 좌표 프레임 ID
  cloud.points.resize(ranges_size);
    for(int i = 0; i < ranges_size; i++)
    {
      r = msg->ranges[i]; // 현재 거리값
  
      if (!(r >= msg->range_min && r <= msg->range_max))
      {
            //printf("666\n");
            continue; // 무효한 데이터를 건너뜁니다
      }
      theta = msg->angle_min + (i * msg->angle_increment); // 현재 각도 계산
      x = r * cos(theta);
      y = r * sin(theta);
      z = 0.0;
      // ROS_INFO("r: %f, theta: %f", r, theta);
      // ROS_INFO("x: %f, y: %f, z: %f", x, y, z);
     
      laser_point.setValue(x, y, z);  // 한 줄로 값을 할당
      point_out = Rotation * laser_point + Translation;
      point_out_.x = point_out.x();
      point_out_.y = point_out.y();
      point_out_.z = point_out.z();
      // ROS_INFO(" point_out_ = (%f, %f, %f \n\r)",point_out_.x, point_out_.y, point_out_.z);
      cloud.points[i] = point_out_;
    }
    
    cloud_pub.publish(cloud);
    ROS_INFO("777\n");
}