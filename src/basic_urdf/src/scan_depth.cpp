#include "basic_urdf/scan_depth.hpp"

scan_depth::scan_depth()
{
    while(!succes_nonsucces && ros::ok())
    {
        listener_func();
    }
    // nh.subscribe ROS에서 제공하는 생성함수, 큐 크기는 버퍼에 저장될 수있는 메시지 수 큐 크기가 1로 설정하면, 최신 메시지만 유지되고 이전메시지는 삭제
    // this 콜백 함수가 클래스 멤버 함수 이기 때문에, 해당 클래스 인스턴스의 멤버 변수와 메서드에 접근
    // 노드가 scan1 토픽을 구족하고 /scan1 토픽에서 메시지가 퍼블리시 되면 subscriber가 수신 
    // 수신된 메시지는 scan_depth 클래스의 scan_callback 멤버 함수로 전달
    scan_sub =nh.subscribe("/scan1", 1, &scan_depth::scan_callback, this); 
    scan_pub = nh.advertise<sensor_msgs::PointCloud>("/revised_scan1",1);

    depth_sub = nh.subscribe("/camera/depth/points",1, &scan_depth::depth_callback, this);
    depth_tf_pub = nh.advertise<sensor_msgs::PointCloud>("/depth_tf",1);
    depth_non_pub = nh.advertise<sensor_msgs::PointCloud>("/depth_non_tf",1); 
}

scan_depth::~scan_depth()
{
    ROS_INFO("scan_depth test down.");

}

void scan_depth::listener_func()
{
      try
      {
        // scan tf
        // 변환 요청하는 기준 좌표계에서 변환 대상 좌표계
        listener_scan.waitForTransform("base_link", "front_laser", ros::Time(0), ros::Duration(3.0)); // 특정 좌표계의 변환 정보가 유효해질 때까지 기다림
        listener_scan.lookupTransform("base_link","front_laser", ros::Time(0), transform_scan); // 두 좌표계 간의 변환 정보를 3x3행렬로 변환
        // 여기서 클래스 tf::transform 클래스를 상속받아 시간과 좌표계 정보를 추가
        rotation_scan = tf::Matrix3x3(transform_scan.getRotation()); // 회전 정보를 쿼터니언으로 반환
        translation_scan = tf::Vector3(transform_scan.getOrigin().x(),transform_scan.getOrigin().y(),transform_scan.getOrigin().z()); // base_link 프레임을 기준
        
        // 회전변환 결과 확인
        std:: cout << "rotation matrix" << std::endl;
        for(int i = 0; i < 3; i++)
        {
            for(int j =0; j < 3; j++)
            {
                 std::cout << rotation_scan[i][j] <<" ";
            }
            std::cout << std::endl;
        }      

        // base_link로 부터 떨어진 거리
        std::cout << "TR: " <<  " x: " <<  translation_scan.getX() <<  " y: " <<  translation_scan.getY() <<  " z: " <<  translation_scan.getZ() << std::endl;

        // 회전값 확인
        double r, p, yaw;
        rotation_scan.getEulerYPR(yaw,p,r);
        std::cout << "r: " << r <<" p: " << p << " yaw: " << yaw << std::endl;

        //depth tf
        listener_depth.waitForTransform("base_link", "camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0)); // 카메라를 기준으로 들어오는 데이터의 기준이 camera_depth_optical_frame으로 한다
        listener_depth.lookupTransform("base_link","camera_depth_optical_frame", ros::Time(0), transform_depth);
        rotation_depth = tf::Matrix3x3(transform_depth.getRotation()); // 회전 정보를 쿼터니언으로 반환 getrotation 회전정보를 추출하여 3x3행려로 변환
        // tf 라이브러리에서 제공하는 벡터 연산용 클래스이고 좌표변환, 벡터연산, 회전 및 변환 계산에 사용

        translation_depth = tf::Vector3(transform_depth.getOrigin().x(),transform_depth.getOrigin().y(),transform_depth.getOrigin().z()); // base_link 프레임을 기준
        
        succes_nonsucces = true; // tf이 적용되면 참으로 변환
        std::cout << "listen succes" << std::endl;
      }
      catch(tf::TransformException ex)
        {
          ROS_WARN("%s",ex.what());
        }
}

void scan_depth::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    int ranges_size = scan_msg->ranges.size(); // pointcloud2의 데이터 크기
    
    // 카메라, 스캐닝 레이저 거리 데이터를 표현하고 3D 공간상의 여러 점을 하나의 메시지로 표현
    // 메시지의 시간 정보와 좌표계 표현 
    sensor_msgs::PointCloud scan; 
    scan.header.stamp = ros::Time::now(); // 현재 시간을 설정
    scan.header.frame_id = "base_link";   // 기준을 베이스 링크 좌표 프레임 ID
    scan.points.resize(ranges_size); // 배열의 크기를 조정
    
    tf::Vector3 laser_scan, scan_result; // 동적 배열 기능 laser_scan은 센서에서 측정한 값을 사용한 변수, scan_result는 베이스 링크를 기준으로 라이다 스캔 값이 변환된 결과값
    geometry_msgs::Point32 point_out_; // 계산된 좌표 데이터를 저장하는 변수
    float x, y, z; 
    float r, theta;

    for(int i = 0; i < ranges_size; i++)
    {
       r = scan_msg->ranges[i]; // 현재 거리값을 r에 대입
      
       if (!(r >= scan_msg->range_min && r <= scan_msg->range_max))
       {
             continue; // 무효한 데이터를 건너뜁니다
       }

       theta = scan_msg->angle_min + (i * scan_msg->angle_increment); // 현재 각도 계산
       // 극 좌표계를 카르테시안 좌표계
       x = r * cos(theta); 
       y = r * sin(theta);
       z = 0.0;

       std::cout << "scan data x : " << x << " y : " << y << " z : " << z << std::endl;

       laser_scan.setValue(x, y, z);  // 한 줄로 값을 할당
       scan_result = rotation_scan * laser_scan + translation_scan; // 회전 행렬 * laser_scan값 + base_link로 부터 직선
       point_out_.x = scan_result.x(); // 계산한 결과를 point_out에 대입
       point_out_.y = scan_result.y();
       point_out_.z = scan_result.z();
       std::cout << "scan_result x : " << point_out_.x  << " y : " << point_out_.y << " z : " << point_out_.z << std::endl;
       scan.points.push_back(point_out_); // points에 x,y,z 좌표를 한개 씩 밀어 넣는다
    }
    scan_pub.publish(scan);
}

void scan_depth::depth_callback(const sensor_msgs::PointCloud2::ConstPtr& depth_msg)
{   
    // 3D 점들의 집합을 표현하는 변수
    sensor_msgs::PointCloud depth_tf, depth_non_tf; // 카메라의 depth 이미지를 3D 공간상에서 점으로 표현할껀데 그 중 tf를 적용한 것과 안한것을 나타냄
    geometry_msgs::Point32 depth_result_, v_non_tf; // 계산된 좌표 데이터를 저장하고 points 필드에 geometry_msgs/Point32[] 타입의 배열로, 각 점의 (x, y, z) 좌표를 보낼 데이터

    depth_tf.header.stamp = ros::Time::now(); // 시작할 시점을 실행 시점으로 함
    depth_tf.header.frame_id = "base_link"; // 어느 좌표계를 기준으로 점들을 표현하는지를 지정 예를 들어 로봇 본체 중심 좌표계를 기준으로 점군 표현

    depth_non_tf.header.stamp = ros::Time::now();
    depth_non_tf.header.frame_id = "base_link";

    // x, y, z 좌표등에 접근하기 위한 상수 반복 float(32비트) 타입
    // iter_x는 depth_msg의 x 필드 값을 참조 -> sensor_msgs::PointCloud2 메시지의 모든 점에서 x 좌표를 읽기 위한 반복자

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*depth_msg, "x");  
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*depth_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*depth_msg, "z");

    tf::Vector3 depth_result; // 행렬 연산의 결과 변수
    
    // 현재 점의 x 좌표 값을 반환하고 ++iter_x: 다음 점의 x 좌표로 이동
    for (size_t i = 0; i < depth_msg->width * depth_msg->height; ++i, ++iter_x, ++iter_y, ++iter_z)
    {
        // ROS_INFO("1-Point[%ld]: x = %f, y = %f, z = %f", i, *iter_x, *iter_y, *iter_z);
        tf::Vector3 laser_depth(*iter_x,*iter_y,*iter_z); // 카메라에서 읽힌 좌표 점의 값

         // 2. 회전 및 평행 이동 적용 (3x3 * 3x1 곱셈을해서 변환된 벡터 반환 + baselink 좌표)
        depth_result = rotation_depth * laser_depth + translation_depth;
        
        // 3. tf::Vector3 depth_result(x,y,z) 좌표 계산한 벡터 값을 points 배열에 보낼 데이터 -> geometry_msgs::Point32 변환
        depth_result_.x = depth_result.x();
        depth_result_.y = depth_result.y();
        depth_result_.z = depth_result.z();

        v_non_tf.x = laser_depth.x();
        v_non_tf.y = laser_depth.y();
        v_non_tf.z = laser_depth.z();

        // 4. 변환된 포인트 추가
        depth_tf.points.push_back(depth_result_);
        depth_non_tf.points.push_back(v_non_tf);
    }

        // 5. 변환된 pointcloud 퍼블리싱 depth_tf_pub은 퍼블리시 변수이고 depth_tf는 좌표변환된 
        depth_tf_pub.publish(depth_tf);
        depth_non_pub.publish(depth_non_tf);
}