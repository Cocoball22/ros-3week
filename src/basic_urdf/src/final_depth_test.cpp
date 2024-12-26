#include "basic_urdf/final_depth_test.hpp"

final_depth_test::final_depth_test()
{
    if(!is_success && ros::ok())
    {
        final_depth();
    }
    depth_sub = nh.subscribe("/camera/depth/points",1, &final_depth_test::camera_callback, this); 
    depth_pub = nh.advertise<sensor_msgs::PointCloud>("/depth_tf",1);
    depth_pub_2 = nh.advertise<sensor_msgs::PointCloud>("/depth_non_tf",1);
    
}

final_depth_test::~final_depth_test()
{
    ROS_INFO("fianl_depth_ test down.");
}

bool final_depth_test::final_depth()
{
  try
  {
      listener.waitForTransform("base_link", "camera_depth_optical_frame", ros::Time(0), ros::Duration(3.0)); // 좌표 변화하는 시간이 걸려서 3초 대기
      listener.lookupTransform("base_link","camera_depth_optical_frame", ros::Time(0), tf_depth); // 좌표변화
      rotation_depth = tf::Matrix3x3(tf_depth.getRotation()); // 회전 정보를 쿼터니언으로 반환
      translation_depth = tf::Vector3(tf_depth.getOrigin().x(),tf_depth.getOrigin().y(),tf_depth.getOrigin().z()); // base_link 프레임을 기준
      is_success = true;
  }
  catch(tf::TransformException ex)
  {
    ROS_WARN("Error\n");
    
  }
    return is_success;
}

// 메시지를 전달 받음
void final_depth_test::camera_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{   
    
    sensor_msgs::PointCloud cloud_tf; 
    geometry_msgs::Point32 v_tf; // 포인트 클라우드에 출력할 데이터 자료형을 맞추려고 씀
    cloud_tf.header.stamp = ros::Time::now();
    cloud_tf.header.frame_id = "base_link";

    sensor_msgs::PointCloud cloud_non_tf;
    geometry_msgs::Point32 v_non_tf;
    cloud_non_tf.header.stamp = ros::Time::now();
    cloud_non_tf.header.frame_id = "base_link";

    // 특정 필드에 접근하여 반복(iteration)하며 값을 읽고, ConstIterator는 읽기 전용으로, 데이터를 수정하지 못합니다.
    // 메시지를 가리키는 포인터를 역참조하여 실제 데이터 메시지를 전달
    // 각 필드의 값이 float타입으로 저장
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"); // Pointcloud2 메시지에서  특정 필드(x,y,z)를 효율적을 순회
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    
    for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z)
    {
        // ROS_INFO("1-Point[%ld]: x = %f, y = %f, z = %f", i, *iter_x, *iter_y, *iter_z);
        tf::Vector3 v1(*iter_x,*iter_y,*iter_z);

         // 2. 회전 및 평행 이동 적용 (3x3 * 3x1 곱셈을해서 변환된 벡터 반환 + baselink 좌표)
        tf::Vector3 transformed_tf = rotation_depth * v1 + translation_depth;
        
        // 3. tf::Vector3 -> geometry_msgs::Point32 변환
        v_tf.x = transformed_tf.x();
        v_tf.y = transformed_tf.y();
        v_tf.z = transformed_tf.z();

        v_non_tf.x = v1.x();
        v_non_tf.y = v1.y();
        v_non_tf.z = v1.z();

        
        // 4. 변환된 포인트 추가
        cloud_tf.points.push_back(v_tf);
        cloud_non_tf.points.push_back(v_non_tf);
    }

        // 5. 변환된 pointcloud 퍼블리싱
        depth_pub.publish(cloud_tf);
        depth_pub_2.publish(cloud_non_tf);
}

// void final_depth_test::non_tf(const sensor_msgs::PointCloud2::ConstPtr& msg)
// {
//     sensor_msgs::PointCloud cloud_non_tf;
//     geometry_msgs::Point32 v_non_tf;
//     cloud_non_tf.header.stamp = ros::Time::now();
//     cloud_non_tf.header.frame_id = "base_link";

//     sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
//     sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
//     sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

//     std::cout << "11111" << std::endl;
//     for (size_t i = 0; i < msg->width * msg->height; ++i, ++iter_x, ++iter_y, ++iter_z)
//     {
//         // ROS_INFO("1-Point[%ld]: x = %f, y = %f, z = %f", i, *iter_x, *iter_y, *iter_z);
//         tf::Vector3 v1(*iter_x,*iter_y,*iter_z);
        
//         v_non_tf.x = v1.x();
//         v_non_tf.y = v1.y();
//         v_non_tf.z = v1.z();

//         // 4. 변환된 포인트 추가
//         cloud_non_tf.points.push_back(v_non_tf);
//     }
//         // 5. 변환된 pointcloud 퍼블리싱
//         depth_pub_2.publish(cloud_non_tf);
//         ROS_INFO("publish_nonTF");
// }