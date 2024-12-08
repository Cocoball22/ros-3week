#include <ros/ros.h>
#include <turtlesim/Pose.h>

class Pose_Subscirber{
private:
    ros::NodeHandle nh;
    ros::Subscriber pose_sub;

    // pose 메시지 생성
    turtlesim::Pose pose_msg;

    // 현재 위치값
    float pose_x , pose_y, pose_theta, distance ; 
public:
    Pose_Subscirber() {
        // Subscirber 초기화
        pose_sub = nh.subscribe("/my_turtle/pose", 1 , &Pose_Subscirber::Subsciber_Pose, this);
    }

 
    void Subsciber_Pose(const turtlesim::Pose::ConstPtr& Subsciber_Pose) // pose의 구조체에서 x,y값을 가져옴 
    {  
        pose_x = Subsciber_Pose -> x;
        pose_y = Subsciber_Pose -> y;
        pose_theta = Subsciber_Pose -> theta;

        ROS_INFO("pose_x: %f, pose_y:%f, pose_theta:%f ",pose_x, pose_y, pose_theta); // data 메시지를 표현
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Pose_Subscirber_node");
    
    Pose_Subscirber Pose_Subscirber;
    
    ros::Rate rate(10); // 10Hz로 실행
    
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}