#include <ros/ros.h>
#include <turtlesim/Pose.h>

class Pose_Publisher {
private:
    ros::NodeHandle nh;
    ros::Publisher pose_pub;

    // pose 메시지 생성
    turtlesim::Pose pose_msg;
public:
    Pose_Publisher() {
        // Publisher 초기화
        pose_pub = nh.advertise<turtlesim::Pose>("/turtle2/pose", 10);
    }

    void publishPose() {
        // 값 설정
        pose_msg.x = 5.0;
        pose_msg.y = 5.0;
        pose_msg.theta = 1.57;
        pose_msg.linear_velocity = 1;
        pose_msg.angular_velocity = 1;

        // publish
        pose_pub.publish(pose_msg);
        ROS_INFO("Published pose - x: %.2f, y: %.2f, theta: %.2f", 
                 pose_msg.x, pose_msg.y, pose_msg.theta);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Pose_Publisher_node");
    
    Pose_Publisher pose_publisher;
    
    ros::Rate rate(10); // 10Hz로 실행
    
    while (ros::ok()) {
        pose_publisher.publishPose();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}