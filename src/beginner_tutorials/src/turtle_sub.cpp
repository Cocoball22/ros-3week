#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <std_srvs/Empty.h>

float vel =1.0; 

class Pose_Subscirber{
private:
    ros::NodeHandle nh;
    // vel_msg 선언
    ros::Publisher turtle_pub;
    geometry_msgs::Twist vel_msg;
    ros::Subscriber pose_sub;
    // 터틀심 생성
    ros::ServiceClient spawn_client;
    // pose 메시지 생성
    turtlesim::Pose pose_msg;
    ros::ServiceClient kill_client;
    ros::ServiceClient reset_client;

    // 현재 위치값
    float pose_x , pose_y, pose_theta; 
   
public:
    Pose_Subscirber() 
    {
        // publisher 초기화
        turtle_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        // Subscirber 초기화
        pose_sub = nh.subscribe("/turtle1/pose", 1 , &Pose_Subscirber::Subsciber_Pose, this);
        // spawn 서비스 클라이언트 생성
        spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
        kill_client = nh.serviceClient<turtlesim::Kill>("/kill");
        reset_client = nh.serviceClient<std_srvs::Empty>("/reset");
    }

    void go_command(float goal_x, float goal_y, float goal_theta)
    {

        float current_x, current_y, current_theta, distance;
        float x, y, theta;

        current_x = pose_x;
        current_y = pose_y;
        current_theta = pose_theta;

        x = goal_x - current_x;
        y = goal_y - current_y;
        // 거리 계산
        distance = sqrt(x*x + y*y);

        vel_msg.linear.x = 1;
        vel_msg.angular.z = 1;

        turtle_pub.publish(vel_msg);

        ROS_INFO("current_x: %f, current_y:%f, current_theta:%f",current_x, current_y, current_theta); // data 메시지를 표현
        ROS_INFO("x: %f, vel: %f, distance:%f",x, vel, distance); // data 메시지를 표현
    }

    // 위치값 읽기
    void Subsciber_Pose(const turtlesim::Pose::ConstPtr& Subsciber_Pose) // pose의 구조체에서 x,y값을 가져옴 
    {  
        pose_x = Subsciber_Pose -> x;
        pose_y = Subsciber_Pose -> y;
        pose_theta = Subsciber_Pose -> theta;
    }   

    // 생성
    void spawnTurtle(float x, float y, float theta, const std::string& name) 
    {
        turtlesim::Spawn srv;
        srv.request.x = x;
        srv.request.y = y;
        srv.request.theta = theta;
        srv.request.name = name;
        
        if (spawn_client.call(srv)) {
            ROS_INFO("Spawned turtle: %s", srv.response.name.c_str());
        }
    }

    // 리셋 터틀
    void resetTurtle() 
    {
        std_srvs::Empty srv;

        if (reset_client.call(srv)) {
            ROS_INFO("Reset turtle");
        }
    }

    // 죽이기
     void killTurtle(const std::string& turtle_name) 
    {
        turtlesim::Kill srv;
        srv.request.name = turtle_name;
        
        if (kill_client.call(srv)) {
            ROS_INFO("Killed turtle: %s", turtle_name.c_str());
        }
    }



};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Pose_Subscirber_node");
    ros::Time::init();
    ros::Rate rate(60);
    Pose_Subscirber Pose_Subscirber;

   while(ros::ok())
   {
    // Pose_Subscirber.spawnTurtle(2.0,3.0,1.57,"turtle2");
    Pose_Subscirber.go_command(8.544445,5.544445,0.0);

    ros::spinOnce();
    rate.sleep(); // 위에서 정한 루프 주기에 따라 슬립에 들어간다.
   }

}