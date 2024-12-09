#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h> // boolean 타입의 메시지 파일 헤더
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>


bool testing = false; 
float PI = 3.141592;

class turtle_time
{
private:
    ros::NodeHandle nh;

    ros::Publisher turtle_pub;
    //ros::Publisher pose_pub;

    ros::Subscriber turtle_position_sub;
    ros::Subscriber trigger_sub;

    ros::ServiceClient kill_client;
    ros::ServiceClient spawn_client;
    ros::ServiceClient reset_client;

    geometry_msgs::Twist vel_msg;

    turtlesim::Pose pose_msg;

    float pose_x , pose_y, pose_theta, distance ; // 현재 위치값
public:
    turtle_time()
    {
        turtle_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        //pose_pub = nh.advertise<turtlesim::Pose>("/turtle1/pose",10);
         
        trigger_sub = nh.subscribe("driver_start", 10 , &turtle_time::msgCallback, this);
        // turtle_position_sub = nh.subscribe("/turtle1/pose", 1 , &turtle_time::coordinate, this);
        // 서비스 클라이언트 초기화
        kill_client = nh.serviceClient<turtlesim::Kill>("/kill");
        spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
        reset_client = nh.serviceClient<std_srvs::Empty>("/reset");
    }

    

    void msgCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        ROS_INFO("Receive msg = %d", msg->data); // data 메시지를 표현
        testing = msg->data;
    }

    void coordinate(const turtlesim::Pose::ConstPtr& coordinate) // pose의 구조체에서 x,y값을 가져옴 
    {  
        pose_x = coordinate -> x;
        pose_y = coordinate -> y;
        pose_theta = coordinate -> theta;

        ROS_INFO("pose_x: %f, pose_y:%f, pose_theta:%f ",pose_x,pose_y,pose_theta); // data 메시지를 표현
    }

    void killTurtle(const std::string& turtle_name) 
    {
        turtlesim::Kill srv;
        srv.request.name = turtle_name;
        
        if (kill_client.call(srv)) {
            ROS_INFO("Killed turtle: %s", turtle_name.c_str());
        }
    }

    void resetTurtle() 
    {
        std_srvs::Empty srv;

        if (reset_client.call(srv)) {
            ROS_INFO("Reset turtle");
        }

       ros::Duration(2.0).sleep(); // 시작 전 잠시 대기
       vel_msg.linear.x = 0.0;
       vel_msg.angular.z = 0.0;
       turtle_pub.publish(vel_msg);

    }

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

    void drawCrown()
    {
        go_command(1.0,0.0,4.0);      // 1unit/s를 4초동안 직진
    
        ros::Duration(2.0).sleep();   // 시작 전 잠시 대기
        go_command(0,(PI*0.25),2.0);  // 45도를 2초동안 회전 => 90도

        ros::Duration(2.0).sleep();   // 시작 전 잠시 대기
        go_command(1.0,0.0,3.0);      // 1unit/s를 3초동안 직진

        ros::Duration(2.0).sleep();   // 시작 전 잠시 대기
        go_command(0,(PI*0.33),2.0);  // 60도를 2초동안 회전 => 120도

        ros::Duration(2.0).sleep();   // 시작 전 잠시 대기
        go_command(1.0,0,4.0);        // 1unit/s를 4초동안 직진

        ros::Duration(2.0).sleep();       // 시작 전 잠시 대기
        go_command(0,-(PI*0.11111),3.0);  // 시계 반향으로 20도를 3초동안 회전

        ros::Duration(2.0).sleep();       // 시작 전 잠시 대기
        go_command(1.0,0,4.0);            // 1unit/s를 4초동안 직진

        ros::Duration(2.0).sleep();       // 시작 전 잠시 대기
        go_command(0,(PI*0.25),2.0);      // 45도를 2초동안 회전 => 90도 

        ros::Duration(2.0).sleep();       // 시작 전 잠시 대기
        go_command(1.0,0.0,4.0);          // 1unit/s를 4초동안 직진

        stop_command();                   // 정지
    }

    void drawcircle()
    {
        go_command(PI*0.0625,PI*0.03125,64);                //  0.2unit/s, 5.62도/s, 64번 반시계 방향회전
        ros::Duration(2.0).sleep(); // 시작 전 잠시 대기
        go_command(PI*0.0625 ,-PI*0.03125, 32);             //  0.2unit/s, 5.62도/s, 32번 회전 시계방향 회전
        ros::Duration(2.0).sleep(); // 시작 전 잠시 대기전
        go_command(PI*0.0625 ,-PI*0.0625, 16);              //  0.2unit/s, 5.62도/s, 16번 회전 시계방향 회전
        ros::Duration(2.0).sleep(); // 시작 전 잠시 대기
        go_command(PI*0.0625 ,PI*0.0625, 16);               //  0.2unit/s, 5.62도/s, 16번 반시계 방향회전
        ros::Duration(2.0).sleep(); // 시작 전 잠시 대기
    }

    void go_command(float vel_linear_x, float vel_angular_z, float duration)
    {
        ros::Time start_time = ros::Time::now();
        // ros::Rate rate(10);

        vel_msg.linear.x = vel_linear_x;
        vel_msg.angular.z = vel_angular_z;
        
        // 지정된 duration 동안만 실행
        while (ros::Time::now() - start_time < ros::Duration(duration))
        {
            turtle_pub.publish(vel_msg);
        }
        
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        turtle_pub.publish(vel_msg);
    }

    void stop_command()
    {
        ROS_INFO("Complete\n\r");
        vel_msg.linear.x = 0.0;
        vel_msg.angular.z = 0.0;
        turtle_pub.publish(vel_msg);
        ros::Duration(2.0).sleep(); // 시작 전 잠시 대기
    }
};

int main(int argc, char** argv) 
{
    // 노드 초기화
    ros::init(argc, argv, "turtle_velocity_controller");
    ros::Time::init();
    ros::Rate rate(10);
    turtle_time turtle;

    while(ros::ok()) // roscore가 실행 되는 동안
    {
        if(testing)
        {
            turtle.drawCrown();
            turtle.resetTurtle();
            turtle.drawcircle();
            turtle.stop_command();
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
