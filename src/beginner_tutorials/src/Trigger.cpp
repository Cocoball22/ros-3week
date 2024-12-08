#include <ros/ros.h>
#include <std_msgs/Bool.h>

class Trigger_turtle_velocity_controller
{
    private:
        ros::NodeHandle nh;
        ros::Publisher Trigger_pub;
        std_msgs::Bool Trigger_msg;
        bool testing;
    public:
        Trigger_turtle_velocity_controller()
        {
            Trigger_pub = nh.advertise<std_msgs::Bool>("/driver_start", 10);
            testing = false; // 초기값 설정
        }

        void trigger_on()
        {
                ROS_INFO("Trigger ON");
                Trigger_msg.data = true;
                testing = Trigger_msg.data;
                Trigger_pub.publish(Trigger_msg);
        }

        void trigger_off()
        {
            ROS_INFO("Trigger OFF");
            Trigger_msg.data = false;
            Trigger_pub.publish(Trigger_msg);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Trigger_Button");
    ros::Time::init();
    ros::Rate rate(10);
    Trigger_turtle_velocity_controller ctr_trigger;

    while (ros::ok())
    {
        ctr_trigger.trigger_on();
        
        ros::spinOnce();
        rate.sleep();
    }

}