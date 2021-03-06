#include <math.h>
#include <signal.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

//---------------------------------------------------------------------------
// Колесная база и радиус колес (в метрах)
#define WHEEL_BASE          0.5
#define WHEEL_RADIUS        0.1

//---------------------------------------------------------------------------
ros::Publisher cmd_vel_pub;
ros::Publisher cmd_angle_pub;
int timeout = 0;
float vel = 0;
float angle = 0;

//---------------------------------------------------------------------------
void twistCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    vel = sqrt(pow(twist->linear.x, 2) + pow(twist->angular.z, 2) * pow(WHEEL_BASE, 2));
    if(twist->linear.x < 0)
        vel = -vel;

    angle = 0;
    if(twist->linear.x >= 0)
        angle = atan2(twist->angular.z * WHEEL_BASE, twist->linear.x) * 180.0 / M_PI;
    else
        angle = atan2(-twist->angular.z * WHEEL_BASE, -twist->linear.x) * 180.0 / M_PI;

    std_msgs::Float32 cmd_vel_msg;
    cmd_vel_msg.data = vel;
    cmd_vel_pub.publish(cmd_vel_msg);

    std_msgs::Float32 cmd_angle_msg;
    cmd_angle_msg.data = angle;
    cmd_angle_pub.publish(cmd_angle_msg);
}

//---------------------------------------------------------------------------
void sigintHandler(int sig)
{
    std_msgs::Float32 cmd_vel_msg;
    cmd_vel_msg.data = 0;
    cmd_vel_pub.publish(cmd_vel_msg);

    ros::shutdown();
}

//---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tricycle_control", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    signal(SIGINT, sigintHandler);

    ROS_INFO("Waiting for teleop twist message...");

    ros::Subscriber sub = n.subscribe("cmd_vel", 1, twistCallback);

    cmd_vel_pub = n.advertise<std_msgs::Float32>("vrep/cmd_vel", 1);
    cmd_angle_pub = n.advertise<std_msgs::Float32>("vrep/cmd_angle", 1);

    ros::spin();

    return 0;
}

//---------------------------------------------------------------------------
