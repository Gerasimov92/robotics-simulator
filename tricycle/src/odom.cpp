#include <math.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include "vrep_common/VrepInfo.h"

//---------------------------------------------------------------------------
// Колесная база и радиус колес (в метрах)
#define WHEEL_BASE          0.5
#define WHEEL_RADIUS        0.1

//---------------------------------------------------------------------------
double wheelSpeed;
double alpha;
ros::Time lastTime;
ros::Time currentTime;
float lastSimTime = 0;
float currentSimTime = 0;
double th = 0;
double x = 0;
double y = 0;
ros::Subscriber velSub;
ros::Subscriber angleSub;
ros::Subscriber vrepInfoSub;
ros::Publisher odomPub;
ros::Publisher jointPub;
sensor_msgs::JointState jointState;

//---------------------------------------------------------------------------
void UpdateOdom(double dt, const ros::Time &odomTime)
{
    if(velSub.getNumPublishers() == 0)
        wheelSpeed = 0;
    if(angleSub.getNumPublishers() == 0)
        alpha = 0;

    double linearVel = wheelSpeed * cos(alpha);
    double angularVel = wheelSpeed / WHEEL_BASE * sin(alpha);

    double linearDist = linearVel * dt;
    double angularDist = angularVel * dt;

    // Интегрируем
    if(fabs(angularDist) < 1e-6)
    {
        double direction = th + angularDist * 0.5;

        // Рунге-Кутта 2го порядка
        x += linearDist * cos(direction);
        y += linearDist * sin(direction);
        th += angularDist;
    }
    else
    {
        // Явная интеграция
        double headingOld = th;
        double r = linearDist / angularDist;
        th += angularDist;
        x += r * (sin(th) - sin(headingOld));
        y += -r * (cos(th) - cos(headingOld));
    }

    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(th);

    // Публикуем преобразованием СК через tf
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.stamp = odomTime;
    odomTrans.header.frame_id = "odom";
    odomTrans.child_frame_id = "base_link";
    odomTrans.transform.translation.x = x;
    odomTrans.transform.translation.y = y;
    odomTrans.transform.translation.z = 0.0;
    odomTrans.transform.rotation = odomQuat;
    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(odomTrans);

    // Публикуем одометрию
    nav_msgs::Odometry odom;
    odom.header.stamp = odomTime;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odomQuat;
    odom.twist.twist.linear.x = linearVel;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = angularVel;
    odomPub.publish(odom);
}

//---------------------------------------------------------------------------
void UpdateJointState()
{
    // Обновляем положение звеньев
    jointState.header.stamp = ros::Time::now();
    jointState.name[0] ="steering_joint";
    jointState.position[0] = alpha;
    jointState.name[1] ="front_wheel_joint";
    jointState.position[1] = 0;
    jointState.name[2] ="left_wheel_joint";
    jointState.position[2] = 0;
    jointState.name[3] ="right_wheel_joint";
    jointState.position[3] = 0;
    jointPub.publish(jointState);
}

//---------------------------------------------------------------------------
void tricycleVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Скорость переднего колеса в м/с
    wheelSpeed = msg->data;
}

//---------------------------------------------------------------------------
void tricycleAngleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Угол поворота переднего колеса в рад
    alpha = msg->data * M_PI / 180.0;

    UpdateJointState();
}

//---------------------------------------------------------------------------
void vrepInfoCallback(const vrep_common::VrepInfo::ConstPtr& msg)
{
    currentTime = ros::Time::now();
    currentSimTime = msg->simulationTime.data;
    double dt = currentSimTime - lastSimTime;

    UpdateOdom(dt, currentTime);

    lastSimTime = currentSimTime;
}

//---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tricycle_odom");

    ROS_INFO("Waiting for tricycle vel and angle topics...");

    ros::NodeHandle n;
    velSub = n.subscribe("vrep/tricycle_vel", 1, tricycleVelCallback);
    angleSub = n.subscribe("vrep/tricycle_angle", 1, tricycleAngleCallback);
    vrepInfoSub = n.subscribe("vrep/info", 1, vrepInfoCallback);
    odomPub = n.advertise<nav_msgs::Odometry>("odom", 50);
    jointPub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

    lastTime = ros::Time::now();
    lastSimTime = 0;

    tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.frame_id = "odom";
    odomTrans.child_frame_id = "axis";

    jointState.name.resize(4);
    jointState.position.resize(4);

    ros::Rate loopRate(200);

    while(n.ok())
    {
        ros::spinOnce();

        // Обновляем преобразование СК для визуализации URDF
        odomTrans.header.stamp = ros::Time::now();
        odomTrans.transform.translation.x = x;
        odomTrans.transform.translation.y = y;
        odomTrans.transform.translation.z = 0;
        odomTrans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
        broadcaster.sendTransform(odomTrans);

        loopRate.sleep();
    }

    return 0;
}

//---------------------------------------------------------------------------
