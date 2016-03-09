#include <math.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include "vrep_common/VrepInfo.h"

//---------------------------------------------------------------------------
#define WHEEL_BASE          0.5 // The distance in m from the center wheel to the middle two wheel
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

    // Integrate odometry
    if(fabs(angularDist) < 1e-6)
    {
        double direction = th + angularDist * 0.5;

        // Runge-Kutta 2nd order integration:
        x += linearDist * cos(direction);
        y += linearDist * sin(direction);
        th += angularDist;
    }
    else
    {
        // Exact integration (should solve problems when angular is zero):
        double headingOld = th;
        double r = linearDist / angularDist;
        th += angularDist;
        x += r * (sin(th) - sin(headingOld));
        y += -r * (cos(th) - cos(headingOld));
    }

    geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(th);

    // Publish the transform over tf
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

    // Publish odom topic
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
    // Update joint_state
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
    // Front wheel speed in m/s
    wheelSpeed = msg->data;
}

//---------------------------------------------------------------------------
void tricycleAngleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Front wheel rotational angle in rad
    alpha = msg->data * M_PI / 180.0;

    UpdateJointState();
}

//---------------------------------------------------------------------------
void vrepInfoCallback(const vrep_common::VrepInfo::ConstPtr& msg)
{
    currentTime = ros::Time::now();
    //double dt = (currentTime - lastTime).toSec();
    //lastTime = currentTime;

    currentSimTime = msg->simulationTime.data;
    //double dt = msg->timeStep.data;
    double dt = currentSimTime - lastSimTime;
    //ROS_INFO("dt = %f", dt);
    ROS_INFO("dt = %f", currentSimTime);

    UpdateOdom(dt, currentTime);

    /*tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped odomTrans;
    odomTrans.header.frame_id = "odom";
    odomTrans.child_frame_id = "axis";
    // Update transform
    odomTrans.header.stamp = ros::Time::now();
    odomTrans.transform.translation.x = x;
    odomTrans.transform.translation.y = y;
    odomTrans.transform.translation.z = 0;
    odomTrans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
    // Send the transform
    broadcaster.sendTransform(odomTrans);*/

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

        //currentTime = ros::Time::now();
        //double dt = (currentTime - lastTime).toSec();

        //UpdateOdom(dt, currentTime);

        // Update transform for URDF (axis)
        odomTrans.header.stamp = ros::Time::now();
        odomTrans.transform.translation.x = x;
        odomTrans.transform.translation.y = y;
        odomTrans.transform.translation.z = 0;
        odomTrans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
        // Send the transform
        broadcaster.sendTransform(odomTrans);

        //lastTime = currentTime;

        loopRate.sleep();
    }

    //ros::spin();

    return 0;
}

//---------------------------------------------------------------------------
