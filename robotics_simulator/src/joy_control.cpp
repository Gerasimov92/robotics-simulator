#include <string>
#include <map>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosStopSimulation.h>

//---------------------------------------------------------------------------
ros::Publisher twistPub;
ros::ServiceClient vrepStartSim;
ros::ServiceClient vrepStopSim;

std::map<std::string, int> axis_linear_map;
std::map<std::string, double> scale_linear_map;
std::map<std::string, int> axis_angular_map;
std::map<std::string, double> scale_angular_map;

//---------------------------------------------------------------------------
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    if(joy_msg->buttons[8] == 1)
    {
        vrep_common::simRosStartSimulation srv;
        if(vrepStartSim.call(srv))
        {
            ROS_INFO("V-REP simulation started");
        }
    }

    if(joy_msg->buttons[9] == 1)
    {
        vrep_common::simRosStopSimulation srv;
        vrepStopSim.call(srv);
        {
            ROS_INFO("V-REP simulation stopped");
        }
    }

    geometry_msgs::Twist cmd_vel_msg;

    if(axis_linear_map.find("x") != axis_linear_map.end())
    {
        cmd_vel_msg.linear.x = joy_msg->axes[axis_linear_map["x"]] * scale_linear_map["x"];
    }
    if(axis_linear_map.find("x") != axis_linear_map.end())
    {
        cmd_vel_msg.linear.y = joy_msg->axes[axis_linear_map["y"]] * scale_linear_map["y"];
    }
    if(axis_linear_map.find("z") != axis_linear_map.end())
    {
        cmd_vel_msg.linear.z = joy_msg->axes[axis_linear_map["z"]] * scale_linear_map["z"];
    }
    if(axis_angular_map.find("yaw") != axis_angular_map.end())
    {
        cmd_vel_msg.angular.z = joy_msg->axes[axis_angular_map["yaw"]] * scale_angular_map["yaw"];
    }
    if(axis_angular_map.find("pitch") != axis_angular_map.end())
    {
        cmd_vel_msg.angular.y = joy_msg->axes[axis_angular_map["pitch"]] * scale_angular_map["pitch"];
    }
    if(axis_angular_map.find("roll") != axis_angular_map.end())
    {
        cmd_vel_msg.angular.x = joy_msg->axes[axis_angular_map["roll"]] * scale_angular_map["roll"];
    }

    twistPub.publish(cmd_vel_msg);
}

//---------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_control");

    ROS_INFO("Waiting for sensor_msgs::Joy messages...");

    ros::NodeHandle nh_param("~");
    nh_param.param<int>("axis_linear", axis_linear_map["x"], 1);
    nh_param.param<double>("scale_linear", scale_linear_map["x"], 0.5);
    nh_param.param<int>("axis_angular", axis_angular_map["yaw"], 2);
    nh_param.param<double>("scale_angular", scale_angular_map["yaw"], 0.5);

    ros::NodeHandle n;
    ros::Subscriber joySub = n.subscribe("joy", 1, joyCallback);

    twistPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    vrepStartSim = n.serviceClient<vrep_common::simRosStartSimulation>("vrep/simRosStartSimulation");
    vrepStopSim = n.serviceClient<vrep_common::simRosStopSimulation>("vrep/simRosStopSimulation");

    ros::spin();

    return 0;
}

//---------------------------------------------------------------------------
