#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include <robot_gui/cvui.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include <robot_gui/gui_subscirbe.h>
#include <string>
#define CVUI_IMPLEMENTATION
#include <robot_gui/cv_gui.h>

int main(int argc, char**argv){
    ros::init(argc,argv,"robot_gui");
    ros::NodeHandle n1("robot_gui_rb");
    std::string rb_info_topic = "/robot_info";
    std::string cmd_topic = "/cmd_vel"; 
    std::string odom_topic ="/odom";
    Guisubscribe<robotinfo_msgs::RobotInfo10Fields> rb_gui(n1, rb_info_topic);
    Guisubscribe<geometry_msgs::Twist> cmd_gui(n1, cmd_topic);
    Guisubscribe<nav_msgs::Odometry> odom_gui(n1, odom_topic);

    ros::Rate r(2);
    robotinfo_msgs::RobotInfo10Fields *rb_ptr = &rb_gui.rb_data;
    geometry_msgs::Twist *cmd_ptr = &cmd_gui.vel_data;
    nav_msgs::Odometry *odom_ptr = &odom_gui.od_data;
    CV tt(&n1,rb_ptr,cmd_ptr,odom_ptr);
    tt.run();  
    // ros::ServiceClient gd_service = n1.serviceClient<std_srvs::Trigger>("/get_distance");
    // std_srvs::Trigger srv;
    // gd_service.call(srv);
    // ROS_INFO("%s",srv.response.message.c_str());
    // ros::spin();

    return 0;
}

