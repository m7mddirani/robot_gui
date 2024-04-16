#include "robot_gui/robot_gui.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include <ros/ros.h>



int main (int argc, char **argv){
    
    ros::init(argc, argv, "robot_gui");
    ros::NodeHandle nh;

    RobotGui received_messages = RobotGui(&nh); 
    received_messages.run(); 

    return 0;
}