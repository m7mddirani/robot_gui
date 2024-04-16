#include "nav_msgs/Odometry.h"
#include "robot_gui/robot_gui.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <string>
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include <ros/ros.h>

RobotGui::RobotGui(ros::NodeHandle *node_handle){

    nh = node_handle;
    general_info_area_init();
    teleoperation_buttons();
    odometry();
    trigger_service(std::string("/get_distance"));
}

void RobotGui::general_info_area_init(){
    topic_name_sub = "robot_info";
    sub_ = nh->subscribe<robotinfo_msgs::RobotInfo10Fields>(topic_name_sub, 10,
                                          &RobotGui::msgCallback, this);
    ROS_INFO("Subscribed to: %s", topic_name_sub.c_str());
}

void RobotGui::teleoperation_buttons(){
    twist_topic_name = "cmd_vel";
    twist_pub_ = nh->advertise<geometry_msgs::Twist>(twist_topic_name, 10);
}

void RobotGui::odometry(){
    odom_topic_name = "odom";
    odom_sub_ = nh->subscribe<nav_msgs::Odometry>(
            odom_topic_name, 2, &RobotGui::OdomMsgCallback, this);
}

void RobotGui::msgCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg){
    data = *msg;
}

void RobotGui::OdomMsgCallback(const nav_msgs::Odometry::ConstPtr &msg){
    odom_data = *msg;
    ROS_DEBUG("Position x,y,z: [%0.2f, %0.2f, %0.2f]", msg->pose.pose.position.x,
            msg->pose.pose.position.y, msg->pose.pose.position.z);
}

void RobotGui::trigger_service(const std::string &srv_name){
    service_client = nh->serviceClient<std_srvs::Trigger>(srv_name);
    service_name = srv_name;       
}


void RobotGui::run(){

/*GENERAL INFO */

    // Window Size in Pixel
    cv::Mat frame = cv::Mat(800, 500, CV_8UC3);

    // Init a OpenCV window and tell cvui to use it.
    cv::namedWindow(WINDOW_NAME);
    cvui::init(WINDOW_NAME);

    while (ros::ok()){
    
    // Fill the frame with a nice color
    frame = cv::Scalar(20, 20, 20);

    // Create window at (40, 20) with size 250x80 (width x height) and title
    cvui::window(frame, 50, 50, 250, 250, "Topic: " + topic_name_sub);

    // Show how many times the button has been clicked inside the window.
    cvui::printf(frame, 55, 75, 0.4, 0xff0000, "1 - %s", data.data_field_01.c_str());
    cvui::printf(frame, 55, 95, 0.4, 0xff0000, "2 - %s", data.data_field_02.c_str());
    cvui::printf(frame, 55, 115, 0.4, 0xff0000, "3 - %s", data.data_field_03.c_str());
    cvui::printf(frame, 55, 135, 0.4, 0xff0000, "4 - %s", data.data_field_04.c_str());
    cvui::printf(frame, 55, 155, 0.4, 0xff0000, "5 - %s", data.data_field_05.c_str());
    cvui::printf(frame, 55, 175, 0.4, 0xff0000, "6 - %s", data.data_field_06.c_str());
    cvui::printf(frame, 55, 195, 0.4, 0xff0000, "7 - %s", data.data_field_07.c_str());
    cvui::printf(frame, 55, 215, 0.4, 0xff0000, "8 - %s", data.data_field_08.c_str());
    cvui::printf(frame, 55, 235, 0.4, 0xff0000, "9 - %s", data.data_field_09.c_str());
    cvui::printf(frame, 55, 255, 0.4, 0xff0000, "10 - %s", data.data_field_10.c_str());


/* CMD VEL PUBLISHER */
    
    // Show a button at position x = 100, y = 20
    if (cvui::button(frame, 135, 350, " Forward ")) {
      // The button was clicked, update the Twist message
      twist_msg.linear.x = twist_msg.linear.x + linear_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 100, y = 50
    if (cvui::button(frame, 135, 400, "   Stop  ")) {
      // The button was clicked, update the Twist message
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 30, y = 50
    if (cvui::button(frame, 50, 400, " Left ")) {
      // The button was clicked, update the Twist message
      twist_msg.angular.z = twist_msg.angular.z + angular_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 195, y = 50
    if (cvui::button(frame, 245, 400, " Right ")) {
      // The button was clicked, update the Twist message
      twist_msg.angular.z = twist_msg.angular.z - angular_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Show a button at position x = 100, y = 80
    if (cvui::button(frame, 135, 450, "Backward")) {
      // The button was clicked,update the Twist message
      twist_msg.linear.x = twist_msg.linear.x - linear_velocity_step;
      twist_pub_.publish(twist_msg);
    }

    // Create window at (320, 20) with size 120x40 (width x height) and title
    cvui::window(frame, 350, 350, 120, 40, "Linear velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 375, 375, 0.4, 0xff0000, "%.02f m/sec",
                 twist_msg.linear.x);

    // Create window at (320 60) with size 120x40 (width x height) and title
    cvui::window(frame, 350, 400, 120, 40, "Angular velocity:");
    // Show the current velocity inside the window
    cvui::printf(frame, 375, 425, 0.4, 0xff0000, "%.02f rad/sec",
                 twist_msg.angular.z);
    
/* ODOMETRY */
    
    // Square for  X
    cvui::window(frame, 50, 520, 80, 100, "X");
    cvui::printf(frame, 60, 565, 0.6, 0xffffff,
                 "%0.2f",odom_data.pose.pose.position.x);
    // Square for  Y
    cvui::window(frame, 150, 520, 80, 100, "Y");
    cvui::printf(frame, 160, 565, 0.6, 0xffffff,
                 "%0.2f",odom_data.pose.pose.position.y);
    // Square for  Z
    cvui::window(frame, 250, 520, 80, 100, "Z");
    cvui::printf(frame, 260, 565, 0.6, 0xffffff,
                 "%0.2f",odom_data.pose.pose.position.z);
    
/* SERVICE */
    
    cvui::window(frame, 50, 700, 300, 70, "Distance travelled");
    
    // Call the service
    if (cvui::button(frame, 50, 660, "Call Service")) {
      // Send the request and wait for a response
      if (service_client.call(srv_req)) {
        // Print the response message and return true
        ROS_DEBUG("Response message: %s", srv_req.response.message.c_str());
        // set latest service call status
        last_service_call_msg = srv_req.response.message;
        service_call_counter++;
      } else {
        last_service_call_msg = "Service call failed.";
        service_call_counter = 0;
      }
    }

    // Display the last response inside the window
    if (not last_service_call_msg.empty()) {
      
      cvui::printf(frame, 55, 740, 0.6, 0xff0000, "%s",
                   last_service_call_msg.c_str());
    }


/* UPDATE AND SHOW IN SCREEN */

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);


/* STOP THE PROGRAM */

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();

    }
}