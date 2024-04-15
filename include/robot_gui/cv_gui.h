#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include "robot_gui/cvui.h"
#include "ros/rate.h"
#include "std_srvs/Trigger.h"

#define WINDOW_NAME "CVUI Test"
class CV{
    public:
        CV(ros::NodeHandle *nnh, robotinfo_msgs::RobotInfo10Fields *rb_data, geometry_msgs::Twist *vel_data,nav_msgs::Odometry *od_data);
        void run();
    protected:
        ros::NodeHandle *nn;
    private:
        robotinfo_msgs::RobotInfo10Fields *rb_da;
        geometry_msgs::Twist *vel_da;
        nav_msgs::Odometry *od_da;
        geometry_msgs::Twist c_vl;
        std::string dd = "hhh";
        ros::Publisher od_pub;
        ros::ServiceClient gd_service ;
        std_srvs::Trigger srv;
        std::string rc;
};
CV::CV(ros::NodeHandle *nnh ,robotinfo_msgs::RobotInfo10Fields *rb_data,geometry_msgs::Twist *vel_data, nav_msgs::Odometry *od_data){
    this -> rb_da = rb_data;
    this -> vel_da = vel_data;
    this -> od_da = od_data;
    this ->nn = nnh;
    c_vl.linear.x =0;
    c_vl.angular.z =0;
    od_pub = this -> nn ->advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    gd_service = this -> nn ->serviceClient<std_srvs::Trigger>("/get_distance");
}
void CV::run() {
  cv::Mat frame = cv::Mat(1000, 350, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);
//Info
    // Create window at (40, 30) with size 250x180 (width x height) and title
    cvui::window(frame, 40, 30, 250, 220, "Info: " );
    //print info 
    cvui::printf(frame, 45, 50, 0.4, 0xffffff, rb_da->data_field_01.c_str());
    cvui::printf(frame, 45, 70, 0.4, 0xffffff, rb_da->data_field_02.c_str());
    cvui::printf(frame, 45, 90, 0.4, 0xffffff, rb_da->data_field_03.c_str());
    cvui::printf(frame, 45, 110, 0.4, 0xffffff, rb_da->data_field_04.c_str());
    cvui::printf(frame, 45, 130, 0.4, 0xffffff, rb_da->data_field_05.c_str());
    cvui::printf(frame, 45, 150, 0.4, 0xffffff, rb_da->data_field_06.c_str());
    cvui::printf(frame, 45, 170, 0.4, 0xffffff, rb_da->data_field_07.c_str());
    cvui::printf(frame, 45, 190, 0.4, 0xffffff, rb_da->data_field_08.c_str());
    cvui::printf(frame, 45, 210, 0.4, 0xffffff, rb_da->data_field_09.c_str());
    cvui::printf(frame, 45, 230, 0.4, 0xffffff, rb_da->data_field_10.c_str());
//Velocity Print
    cvui::window(frame, 25, 460, 140, 40, "Linear Velocity: " );   
    cvui::window(frame, 170, 460, 140, 40, "Angular Velocity " );   
    //print velocity
    cvui::printf(frame, 50, 485, 0.4, 0xff0000,"%0.2f m/sec" ,vel_da->linear.x);
    cvui::printf(frame, 190, 485, 0.4, 0xff0000,"%0.2f m/sec" ,vel_da->angular.z); 

//Velocity pub button
    if (cvui::button(frame, 125, 260, 80,60 ," Forward ")) {
      //The button was clicked, update the Twist message
        if(c_vl.linear.x<0 || c_vl.angular.z != 0 ) {c_vl.linear.x = 0; c_vl.angular.z =0; };
        c_vl.linear.x += 0.05;
    }
    if (cvui::button(frame, 125, 325, 80,60 ," Stop ")) {
      // The button was clicked, update the Twist message
        c_vl.linear.x = 0;
        c_vl.angular.z =0;
    }
     if (cvui::button(frame, 210, 325, 80,60 ," Right ")) {
        if(c_vl.linear.x <0){
            c_vl.linear.x -= 0.1;
            c_vl.angular.z -= 0.1;
        }
        else{
        c_vl.linear.x += 0.1;
        c_vl.angular.z -= 0.1;}
    }
     if (cvui::button(frame, 40, 325, 80 , 60 ," Left ")) {
       
        if(c_vl.linear.x <0){
            c_vl.linear.x -= 0.1;
            c_vl.angular.z += 0.1;
        }else {
            c_vl.linear.x += 0.1;
            c_vl.angular.z += 0.1;
            }
    }
    if (cvui::button(frame, 125, 390, 80 , 60 ," Back ")) {
        if(c_vl.linear.x>0 || c_vl.angular.z != 0) {c_vl.linear.x = 0; c_vl.angular.z =0; };
         c_vl.linear.x -= 0.05;
    }
    od_pub.publish(c_vl);

    
// Estimated position
    cvui::printf(frame, 25, 510, 0.4, 0xffffff,"Estimated robot position based off odometry");  
    cvui::window(frame, 25, 530, 100, 100, "X:" );   
    cvui::window(frame, 130, 530, 100, 100, "Y:" ); 
    cvui::window(frame, 235, 530, 100, 100, "Z:" ); 

    cvui::printf(frame, 50, 590, 1.0, 0xffffff,"%0.1f",od_da->pose.pose.position.x);  
    cvui::printf(frame, 160, 590, 1.0, 0xffffff,"%0.1f",od_da->pose.pose.position.y);  
    cvui::printf(frame, 260, 590, 1.0, 0xffffff,"%0.1f",od_da->pose.pose.position.z);   
// Distance Traveled call
    cvui::printf(frame, 25, 635, 0.4, 0xffffff,"Distance Traveled");  

    // cvui::window(frame, 235, 560, 100, 100, "Z:" ); 
    cvui::window(frame, 135, 650, 200, 100, "Distance in meters" ); 
    if (cvui::button(frame, 25, 650, 100,100 ," Call ")) {
        gd_service.call(srv);
        rc = srv.response.message; 
    }
    cvui::printf(frame, 240, 690, 1.0, 0xffffff,rc.c_str());

// Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
  }
}