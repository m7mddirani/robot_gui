#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <robotinfo_msgs/RobotInfo10Fields.h>
#include "robot_gui/cvui.h"
#include "std_srvs/Trigger.h"

template<typename rosmsg>
class Guisubscribe{
    public:
        Guisubscribe(ros::NodeHandle &noh,const std::string topic_name);
        ~Guisubscribe();
        // Guisubscribe(ros::NodeHandle *noh,const std::string sub_topic_name, const std::string pub_topic_name){};
        robotinfo_msgs::RobotInfo10Fields rb_data;
        nav_msgs::Odometry od_data;
        geometry_msgs::Twist vel_data;

    protected:
        ros::NodeHandle *nh;

    private:
        std::string topic_name;
        ros::Subscriber sub;
        ros::Publisher pub;
        void callbacktopic(const typename rosmsg::ConstPtr &msg);
        // void display(std::string topic_name);
};
template<typename rosmsg>
Guisubscribe<rosmsg>::Guisubscribe(ros::NodeHandle &noh,const std::string topic_name){
    this->nh = &noh;
    this->topic_name = topic_name;
    sub = this->nh-> subscribe(this->topic_name, 10, &Guisubscribe<rosmsg>::callbacktopic, this);
};

template<typename rosmsg>
Guisubscribe<rosmsg>::~Guisubscribe(){
    std::cout << "Guisubscribe Destructor is called" << std::endl;
};

// template<typename rosmsg> 
// void Guisubscribe<rosmsg>::callbacktopic(const typename rosmsg::ConstPtr &msg){
// };

template<> 
inline void Guisubscribe<robotinfo_msgs::RobotInfo10Fields>::callbacktopic(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg){
    rb_data = *msg;
    
};


template<> 
inline void Guisubscribe<nav_msgs::Odometry>::callbacktopic(const nav_msgs::Odometry::ConstPtr &msg){
    od_data =*msg;

};

template<> 
inline void Guisubscribe<geometry_msgs::Twist>::callbacktopic(const geometry_msgs::Twist::ConstPtr &msg){
    vel_data =*msg;
};