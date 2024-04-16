#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include <iomanip>
#include <sstream>

class DistanceTracker {
public:
  DistanceTracker() {
    distance_ = 0.0;
    odom_sub_ = nh_.subscribe("odom", 1, &DistanceTracker::odomCallback, this);
    get_distance_service_ = nh_.advertiseService(
        "get_distance", &DistanceTracker::getDistanceServiceCallback, this);
    ROS_INFO("Distance tracker node initialized. Use: rosservice call "
             "/get_distance \"{}\" ");
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::ServiceServer get_distance_service_;
  double distance_;

  std::string formatFloatToString(float f) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << f;
    return out.str();
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // calculate distance traveled using Euclidean distance formula
    double dx = msg->pose.pose.position.x - prev_pose_.position.x;
    double dy = msg->pose.pose.position.y - prev_pose_.position.y;
    double dz = msg->pose.pose.position.z - prev_pose_.position.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);

    // add distance traveled to total distance
    distance_ += distance;

    // update previous pose
    prev_pose_ = msg->pose.pose;
  }

  bool getDistanceServiceCallback(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res) {
    res.success = true;
    // Respond with distance traveled in meters
    res.message = formatFloatToString(distance_);
    return true;
  }

  geometry_msgs::Pose prev_pose_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "distance_tracker");
  DistanceTracker tracker;
  ros::spin();
  return 0;
}
