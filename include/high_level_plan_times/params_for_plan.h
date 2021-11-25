// Here all the parameters for planning are defined

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#define DEBUG

// Moveit group name
const std::string PLANNING_GROUP = "panda_arm";
const std::string BASE_LINK = "panda_link0";
const std::string EE_LINK = "panda_link8";

// Tolerances
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);

// First Pose
geometry_msgs::PoseStamped first_pose;

void assign_params(){

    // Assigning values to first_pose
    first_pose.header.frame_id = BASE_LINK;
    first_pose.pose.position.x = 0.3;
    first_pose.pose.position.y = 0.4;
    first_pose.pose.position.z = 0.75;
    first_pose.pose.orientation.w = 1.0;

}
