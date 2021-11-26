// Here all the parameters for planning are defined

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#define DEBUG
// #define RVIZ_TRAJ_LINE

// Main constants
const std::string OUTPUT_FILENAME = "plan_times";
const std::string ROBOT_NAME = "franka";            // Attention! Just used for the output filename (franka, UR)
const int NUM_IT = 10;                              // Number of iterations for set of actions

// Moveit params
const std::string PLANNING_GROUP = "panda_arm";     // (panda_arm, ???)
const std::string BASE_LINK = "panda_link0";
const std::string EE_LINK = "panda_link8";
const std::string PLANNER_ID = "RRTkConfigDefault"; // (RRTkConfigDefault, RRTConnectkConfigDefault, RRTstarkConfigDefault)
const double MAX_PLANNING_TIME = 5.0;

// Tolerances
std::vector<double> tolerance_pose(3, 0.01);
std::vector<double> tolerance_angle(3, 0.01);

// First Pose
// geometry_msgs::PoseStamped first_pose;

// void assign_params(){

//     // Assigning values to first_pose
//     first_pose.header.frame_id = BASE_LINK;
//     first_pose.pose.position.x = 0.3;
//     first_pose.pose.position.y = 0.4;
//     first_pose.pose.position.z = 0.75;
//     first_pose.pose.orientation.w = 1.0;

// }

// Joint Configs for Franka
std::vector<double> home_joints = {0.9801277471505729, -1.2118281784726863, -0.8287402532068979, 
    -2.4747393955096864, 0.6757921851841364, 2.0183060749504302, 0.227496828169762};

std::vector<double> joints_1 = {0.21456287496505022, 0.5249454632474256, -0.34124799590790816,
    -1.7054843924788552, -0.2914662263796135, 1.8465625882413648, 0.1931950980529690};

std::vector<double> joints_2 = {0.2144587004080274, 0.7856749983412846, -0.3472224124820871, 
    -1.6927943650955561, -0.2905159043106768, 2.2015896898013114, 0.22034951782226556};

std::vector<double> joints_3 = {0.21456287496505022, 0.5249454632474256, -0.34124799590790816, 
    -1.7054843924788552, -0.2914662263796135, 1.8465625882413648, 0.19319509805296906};

std::vector<double> joints_4 = {0.431987905920901, 0.44295161317798065, -0.982301910915018, 
    -2.5692466450406792, 0.9404757073984309, 2.2948205383883584, 0.08062765405741003};

std::vector<double> joints_5 = {0.1722585239494056, -0.6117508801577384, -0.8719179760394334, 
    -2.463224751688279, -0.3867183050126941, 2.0400619047026485, 1.1205479997722625};

std::vector<double> joints_6 = {};

// Joint Configs for UR
// std::vector<double> home_joints = {};

// std::vector<double> joints_1 = {};

// std::vector<double> joints_2 = {};

// std::vector<double> joints_3 = {};

// std::vector<double> joints_4 = {};

// std::vector<double> joints_5 = {};

// std::vector<double> joints_6 = {};

// Vector of strings with the planning times for each iteration
std::vector<std::string> planning_times_single_it;
