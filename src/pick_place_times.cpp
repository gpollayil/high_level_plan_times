// This is to get motionplanning times for pick and place actions
// with robots (Franka or UR10e) 
// REFERENCE: 
// https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/motion_planning_api/src/motion_planning_api_tutorial.cpp

#include <sstream>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

// Custom
#include "high_level_plan_times/params_for_plan.h"


// Aux function for building a pose MotionPlanRequest
planning_interface::MotionPlanRequest build_pose_motion_plan_req(geometry_msgs::PoseStamped pose,
    std::vector<double> tolerance_pose, std::vector<double> tolerance_angle, 
    const std::string PLANNING_GROUP, std::string ee_link_name){

    #ifdef DEBUG
    ROS_INFO("Starting to building a pose MotionPlanRequest.");
    #endif

    moveit_msgs::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints(ee_link_name, pose, tolerance_pose, tolerance_angle);

    planning_interface::MotionPlanRequest req;
    req.group_name = PLANNING_GROUP;
    req.planner_id = PLANNER_ID;
    req.allowed_planning_time = MAX_PLANNING_TIME;
    req.goal_constraints.push_back(pose_goal);

    return req;
}

// Aux function for building a joint MotionPlanRequest
planning_interface::MotionPlanRequest build_joint_motion_plan_req(std::vector<double> joint_values,
    const robot_state::JointModelGroup* joint_model_group, robot_model::RobotModelPtr robot_model, 
    const std::string PLANNING_GROUP){

    #ifdef DEBUG
    ROS_INFO("Starting to building a joint MotionPlanRequest.");
    #endif

    robot_state::RobotState goal_state(robot_model);
    goal_state.setJointGroupPositions(joint_model_group, joint_values);
    moveit_msgs::Constraints joint_goal = 
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

    planning_interface::MotionPlanRequest req;
    req.group_name = PLANNING_GROUP;
    req.planner_id = PLANNER_ID;
    req.allowed_planning_time = MAX_PLANNING_TIME;
    req.goal_constraints.push_back(joint_goal);

    return req;
}


// Aux function for planning using Planning Context
planning_interface::MotionPlanResponse plan_for_goal(planning_interface::PlannerManagerPtr planner_instance,
    planning_scene::PlanningScenePtr planning_scene, planning_interface::MotionPlanRequest req){

    #ifdef DEBUG
    ROS_INFO("Starting to plan using Planning Context.");
    #endif

    planning_interface::MotionPlanResponse res;
    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS){
        ROS_ERROR("Could not compute plan successfully");
    }

    ROS_INFO_STREAM("The planning time is " << res.planning_time_ );

    return res;

}

// Aux function to display the trajectory
void display_trajectory_visual(ros::Publisher display_publisher, const robot_state::JointModelGroup* joint_model_group,
    moveit_visual_tools::MoveItVisualTools visual_tools, planning_interface::MotionPlanResponse res){

    #ifdef DEBUG
    ROS_INFO("Starting to display the trajectory.");
    #endif

    moveit_msgs::DisplayTrajectory display_trajectory;

    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory);

}

// Aux function to set the state in planning scene and see in rviz
void set_state_planning_scene(const robot_state::JointModelGroup* joint_model_group, robot_state::RobotStatePtr robot_state, 
    planning_scene::PlanningScenePtr planning_scene, moveit_visual_tools::MoveItVisualTools visual_tools,
    planning_interface::MotionPlanResponse res){
    
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst());

}


// Main
int main(int argc, char** argv) {

    const std::string node_name = "pick_place_times_node";
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    // Assigning the params
    assign_params();

    /* MAIN SETTING UP */

    // Defining planning group name and getting robot model
    robot_model_loader::RobotModelLoader robot_model_loader("/robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    // Using the :moveit_core:`RobotModel`, we can construct a :planning_scene:`PlanningScene`
    // that maintains the state of the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // Configure a valid robot state
    planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;

    // We will get the name of planning plugin we want to load
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    std::string planner_plugin_name;
    if (!node_handle.getParam("/move_group/planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    } catch (pluginlib::PluginlibException& ex) {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
    }


    /* LOADING VISUALIZATION PART */

    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(BASE_LINK);
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();

    /* Remote control is an introspection tool that allows users to step through a high level script
    via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    // Publisher to visualize the trajectory
    ros::Publisher display_publisher = 
        node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);


    #ifdef DEBUG
    ROS_INFO("Starting to do the pose plannings.");
    #endif

    /* PLANNING AND EXECUTING */

    planning_interface::MotionPlanRequest plan_req;
    planning_interface::MotionPlanResponse plan_res;

    planning_times_single_it.clear();

    // First pose
    plan_req = build_pose_motion_plan_req(first_pose, tolerance_pose, tolerance_angle,
        PLANNING_GROUP, EE_LINK);

    plan_res = plan_for_goal(planner_instance, planning_scene, plan_req);
    planning_times_single_it.push_back(boost::lexical_cast<std::string>(plan_res.planning_time_));

    display_trajectory_visual(display_publisher, joint_model_group, visual_tools, plan_res);
    set_state_planning_scene(joint_model_group, robot_state, planning_scene, visual_tools, plan_res);

    // Second joints
    plan_req = build_joint_motion_plan_req(second_joints, joint_model_group, robot_model, PLANNING_GROUP);

    plan_res = plan_for_goal(planner_instance, planning_scene, plan_req);

    display_trajectory_visual(display_publisher, joint_model_group, visual_tools, plan_res);
    set_state_planning_scene(joint_model_group, robot_state, planning_scene, visual_tools, plan_res);


}