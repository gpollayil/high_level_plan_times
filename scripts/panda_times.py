#!/usr/bin/env python

# ROSPY node for taking planning times with Franka
# Example in paper

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

print_details = False

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4

def main_fun():

    try:

        # Initialize moveit_commander and a rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('panda_times', anonymous=True)

        print('Built commander!')

        # This gives robot kinematic model and the robot current joint states
        robot = moveit_commander.RobotCommander()

        # This provides a remote interface for getting, setting, and updating the 
        # robots internal understanding of the surrounding world
        #scene = moveit_commander.PlanningSceneInterface()
        moveit_commander.PlanningSceneInterface()

        # This object is an interface to a planning group (group of joints)
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # To display trajectories in Rviz
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
            moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # Printout info 
        if print_details:
            display_info_fun(move_group, robot) 

        # Change planning algorithm
        move_group.set_planner_id("RRTkConfigDefault")
        print("The set planner is " + move_group.get_planner_id())

        # Plan and go to pose
        plan_traj_pose = plan_pose_fun(move_group, pose_goal) 

        print("Finished the node.")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


# Function for displaying info about move group and robot
def display_info_fun(move_group, robot):
    # Get the name of the reference frame for this robot
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # Print the name of the end-effector link for this group
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # Get a list of all the groups in the robot
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", group_names

    # For debugging it is useful to print the entire state of the robot
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""


# Function for planing to a pose
def plan_pose_fun(move_group, pose_goal):

    move_group.set_pose_target(pose_goal)
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses
    move_group.clear_pose_targets()

    return plan


# MAIN
if __name__ == '__main__':
    print('Entered main!')
    main_fun()