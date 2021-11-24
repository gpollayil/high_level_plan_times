#!/usr/bin/env python

# ROSPY node for taking planning times with Franka
# Example in paper

import sys
import copy
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
import moveit_commander.conversions as conversions
from moveit_ros_planning_interface import _moveit_move_group_interface

print_details = False

robot_description = "robot_description"
ns = ""
wait_for_servers = 5.0

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4

home_joints = [0, -pi/4, 0, -pi/2, 0, pi/3, 0]


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
        move_group_interface = _moveit_move_group_interface.MoveGroupInterface(
            group_name, robot_description, ns, wait_for_servers)
        
        # Get get_end_effector_link
        end_effector = move_group_interface.get_end_effector_link()

        # Printout info 
        if print_details:
            display_info_fun(move_group, robot) 

        # Change planning algorithm (RRTkConfigDefault, RRTConnectkConfigDefault, RRTstarkConfigDefault)
        move_group.set_planner_id("RRTConnectkConfigDefault")
        print("The set planner is " + move_group.get_planner_id())

        # Plan and go to home joints
        plan_go_to_joint_fun(move_group, home_joints)

        # Plan and go to pose
        plan_go_to_pose_fun(move_group_interface, end_effector, pose_goal)

        # Plan and go to home joints
        plan_go_to_joint_fun(move_group, home_joints)

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
def plan_go_to_pose_fun(move_group_interface, end_effector, pose_goal):

    move_group_interface.set_pose_target(conversions.pose_to_list(pose_goal), end_effector)
    plan = moveit_msgs.msg.RobotTrajectory()

    start_time = time.time()
    plan.deserialize(move_group_interface.compute_plan())
    end_time = time.time()
    move_group_interface.execute(conversions.msg_to_string(plan))

    print("The function plan took " + str(end_time - start_time) + " to plan this!")
    # print("The motion planning algo took " + str(planned_time) + " to plan this!")
    # plan = move_group.go(wait=True)
    # Calling stop() ensures that there is no residual movement
    move_group_interface.stop()
    # It is always good to clear your targets after planning with poses
    move_group_interface.clear_pose_targets()

    return plan

def plan_go_to_joint_fun(move_group, joint_goal):

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling this ensures that there is no residual movement
    move_group.stop()


# MAIN
if __name__ == '__main__':
    print('Entered main!')
    main_fun()