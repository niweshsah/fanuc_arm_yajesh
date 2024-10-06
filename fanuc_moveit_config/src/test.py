#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def set_end_effector_position(x, y, z):
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Set pose target for a specific link (link3)
    target_pose1 = geometry_msgs.msg.Pose()
    target_pose1.orientation.w = 1.0  # Default orientation, modify as needed
    target_pose1.position.x = x
    target_pose1.position.y = y
    target_pose1.position.z = z
    move_group.set_pose_reference_frame("base_link")
    move_group.set_end_effector_link("link_6")
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_goal_joint_tolerance(0.01)
    move_group.set_num_planning_attempts(10)
    move_group.set_planning_time(50)
    # move_group.set_approximate_joint_value_target(target_pose1, "link_6")
    # Setting a precise pose target
    # move_group.set_pose_target(target_pose1, "link_6")

    # Or you can set an approximate joint value target
    # move_group.set_approximate_joint_value_target(target_pose1, "link_6")

    # Plan and execute
    success, plan, _, _ = move_group.plan()

    if success:
        rospy.loginfo("Planning successful! Executing the plan...")
        move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
    else:
        rospy.logwarn("Planning failed.")

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    set_end_effector_position(0.3, 0.0, 0.5)  # Example target position
