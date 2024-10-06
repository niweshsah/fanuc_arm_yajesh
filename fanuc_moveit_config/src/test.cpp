#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

void setEndEffectorPosition(double x, double y, double z) {
    // Initialize the ROS node
    ros::NodeHandle nh;

    // Create a MoveGroupInterface for the manipulator
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Set the reference frame for pose targets
    move_group.setPoseReferenceFrame("base_link");

    // Set the end effector link
    move_group.setEndEffectorLink("link_6");

    // Set tolerances for goal position, orientation, and joint values
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.01);
    move_group.setGoalJointTolerance(0.01);
    
    // Set the number of planning attempts and planning time
    move_group.setNumPlanningAttempts(10);
    move_group.setPlanningTime(20.0);
    move_group.setPlannerId("RRTConnect"); // Use a valid planner ID

    // Create a target pose
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;  // Default orientation, modify as needed
    target_pose1.position.x = x;
    target_pose1.position.y = y;
    target_pose1.position.z = z;

    // Log the current joint state and target pose
    ROS_INFO_STREAM("Current joint values: " << move_group.getCurrentJointValues());
    ROS_INFO_STREAM("Target pose: " << target_pose1);

    // Set the pose target for the end effector
    move_group.setPoseTarget(target_pose1, "link_6");

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode error_code = move_group.plan(plan);

    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Planning successful! Executing the plan...");
        move_group.execute(plan);
        move_group.stop();  // Ensure no residual movement
        move_group.clearPoseTargets();  // Clear targets
    } else {
        ROS_WARN("Planning failed with error code: %s", error_code.val);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_group_cpp_interface");
    ros::AsyncSpinner spinner(1);  // Use a spinner with one thread
    spinner.start();

    // Example target position
    setEndEffectorPosition(1.0, 0.0, 0.0);  // Modify as needed

    ros::shutdown();
    return 0;
}
