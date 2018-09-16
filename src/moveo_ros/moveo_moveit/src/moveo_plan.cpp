#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string PLANNING_GROUP("arm");
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("plane_origin");
    visual_tools.deleteAllMarkers();

    /*
    Eigen::Affine3d target_pose = Eigen::Affine3d::Identity();
    auto rotate = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());
    target_pose.translate(Eigen::Vector3d(0, 0.025, 0.17));
    target_pose.rotate(rotate * Eigen::AngleAxisd(-0.64, Eigen::Vector3d::UnitY()));
    */
    geometry_msgs::Pose object_pose;
    geometry_msgs::Pose grasp_pose, pre_grasp_approach, post_grasp_retreat;
    
    object_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    object_pose.position.x = 0;
    object_pose.position.y = 0.19;
    object_pose.position.z = 0;
    visual_tools.publishZArrow(object_pose);
    
    grasp_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -0.68, -M_PI/2);
    grasp_pose.position.x = 0;
    grasp_pose.position.y = 0.06;
    grasp_pose.position.z = 0.12;
    visual_tools.publishAxis(grasp_pose);
    visual_tools.trigger();

    pre_grasp_approach = grasp_pose;
    pre_grasp_approach.position.y -= 0.08;

    post_grasp_retreat = grasp_pose;
    post_grasp_retreat.position.z += 0.2;

    move_group.setPlanningTime(6.5);
    move_group.setPoseReferenceFrame("plane_origin");
    ROS_INFO("ee: %s", move_group.getEndEffectorLink().c_str());

    // start planning
    // move_group.setGoalTolerance
    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    move_group.setPoseTarget(pre_grasp_approach);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing pre_grasp_approach (pose goal) %s", success ? "" : "FAILED");
    move_group.move();
    ros::WallDuration(1.0).sleep();
    
    move_group.setPoseTarget(grasp_pose);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing grasp_pose (pose goal) %s", success ? "" : "FAILED");
    move_group.move();
    ros::WallDuration(1.0).sleep();

    move_group.setPoseTarget(post_grasp_retreat);
    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing post_grasp_retreat (pose goal) %s", success ? "" : "FAILED");
    move_group.move();
    
    ros::shutdown();
    return 0;
}
