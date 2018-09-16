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
    ros::init(argc, argv, "set_one_target");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh("~");
    double x, y, z, roll, pitch, yaw;
    nh.getParam("x", x);
    nh.getParam("y", y);
    nh.getParam("z", z);
    nh.getParam("roll", roll);
    nh.getParam("pitch", pitch);
    nh.getParam("yaw", yaw);
    ROS_WARN("x=%f y=%f z=%f  roll=%f pitch=%f yaw=%f", x, y, z, roll, pitch, yaw);


    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    std::vector<double> joint_values;
    kinematic_state->setToDefaultValues();
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    
    
    planning_scene::PlanningScene planning_scene(kinematic_model);
    moveit_visual_tools::MoveItVisualTools visual_tools("plane_origin");
    
    // planing scence
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    robot_state::RobotStatePtr currState = move_group.getCurrentState();

    ROS_INFO("INITIAL");
    currState->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); i++) {
        ROS_INFO("%s: %f", joint_names[i].c_str(), joint_values[i]);
    }
    

    auto rotate = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());
    Eigen::Affine3d target_pose = Eigen::Affine3d::Identity(), I = Eigen::Affine3d::Identity();

    target_pose.translate(Eigen::Vector3d(x, y, z));
    I.translate(Eigen::Vector3d(0, 0, 0.02));
    target_pose.rotate(rotate * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));

    auto state = move_group.getCurrentState();
    auto trans = state->getFrameTransform("Link_5");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxis(target_pose);
    visual_tools.trigger();
    
    move_group.setPoseReferenceFrame("plane_origin");
    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    move_group.setNumPlanningAttempts(2);
    move_group.setPlanningTime(4);
    move_group.setGoalOrientationTolerance(0.05);
    move_group.setGoalPositionTolerance(0.002);
   
    // for (int i = -10; i < 10; i++) {
    //     for (int j = -10; j < 10; j++) {
    //         Eigen::Affine3d target_pose = Eigen::Affine3d::Identity();
    //         target_pose.translate(Eigen::Vector3d(x+i*0002, y+j*0.002, z));
    //         target_pose.rotate(rotate * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    //         move_group.setPoseTarget(target_pose);
    //         bool success = (move_group.plan(plan) ==
    //                         moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //         if (success) {
    //             ROS_INFO("found %d %d", i, j);
    //         }
    //     }
    // }

     
    move_group.setPoseTarget(target_pose);
    bool success = (move_group.plan(plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing pre_grasp_approach (pose goal) %s", success ? "" : "FAILED");
    if (success) {
        ROS_INFO("target pose:");
        move_group.move();
        currState = move_group.getCurrentState();
        currState->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); i++) {
            ROS_INFO("%s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }

    ros::shutdown();
    return 0;
}
