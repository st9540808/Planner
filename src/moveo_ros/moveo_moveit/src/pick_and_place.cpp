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

#include "gripper.h"

std::vector<double> pre_grasp_approach2 { -0.034241, 0.937831, -0.321798, -0.412780, 1.2148 };
std::vector<double> pre_grasp_upper_approach { -0.034597, 0.421936, -1.479184, -0.412780, 1.584228 };
std::vector<double> pre_grasp_approach { -0.034241, -0.481577, -1.421798, -0.412780, 2.476590 };
std::vector<double> grasp { -0.034241, -0.550373, -1.169543, -0.414351, 2.293121 };
std::vector<double> post_grasp_retreat { -0.034241, 0.217699, -1.123679, -0.424089, 1.547936 };
std::vector<double> pre_place_appraoch { 0.505034, -0.137593, -1.146611, -0.424089, 1.880316 };
std::vector<double> place { 0.505034, -0.653411, -0.722208, -0.424089, 1.973675 };
std::vector<double> post_place_retreat { 0.505034, -0.538750, -1.318446, -0.412780, 2.442347 };
std::vector<double> post_place_retreat2 { 0.503411, 0.137593, -1.490594, -0.412780, 1.993675 };

// Eigen::Vector3d(0.000619158, -0.0496657, 0.105)   rotate * Eigen::AngleAxisd(0.91, Eigen::Vector3d::UnitY())
std::vector<double> pre_grasp_approach_ik { -0.029413, -0.494959, -1.362465, -0.411424, 2.400043 };
// _x:=0 _y:=-0.01 _z:=0.105  _roll:=0.0 _pitch:=0.923 _yaw:=0
std::vector<double> grasp_ik { -0.030245, -0.527847, -1.165259, -0.411424, 2.232265 };
std::vector<double> post_grasp_retreat_ik { -0.033135, 0.276858, -1.269621, -0.411424, 1.543505 };

std::vector<double> place_ik { 0.500156, -0.592920, -0.925649, -0.411424, 2.089116 };
std::vector<double> post_place_retreat_ik { 0.499571, -0.548320, -1.060053, -0.411424, 2.168312 };
std::vector<double> post_place_retreat2_ik { 0.504354, 0.021069, -1.196794, -0.411424, 1.730076 };

std::vector<std::vector<double>> reverseOperation {
    post_place_retreat2_ik,
    post_place_retreat_ik,
    place_ik,
    post_grasp_retreat_ik,
    grasp_ik,
    pre_grasp_approach_ik,
    pre_grasp_upper_approach
};


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface&
                         planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "plane_origin";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.15;
    collision_objects[0].primitives[0].dimensions[1] = 0.15;
    collision_objects[0].primitives[0].dimensions[2] = 0.0815;
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0.19;
    collision_objects[0].primitive_poses[0].position.z = -0.04075;
    collision_objects[0].operation = collision_objects[0].ADD;

    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "plane_origin";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.15;
    collision_objects[1].primitives[0].dimensions[1] = 0.15;
    collision_objects[1].primitives[0].dimensions[2] = 0.0815;
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = -0.4;
    collision_objects[1].primitive_poses[0].position.y = -0.2;
    collision_objects[1].primitive_poses[0].position.z = -0.04075;
    collision_objects[1].operation = collision_objects[0].ADD;

    collision_objects[2].header.frame_id = "plane_origin";
    collision_objects[2].id = "object";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.045;
    collision_objects[2].primitives[0].dimensions[1] = 0.045;
    collision_objects[2].primitives[0].dimensions[2] = 0.1;
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0;
    collision_objects[2].primitive_poses[0].position.y = 0.19;
    collision_objects[2].primitive_poses[0].position.z = 0.05;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);
    
    grasps[0].grasp_pose.header.frame_id = "plane_origin";
    grasps[0].grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -0.6, -M_PI/2);
    grasps[0].grasp_pose.pose.position.x = 0;
    grasps[0].grasp_pose.pose.position.y = 0.06;
    grasps[0].grasp_pose.pose.position.z = 0.12;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "plane_origin";
    grasps[0].pre_grasp_approach.direction.vector.y = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "plane_origin";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    move_group.setSupportSurfaceName("table1");
    move_group.pick("object", grasps);
}

void workflow(moveit::planning_interface::MoveGroupInterface& move_group)
{
    gripperControl gripper;

    ros::WallDuration wait(1.0);
    moveit::planning_interface::MoveItErrorCode ret;
    move_group.setPlanningTime(4.5);
    gripper.setAngle(0);

    moveit::core::RobotStatePtr currState = move_group.getCurrentState();
    
    move_group.setJointValueTarget(pre_grasp_approach2);
    ret = move_group.move();
    if (ret == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
        goto end;

    move_group.setJointValueTarget(pre_grasp_approach);
    ret = move_group.move();
    if (ret == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
        goto end;
    wait.sleep();

    move_group.setJointValueTarget(grasp);
    move_group.move();
    if (ret == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
        goto end;
    wait.sleep();
    gripper.setAngle(65);

    move_group.setJointValueTarget(post_grasp_retreat);
    move_group.move();
    if (ret == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
        goto end;
    wait.sleep();

    move_group.setJointValueTarget(pre_place_appraoch);
    move_group.move();
    if (ret == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
        goto end;
    wait.sleep();

    move_group.setJointValueTarget(place);
    move_group.move();
    if (ret == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
        goto end;
    wait.sleep();
    gripper.setAngle(0);

    move_group.setJointValueTarget(post_place_retreat);
    move_group.move();
    if (ret == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
        goto end;
    wait.sleep();    

    move_group.setJointValueTarget(post_place_retreat2);
    move_group.move();
    if (ret == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
        goto end;
    wait.sleep();    

    move_group.setNamedTarget("Upright");
    move_group.move();
    if (ret == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
        goto end;
    wait.sleep();    

end:
    // ros::shutdown();
    if (ret == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
        ROS_ERROR("preempted");
}

int workflow_ik(moveit::planning_interface::MoveGroupInterface& move_group)
{
    gripperControl gripper;

    ros::WallDuration wait(0.5);
    moveit::planning_interface::MoveItErrorCode ret;
    move_group.setPlanningTime(4.5);
    gripper.setAngle(0);

    moveit::core::RobotStatePtr currState = move_group.getCurrentState();
    
    move_group.setJointValueTarget(pre_grasp_upper_approach);
    ret = move_group.move();
    ROS_INFO("return value: %d", ret);
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;

    move_group.setJointValueTarget(pre_grasp_approach_ik);
    ret = move_group.move();
    ROS_INFO("return value: %d", ret);
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;
    wait.sleep();

    move_group.setJointValueTarget(grasp_ik);
    ret = move_group.move();
    ROS_INFO("return value: %d", ret);
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;
    wait.sleep();
    gripper.setAngle(65);

    move_group.setJointValueTarget(post_grasp_retreat_ik);
    ret = move_group.move();
    ROS_INFO("return value: %d", ret);
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;
    wait.sleep();

    move_group.setJointValueTarget(place_ik);
    ret = move_group.move();
    ROS_INFO("return value: %d", ret);
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;
    wait.sleep();
    gripper.setAngle(0);

    move_group.setJointValueTarget(post_place_retreat_ik);
    ret = move_group.move();
    ROS_INFO("return value: %d", ret);
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;
    wait.sleep();    

    move_group.setJointValueTarget(post_place_retreat2_ik);
    ret = move_group.move();
    ROS_INFO("return value: %d", ret);
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;
    wait.sleep();    

    move_group.setNamedTarget("Upright");
    ret = move_group.move();
    ROS_INFO("return value: %d", ret);
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;
    wait.sleep();    

end:
    // ros::shutdown();
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("preempted");
        std::exit(1);
    }
}

void workflow_reverse(moveit::planning_interface::MoveGroupInterface& move_group)
{
    gripperControl gripper;

    ros::WallDuration wait(0.5);
    moveit::planning_interface::MoveItErrorCode ret;
    move_group.setPlanningTime(4.5);
    gripper.setAngle(0);

    for (int i = 0; i < 3; i++) {
        move_group.setJointValueTarget(reverseOperation[i]);
        ret = move_group.move();
        if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
            goto end;
        wait.sleep();
    }

    gripper.setAngle(65);
    
    move_group.setJointValueTarget(reverseOperation[3]);
    ret = move_group.move();
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;
    move_group.setJointValueTarget(reverseOperation[4]);
    ret = move_group.move();
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;

    gripper.setAngle(0);

    move_group.setJointValueTarget(reverseOperation[5]);
    ret = move_group.move();
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;
    move_group.setJointValueTarget(reverseOperation[6]);
    ret = move_group.move();
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;

    move_group.setNamedTarget("Upright");
    ret = move_group.move();
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        goto end;
    wait.sleep(); 

end:
    // ros::shutdown();
    if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR("preempted");
        std::exit(1);
    }
}


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "moveo_arm_pick_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("arm");
    
    workflow_ik(move_group);
    ros::Duration(0.2).sleep();
    workflow_reverse(move_group);

    ros::shutdown();
    return 0;
}
