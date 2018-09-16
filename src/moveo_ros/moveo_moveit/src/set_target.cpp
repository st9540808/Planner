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
    ros::init(argc, argv, "set_target");
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
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); i++) {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("Link_5");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
    ROS_INFO_STREAM(joint_model_group->getLinkModelNames().back() << "\n");


    // planing scence
    planning_scene::PlanningScene planning_scene(kinematic_model);
    moveit_visual_tools::MoveItVisualTools visual_tools("plane_origin");
    visual_tools.deleteAllMarkers();

    moveit::planning_interface::MoveGroupInterface move_group("arm");
    ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
    
    Eigen::Affine3d pose = Eigen::Affine3d::Identity(),
                    target_pose = Eigen::Affine3d::Identity();
    auto rotate = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());

    auto orientation =
        rotate * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    
    // target_pose.translate(Eigen::Vector3d(0, 0.03, 0.28));
    // target_pose.rotate(rotate * Eigen::AngleAxisd(0.925, Eigen::Vector3d::UnitY()));
    target_pose.translate(Eigen::Vector3d(x, y, z));
    target_pose.rotate(orientation);
    // pose.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    
    
    // target_pose.rotate((
    //         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    //         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
    //     ).inverse()
    // );
    // target_pose.translate(Eigen::Vector3d(0, 0, 0.08));
    // target_pose.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    
    
    // visual_tools.publishAxis(target_pose);
    // visual_tools.trigger();

    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setPlanningTime(0.75);
    move_group.setPoseReferenceFrame("plane_origin");
    move_group.setGoalOrientationTolerance(0.02);
    move_group.setGoalPositionTolerance(0.002);
    ROS_INFO("orientation tolerance: %f", move_group.getGoalOrientationTolerance());
    ROS_INFO("position tolerance: %f", move_group.getGoalPositionTolerance());
    
    auto trySolve = [&](double x_off, double y_off, double z_off) {
        bool success;
        Eigen::Affine3d test = Eigen::Affine3d::Identity();
        
        test.translate(Eigen::Vector3d(x+x_off, y+y_off, z+z_off));
        test.rotate(orientation);
        // test.rotate((
        //         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        //         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
        //     ).inverse()
        // );
        // test.translate(Eigen::Vector3d(0, 0, 0.08));
        // test.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
        // visual_tools.publishAxis(test);
        // visual_tools.trigger();

        move_group.setPoseTarget(test);
        success = (move_group.plan(plan) ==
                   moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            ROS_INFO("solution found: %f %f %f", x+x_off, y+y_off, z+z_off);
            return true;
        }
        return false;
    };
    for (int i = -40; i < 40; i++) {
        for (int j = -40; j < 40; j++) {
            if (trySolve(i*0.0002, j*0.0002, 0) == true) {
                int times = 0;
                for (; trySolve(i*0.0002, j*0.0002, 0) && times < 4; times++) ;
                if (times == 4) {
                    ROS_INFO("solution found for 4 consective times: %f %f %f", x+i*0.0002, y+j*0.0002, z+0);
                }
            }
        }
    }
    // ROS_INFO("Visualizing pre_grasp_approach (pose goal) %s", success ? "" : "FAILED");

    // auto trySolve = [&](double x, double y, double z) {
    //     bool success;
    //     target_pose.translate(Eigen::Vector3d(-0.135, -0.0625, 0.28));
    //     target_pose.rotate(
    //         rotate *
    //         Eigen::AngleAxisd(0.611 + x, Eigen::Vector3d::UnitX()) *
    //         Eigen::AngleAxisd(0.00 + y, Eigen::Vector3d::UnitY()) *
    //         Eigen::AngleAxisd(0.00 + z, Eigen::Vector3d::UnitZ())
    //     );
    //     move_group.setPoseTarget(target_pose);
    //     success = (move_group.plan(plan) ==
    //                moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     // ROS_INFO("Visualizing pre_grasp_approach (pose goal) %s", success ? "" : "FAILED");
    //     if (success) {
    //         ROS_INFO("solution found: %f %f %f", 0.611+x, y, z);
    //     }
    // };
    // for (int j = -50; j < 50; j++)
    //     for (int k = -50; k < 50; k++)
    //         trySolve(0, j*0.007, k*0.007);

    // move_group.move();
    // ros::WallDuration(1.0).sleep();

    ros::shutdown();
    return 0;
}