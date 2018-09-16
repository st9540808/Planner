#include <atomic>
#include <type_traits>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "moveo_moveit/ArmJointState.h"
#include "math.h"

std::atomic_bool joint_status(false);
moveo_moveit::ArmJointState arm_steps;
moveo_moveit::ArmJointState total;

// microsteps/revolution (using 16ths) from observation, for each motor
int stepsPerRevolution[6] = {32800, 18000, 72000, 3280, 14400, 0};
std::remove_reference<
    decltype(sensor_msgs::JointState::position[0])
>::type prev_angle[6];

// keep a running sum of all the step counts and use that as the final step to send to arduino accelstepper
// int angle_to_steps(double x)
// {
//   float steps;
//   steps=((x / M_PI)*stepsPerRevolution)+0.5; // (radians)*(1 revolution/PI radians)*(200 steps/revolution)
//   return steps;
// }

//command callback (for position) function
void cmd_cb(const sensor_msgs::JointState& cmd_arm)
{
    static volatile int times = 1;

    ROS_INFO_NAMED("test", "Received /move_group/fake_controller_joint_states %d", times);
    ROS_INFO_NAMED("test", "position: %f %f %f %f %f",
                   cmd_arm.position[0], cmd_arm.position[1], cmd_arm.position[2],
                   cmd_arm.position[3], cmd_arm.position[4]);

    // init
    if (times == 1) {
        for (int i = 0; i < 6; i++)
            prev_angle[i] = cmd_arm.position[i];
        
        total.position1 = 0;
        total.position2 = 0;
        total.position3 = 0;
        total.position4 = 0;
        total.position5 = 0;
        total.position6 = 0;
    }

    //compute relative step count to move each joint-- only works if all joint_angles start at 0
    //otherwise, we need to set the current command to the initial joint_angles
    //ROS_INFO_NAMED("test", "cmd_arm.position[4]: %f, prev_angle[4]: %f, init_angle[4]: %f", cmd_arm.position[4], prev_angle[4], init_angle[4]);
    //ROS_INFO_NAMED("test", "arm_steps.position5 #1: %f", (cmd_arm.position[4]-prev_angle[4])*stepsPerRevolution[4]/M_PI);
    arm_steps.position1 = (int)((cmd_arm.position[0] - prev_angle[0]) * stepsPerRevolution[0] / (2. * M_PI));
    arm_steps.position2 = (int)((cmd_arm.position[1] - prev_angle[1]) * stepsPerRevolution[1] / (2. * M_PI));
    arm_steps.position3 = (int)((cmd_arm.position[2] - prev_angle[2]) * stepsPerRevolution[2] / (2. * M_PI));
    arm_steps.position4 = (int)((cmd_arm.position[3] - prev_angle[3]) * stepsPerRevolution[3] / (2. * M_PI));
    arm_steps.position5 = (int)((cmd_arm.position[4] - prev_angle[4]) * stepsPerRevolution[4] / (2. * M_PI));
    arm_steps.position6 = (int)((cmd_arm.position[5] - prev_angle[5]) * stepsPerRevolution[5] / (2. * M_PI));

    for (int i = 0; i < 6; i++)
        prev_angle[i] = cmd_arm.position[i];

    //total steps taken to get to goal
    total.position1 += arm_steps.position1;
    total.position2 += arm_steps.position2;
    total.position3 += arm_steps.position3;
    total.position4 += arm_steps.position4;
    total.position5 += arm_steps.position5;

    ROS_INFO_NAMED("test", "total: %d %d %d %d %d %d\n",
                   total.position1, total.position2, total.position3,
                   total.position4, total.position5, total.position6);

    if (times > 1) joint_status.store(true);
    times++;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "moveo_moveit");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("In main function");
    ros::Subscriber sub = nh.subscribe("/move_group/fake_controller_joint_states", 1000, cmd_cb);
    ros::Publisher pub = nh.advertise<moveo_moveit::ArmJointState>("joint_steps", 50);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        if (joint_status.load()) {
            joint_status.store(false);
            pub.publish(total);
            // pub.publish(arm_steps);
            // ROS_INFO_STREAM("Published to /joint_steps");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    //ros::spin();
    return 0;
}
