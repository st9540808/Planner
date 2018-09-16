#include <vector>
#include "ros/ros.h"
#include "moveo_moveit/ArmJointState.h"
#include "math.h"

moveo_moveit::ArmJointState arm_steps;
moveo_moveit::ArmJointState total;
std::vector<std::vector<int>> joint_steps_queue = {
    {7, 7, -16, 9, 4, 0},
    {64},
    {121},
    {186},
    {326},
    {502},
    {718},
    {970},
    {1257},
    {1577},
    {1928},
    {2306},
    {2711},
    {3124},
    {3537},
    {3950},
    {4363},
    {4776},
    {5189},
    {5602},
    {6001},
    {6716},
    {7027},
    {7304},
    {7549},
    {7753},
    {7918},
    {8041},
    {8098},
    {8155},
    {8194},
    {0}
};

int main(int argc, char **argv)
{
    ROS_INFO("In main function");

    ros::init(argc, argv, "moveo_motor");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<moveo_moveit::ArmJointState>("joint_steps", 50);

    ros::Rate loop_rate(10);

    if (ros::ok()) {
        sleep(1);
        for (const auto& joint_steps : joint_steps_queue) {
            total.position1 = joint_steps[0];
            ROS_INFO("publish: %d", total.position1);
            pub.publish(total);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    ros::shutdown();
    return 0;
}
