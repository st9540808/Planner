#include "gripper.h"

gripperControl::gripperControl(void)
    : mPub(mNh.advertise<std_msgs::UInt16>("/gripper/angle", 10))
{
        ros::Duration(1.).sleep();
}

void gripperControl::setAngle(uint16_t angle)
{
    std_msgs::UInt16 msg;
    
    msg.data = angle;
    mPub.publish(msg);
    ros::Duration(1.5).sleep();
}
