#ifndef GRIPPER_H
#define GRIPPER_H

#include <ros/ros.h>
#include <std_msgs/UInt16.h>

class gripperControl {
    ros::NodeHandle mNh;
    ros::Publisher mPub;

public:
    gripperControl(void);
    void setAngle(uint16_t angle);
};

#endif
