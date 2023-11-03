#include "ros/ros.h"
#include "std_msgs/UInt16.h"

ros::Publisher servoPub; // Declare servoPub as a global variable

void controlServo(int angle)
{
    std_msgs::UInt16 msg;
    msg.data = angle;
    servoPub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_control");
    ros::NodeHandle n;
    servoPub = n.advertise<std_msgs::UInt16>("servo", 10); // Initialize servoPub

    int angle = 100;  // Set the desired angle here

    while (ros::ok())
    {
        controlServo(angle);
        ros::spinOnce();
    }

    return 0;
}

