#include "ros/ros.h"
#include "ninebot_gx/gx_robot.h"

int main(int argc, char **argv)
{
    init_control();

    ros::init(argc, argv, "SmartCar");
    ros::NodeHandle n;

    gxrobot::Chassis sbv(n);
    
    ros::Rate loop_rate(10);
 
    int count = 0;
    while (ros::ok())
    { 
        ros::spinOnce();//周期执行
        loop_rate.sleep();
        ++count;
    }
    return 0;
}