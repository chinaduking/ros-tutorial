#include "ros/ros.h"
#include "ninebot_gx/Odoemtry.h"

#define WHEEL_INTERVAL          423.20f  //两轮之间的轴距   mm
#define DECELERATION_RATIO      1.0f     //减速比
#define WHEEL_DIAMETER          200.0f   //轮子直径 mm
#define ENCODER_NUM             (uint16_t)16384    //编码器线数
#define ENCODER_MULTIPLIER      1.0f     //编码器倍频数

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;

    odometry::Odometry gxOdoemtry(n,WHEEL_INTERVAL,DECELERATION_RATIO,WHEEL_DIAMETER,ENCODER_NUM,ENCODER_MULTIPLIER);

    ros::Rate loop_rate(20);
 
    while (ros::ok())
    { 
        ros::spinOnce();//周期执行
        loop_rate.sleep();
    }
    return 0;
}