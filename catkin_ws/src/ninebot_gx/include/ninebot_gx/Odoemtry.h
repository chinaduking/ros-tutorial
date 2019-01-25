#ifndef _Odoemtry_H_
#define _Odoemtry_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "ninebot_gx/sensor.h"

#define pi      3.141593f

namespace odometry
{
    class Odometry
    {
        public:
            Odometry(const ros::NodeHandle& nh,float wheel_interval,float deceleration_ratio,float wheel_diameter,uint16_t encoder_number,float encoder_multiplier);

        private:
            void Odometry_Cacl(int16_t right_ticks,int16_t left_ticks);
            void Sensor_callback(const ninebot_gx::sensor::ConstPtr &msg);
            void TimeUpdate20Hz(const ros::TimerEvent& event);
        
            ros::NodeHandle nh_;

            ros::Subscriber Sensor_sub_;
            ros::Publisher Odom_pub_;

            ros::Timer update_timer_;

            tf::TransformBroadcaster odom_broadcaster;
            nav_msgs::Odometry odom;//定义里程计对象
            geometry_msgs::Quaternion odom_quat; //四元数变量
            geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息

            int16_t L_Ticks_;
            int16_t R_Ticks_;
            int16_t Pre_L_Ticks_;
            int16_t Pre_R_Ticks_;

            float oriention;             //里程计方向
            float position_x;            //程计x坐标
            float position_y;            //程计y坐标
            float velocity_linear;       //里程计线速度
            float velocity_angular;      //里程计角速度

            float wheel_interval;        //轴距  mm
            float deceleration_ratio;    //减速比
            float wheel_diameter;        //轮子直径，单位mm
            float encoder_number;        //码盘线数
            float encoder_multiplier;    //码盘倍频数

            float const_frame;
            float const_angle;
    };

}

#endif