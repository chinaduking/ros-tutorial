#include "ninebot_gx/gx_robot.h"

namespace gxrobot
{

    Chassis::Chassis(const ros::NodeHandle& nh): nh_(nh)
    {
        velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Chassis::cmd_vel_callback, this);
        SpeedLopp_sub_ = nh_.subscribe("speed_loop_cmd", 1, &Chassis::speedloop_cmd_callback, this);

        Sensor_pub = nh_.advertise<ninebot_gx::sensor>("Sensor", 1);
        Version_pub = nh_.advertise<ninebot_gx::version>("version_info", 1);

        update_timer_ = nh_.createTimer(ros::Duration(0.05), &Chassis::TimeUpdate20Hz,this);
    }

    /* code */
    void Chassis::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd_input)//订阅/cmd_vel主题回调函数
    {
        angular_vel_ = cmd_input->angular.z ;//获取/cmd_vel的角速度,rad/s
        linear_vel_ = cmd_input->linear.x ;//获取/cmd_vel的线速度.m/s

        set_gx_speed_forward(int(linear_vel_*3600));
        set_gx_speed_turn(int(angular_vel_*2272));

        ROS_INFO("angular_temp:%f  rad/s   linear_temp:%f  m/s", angular_vel_,linear_vel_);
    }

    void Chassis::speedloop_cmd_callback(const ninebot_gx::status::ConstPtr& msg)
    {
        speedloop_cmd_ = msg->speedloop ;//获取speedloop_cmd

        speedloop_status_ = get_gx_Chassis_work_model();

        if(speedloop_cmd_ == 1){
            if(speedloop_status_ == 0){
                set_gx_speed_loop(1);
            }
        }else{
            if(speedloop_status_ == 1){
                set_gx_speed_loop(0);
            }
        }

        ROS_INFO("speedloop_cmd_:%d", speedloop_cmd_);
    }

    void Chassis::TimeUpdate20Hz(const ros::TimerEvent& event)
    {
        gxSensor.bumper = 10;
        gxSensor.l_ticks++;
        gxSensor.r_ticks++;

        gxVersion.Route = get_gx_route_version();

        Sensor_pub.publish(gxSensor);
        Version_pub.publish(gxVersion);

        ros::spinOnce();
    }
}
