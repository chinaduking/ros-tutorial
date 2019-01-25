#ifndef GX_ROBOT_H
#define GX_ROBOT_H

# include <ros/ros.h>
# include <geometry_msgs/Twist.h>
#include "comm_ctrl.h"
#include "ninebot_gx/sensor.h"
#include "ninebot_gx/version.h"
#include "ninebot_gx/status.h"

namespace gxrobot
{

    class Chassis
    {
        public:
            Chassis(const ros::NodeHandle& nh);

        private:
            void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& vel);
            void speedloop_cmd_callback(const ninebot_gx::status::ConstPtr& msg);
            void TimeUpdate20Hz(const ros::TimerEvent& event);
        
            ros::NodeHandle nh_;

            ros::Subscriber velocity_sub_;
            ros::Subscriber SpeedLopp_sub_;

            ros::Publisher Sensor_pub;
            ros::Publisher Version_pub;

            ros::Timer update_timer_;

            ninebot_gx::sensor gxSensor;
            ninebot_gx::version gxVersion;

            double linear_vel_;
            double angular_vel_;

            bool speedloop_cmd_;
            bool speedloop_status_;

    };

}

#endif