#include "ninebot_gx/Odoemtry.h"

namespace odometry
{
    Odometry::Odometry(const ros::NodeHandle& nh,float wheel_interval,float deceleration_ratio,\
                            float wheel_diameter,uint16_t encoder_number,float encoder_multiplier): nh_(nh)
    {
        Sensor_sub_ = nh_.subscribe("Sensor", 20, &Odometry::Sensor_callback,this); //订阅/Sensor主题

        Odom_pub_= nh_.advertise<nav_msgs::Odometry>("odom", 20,this); //定义要发布/odom主题

        update_timer_ = nh_.createTimer(ros::Duration(0.05), &Odometry::TimeUpdate20Hz,this);

        this->oriention = 0.0;
        Pre_R_Ticks_ = 0;
        Pre_L_Ticks_ = 0;

        this->wheel_interval = wheel_interval;
        this->deceleration_ratio = deceleration_ratio;
        this->wheel_diameter = wheel_diameter;
        this->encoder_number = encoder_number;
        this->encoder_multiplier = encoder_multiplier;

        this->const_frame=this->wheel_diameter*pi/(this->encoder_number*this->encoder_multiplier*this->deceleration_ratio);
        this->const_angle=this->const_frame/this->wheel_interval;
    }

    /* code */
    void Odometry::Odometry_Cacl(int16_t right_ticks,int16_t left_ticks)
    {
        float distance_sum = 0.0,distance_diff = 0.0;
        float delta_distance = 0.0,delta_oriention = 0.0; //采样时间间隔内运动的距离
        float oriention_interval=0.0;  //dt时间内方向变化值
        float oriention_1 = 0.0;
        float sin_=0;                   //角度计算值
        float cos_=0;
        float dt=0.05f;                 //采样时间间隔50ms
        float r_speed,l_speed;

        r_speed = right_ticks - Pre_R_Ticks_;
        l_speed = left_ticks - Pre_L_Ticks_;
        Pre_R_Ticks_ = right_ticks;
        Pre_L_Ticks_ = left_ticks;

        distance_sum = 0.5f*(right_ticks+left_ticks);//在很短的时间内，小车行驶的路程为两轮速度和
        distance_diff = right_ticks-left_ticks;//在很短的时间内，小车行驶的角度为两轮速度差

        //根据左右轮的方向，纠正短时间内，小车行驶的路程和角度量的正负
        if((r_speed>0)&&(l_speed>0))            //左右均正
        {
            delta_distance = distance_sum;
            delta_oriention = distance_diff;
        }
        else if((r_speed<0)&&(l_speed<0))       //左右均负
        {
            delta_distance = -distance_sum;
            delta_oriention = -distance_diff;
        }
        else if((r_speed<0)&&(l_speed>0))       //左正右负
        {
            delta_distance = -distance_diff;
            delta_oriention = -2.0f*distance_sum;       
        }
        else if((r_speed>0)&&(l_speed<0))       //左负右正
        {
            delta_distance = distance_diff;
            delta_oriention = 2.0f*distance_sum;
        }
        else
        {
            delta_distance=0;
            delta_oriention=0;
        }

        oriention_interval = delta_oriention * this->const_angle;//采样时间内走的角度  
        this->oriention = this->oriention + oriention_interval;//计算出里程计方向角
        oriention_1 = this->oriention + 0.5f * oriention_interval;//里程计方向角数据位数变化，用于三角函数计算

        sin_ = sin(oriention_1);//计算出采样时间内y坐标
        cos_ = cos(oriention_1);//计算出采样时间内x坐标

        this->position_x = this->position_x + delta_distance * cos_ * this->const_frame;//计算出里程计x坐标
        this->position_y = this->position_y + delta_distance * sin_ * this->const_frame;//计算出里程计y坐标

        this->velocity_linear = delta_distance*const_frame / dt;//计算出里程计线速度
        this->velocity_angular = oriention_interval / dt;//计算出里程计角速度

        //方向角角度纠正
        if(this->oriention > pi)
        {
            this->oriention -= 2*pi;
        }
        else
        {
            if(this->oriention < -pi)
            {
                this->oriention += 2*pi;
            }
        }

        //里程计的偏航角需要转换成四元数才能发布
        odom_quat = tf::createQuaternionMsgFromYaw(this->oriention);//将偏航角转换成四元数

        //载入坐标（tf）变换时间戳
        odom_trans.header.stamp = ros::Time::now();
        //发布坐标变换的父子坐标系
        odom_trans.header.frame_id = "odom";     
        odom_trans.child_frame_id = "base_footprint";       
        //tf位置数据：x,y,z,方向
        odom_trans.transform.translation.x = this->position_x/1000;  //m
        odom_trans.transform.translation.y = this->position_y/1000;  //m
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;        
        //发布tf坐标变化
        odom_broadcaster.sendTransform(odom_trans);

        //载入里程计时间戳
        odom.header.stamp = ros::Time::now(); 
        //里程计的父子坐标系
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";       
        //里程计位置数据：x,y,z,方向
        odom.pose.pose.position.x = position_x/1000;     
        odom.pose.pose.position.y = position_y/1000;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;       
        //载入线速度和角速度
        odom.twist.twist.linear.x = velocity_linear/1000;
        odom.twist.twist.angular.z = velocity_angular/1000;    
        //发布里程计
        Odom_pub_.publish(odom);

        ROS_INFO("Odometry_Cacl: x:%f   y:%f",odom.pose.pose.position.x,odom.pose.pose.position.y);
    }

    void Odometry::Sensor_callback(const ninebot_gx::sensor::ConstPtr &msg)
    {
        L_Ticks_ = msg->l_ticks;
        R_Ticks_ = msg->r_ticks;

        Odometry_Cacl(L_Ticks_,R_Ticks_);
    }

    void Odometry::TimeUpdate20Hz(const ros::TimerEvent& event)
    {
        ros::spinOnce();
    }
}
