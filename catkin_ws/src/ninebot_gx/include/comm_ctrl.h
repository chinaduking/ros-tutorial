#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMM_CTRL_H
#define COMM_CTRL_H

#include <stdbool.h>
#include <stdint.h>


#define TYPE_ID_NUM                     18   //回调函数的个数
//--------------------------DATA CALL BACK INDEX--------------------------
#define Body_Data_Imu                   1
#define Body_Data_Irf                   2
#define Body_Data_Ultrasonic            3

#define Chassis_Data_Speed              4
#define Chassis_Data_Imu                5
#define Chassis_Data_Ticks              6

#define Connect_Data_Irf_Group1         7
#define Connect_Data_Irf_Group2         8
#define Connect_Data_Irf_Group3         9
#define Connect_Data_Irf_Group4         10
#define Connect_Data_Irf_Group5         11
#define Connect_Data_Irf_Group6         12
#define Connect_Data_Bumper             13
#define Connect_Data_Ultrasonic         14

#define Chassis_Data_Iq                 15

#define Chassis_Imu_Gyro                16
#define Chassis_Imu_Acc                 17

//-----------------------Event---------------------------------------------
#define PadPowerOffEvent      1
#define OnEmergeStopEvent     2
#define OutEmergeStopEvent    3
#define LASER1EVENT           4
#define LASER2EVENT           5
#define LASER3EVENT           6
#define LASER4EVENT           7

void AprGxJniEventRegister(int32_t (*callback)(int32_t));

//-------------------Timestamp------------------------
#define MAX_BASIC_FRM_SZ 0x1F

#pragma pack(1)
typedef struct StampedBasicFrame_{
    uint32_t type_id;                 //数据类型编号
    uint64_t timestamp;               //Linux时间戳
    char  data[MAX_BASIC_FRM_SZ];     //具体数值
} StampedBasicFrame;
#pragma pack()
typedef void (*h_aprctrl_datastamped_t)(StampedBasicFrame* frm);

typedef struct {
    h_aprctrl_datastamped_t on_new_data;
}s_aprctrl_datastamped_t;

void aprctrl_datastamped_jni_register(s_aprctrl_datastamped_t* f);  //时间戳回调注册函数

//--------------------GX--API------------------
int init_control(void);
void exit_control(void);

//-----------------GX  IAP--------------------
void IapAllBoard(const char **file_paths, char **versions);
int32_t IapSingerBoard(char * path,char * boradname,char* version);
int32_t getIapTotalProgress(void);

//------------------GX Host-------------------
uint16_t get_gx_host_version(void);
char* GetGxErrCoder(void);
char* GetGxHardVersion(void);
void TracePrint_Switch(bool status);  //true  or false

//------------------GX Route-------------------
uint16_t get_gx_route_version(void);
uint32_t get_gx_route_Err_Status(void);
void set_gx_poweroff(void);
void set_gx_route_bat_voltage(uint16_t voltage);
void reset_sys_power(void);
uint16_t get_power_off_reason(uint8_t time_t);
uint16_t get_power_off_bat_vol(void);
void set_gx_route_charge_status(uint16_t status_t);
void heart_beats_to_route(void);

//-------------------GX  Body-----------------
uint16_t get_gx_body_version(void);
uint32_t get_gx_body_Err_Status(void);
void set_gx_body_led_model(uint16_t mode);
uint16_t get_gx_body_Imu_Calib_Status(void);
uint16_t get_gx_body_Imu_Calib_Result(void);
void clear_gx_body_Imu_Calib_Result(void);
void Calib_body_Imu(void);

//------------------GX Connect--------------------
uint16_t get_gx_connect_version(void);
uint32_t get_gx_connect_Err_Status(void);
uint16_t get_gx_connect_bumpers(void);
void set_gx_connect_led_model(uint16_t mode);
void set_gx_connect_bat_voltage(uint16_t voltage);

//-------------------GX  Chassis-----------------
uint16_t get_gx_chassis_version(void);
uint32_t get_gx_chassis_Err_Status(void);
void set_gx_speed_forward(uint16_t speed_forward);
void set_gx_speed_turn(uint16_t speed_turn);
void set_gx_speed_loop(uint16_t enable);
void set_gx_linespeed_limit(int16_t line_limit_speed);
void set_gx_anglespeed_limit(int16_t angle_limit_speed);
void set_gx_chassis_bumper_status(int16_t status);
void set_gx_chassis_bms_offline(int16_t status);
uint16_t get_gx_LineSpeed_limit(void);
uint16_t get_gx_AngleSpeed_limit(void);
uint16_t get_gx_Chassis_work_model(void);
uint16_t get_gx_Mile(void);
uint16_t get_gx_Stop_Bottom_Status(void);
int16_t get_gx_Chassis_Imu_GX(void);
int16_t get_gx_Chassis_Imu_GY(void);
int16_t get_gx_Chassis_Imu_GZ(void);
int16_t get_gx_Chassis_Imu_AX(void);
int16_t get_gx_Chassis_Imu_AY(void);
int16_t get_gx_Chassis_Imu_AZ(void);
uint16_t get_gx_Chassis_Imu_Calib_Status(void);
void Calib_Chassis_Imu(void);
uint16_t get_gx_Chassis_Imu_Calib_Result(void);
void clear_gx_Chassis_Imu_Calib_Result(void);
void clear_gx_Chassis_Stop_Bottom_Status(void);
int16_t get_gx_Chassis_LMotor_HallSpeed(void);
int16_t get_gx_Chassis_RMotor_HallSpeed(void);
//int16_t get_gx_Chassis_LMotor_Iq(void);
//int16_t get_gx_Chassis_RMotor_Iq(void);

//------------------GX Bms--------------------
uint16_t get_gx_bms_version(void);
uint32_t get_gx_bms_Err_Status(void);
uint16_t get_bat_voltage(void);
uint16_t get_gx_bms_charge_status(void);
uint16_t get_gx_bms_offline_status(void);
uint16_t get_gx_bms_voltage(void);
int16_t get_gx_bms_current(void);


//-----------------Body Data typedef-------------------------//
typedef struct GxBodyImuData_ {
    int16_t  Pitch;
    int16_t  Roll;
    int16_t  Yaw;
}GxBodyImuData;

typedef struct GxBodyIrfData_ {
    uint16_t  IR_Dist_1;
    uint16_t  IR_Dist_2;
}GxBodyIrfData;

typedef struct GxBodyUltrasonicData_ {
    uint16_t  Ul_Dist_1;
//    uint16_t  Ul_Dist_2;
//    uint16_t  Ul_Dist_3;
}GxBodyUltrasonicData;

//-----------------Chassis Data typedef-------------------------//
typedef struct GxChassisSpeedData_ {
    int16_t  L_Motor_Speed;
    int16_t  R_Motor_Speed;
    int16_t  Forward_Speed;
    int16_t  Turn_Speed;
}GxChassisSpeedData;

typedef struct GxChassisImuData_ {
    int16_t  Pitch;
    int16_t  Roll;
    int16_t  Yaw;
}GxChassisImuData;

typedef struct GxChassisTicksData_ {
    int32_t  L_Motor_Ticks;
    int32_t  R_Motor_Ticks;
}GxChassisTicksData;

typedef struct GxChassisIqData_ {
    int16_t  L_Motor_Iq;
    int16_t  R_Motor_Iq;
}GxChassisIqData;

typedef struct GxChassisImuGroData_ {
    uint16_t GX;
    uint16_t GY;
    uint16_t GZ;
}GxChassisImuGroData;

typedef struct GxChassisImuAccData_ {
    uint16_t  AX;
    uint16_t  AY;
    uint16_t  AZ;
}GxChassisImuAccData_;

//-----------------Connect Data typedef-------------------------//
typedef struct GxConnectIrfGroup1Data_ {
    uint16_t  Irf01;
    uint16_t  Irf02;
    uint16_t  Irf03;
}GxConnectIrfGroup1Data;

typedef struct GxConnectIrfGroup2Data_ {
    uint16_t  Irf04;
    uint16_t  Irf05;
    uint16_t  Irf06;
}GxConnectIrfGroup2Data;

typedef struct GxConnectIrfGroup3Data_ {
    uint16_t  Irf07;
    uint16_t  Irf08;
    uint16_t  Irf09;
}GxConnectIrfGroup3Data;

typedef struct GxConnectIrfGroup4Data_ {
    uint16_t  Irf10;
    uint16_t  Irf11;
    uint16_t  Irf12;
}GxConnectIrfGroup4Data;

typedef struct GxConnectIrfGroup5Data_ {
    uint16_t  Irf13;
    uint16_t  Irf14;
    //uint16_t  Irf15;
}GxConnectIrfGroup5Data;

//typedef struct GxConnectIrfGroup6Data_ {
//    uint16_t  Irf16;
//    uint16_t  Irf17;
//    uint16_t  Irf18;
//}GxConnectIrfGroup6Data;

typedef struct GxConnectBumperData_ {
    uint16_t  Value;
}GxConnectBumperData;

typedef struct GxConnectUltrasonicData_ {
    uint16_t  Ul_Dist_1;
    uint16_t  Ul_Dist_2;
}GxConnectUltrasonicData;

#endif

#ifdef __cplusplus
}
#endif