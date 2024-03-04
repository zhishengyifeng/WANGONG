/**
  ****************************(C)SWJTU_ROBOTCON****************************
  * @file       MI.c/h
  * @brief      小米电机函数库
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     1-10-2023       ZDYukino        1. done
  *
  @verbatim
  =========================================================================
  =========================================================================
  @endverbatim
  ****************************(C)SWJTU_ROBOTCON****************************
  **/

#ifndef CYBERGEAR_H
#define CYBERGEAR_H

#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

// 控制参数最值，谨慎更改
#define MI_P_MIN -12.5f
#define MI_P_MAX 12.5f
#define MI_V_MIN -30.0f
#define MI_V_MAX 30.0f
#define MI_KP_MIN 0.0f
#define MI_KP_MAX 500.0f
#define MI_KD_MIN 0.0f
#define MI_KD_MAX 5.0f
#define MI_T_MIN -12.0f
#define MI_T_MAX 12.0f
#define MI_MAX_P 720
#define MI_MIN_P -720
// 主机CANID设置
#define Master_CAN_ID 0x00 // 主机ID
// 控制命令宏定义
#define MI_Communication_Type_GetID 0x00         // 获取设备的ID和64位MCU唯一标识符
#define MI_Communication_Type_MotionControl 0x01 // 用来向主机发送控制指令
#define MI_Communication_Type_MotorRequest 0x02  // 用来向主机反馈电机运行状态
#define MI_Communication_Type_MotorEnable 0x03   // 电机使能运行
#define MI_Communication_Type_MotorStop 0x04     // 电机停止运行
#define MI_Communication_Type_SetPosZero 0x06    // 设置电机机械零位
#define MI_Communication_Type_CanID 0x07         // 更改当前电机CAN_ID
#define MI_Communication_Type_Control_Mode 0x12
#define MI_Communication_Type_GetSingleParameter 0x11 // 读取单个参数
#define MI_Communication_Type_SetSingleParameter 0x12 // 设定单个参数
#define MI_Communication_Type_ErrorFeedback 0x15      // 故障反馈帧
// 参数读取宏定义
#define MI_Run_mode 0x7005
#define MI_Iq_Ref 0x7006
#define MI_Spd_Ref 0x700A
#define MI_Limit_Torque 0x700B
#define MI_Cur_Kp 0x7010
#define MI_Cur_Ki 0x7011
#define MI_Cur_Filt_Gain 0x7014
#define MI_Loc_Ref 0x7016
#define MI_Limit_Spd 0x7017
#define MI_Limit_Cur 0x7018

#define MI_Gain_Angle 720 / 32767.0
#define MI_Bias_Angle 0x8000
#define MI_Gain_Speed 30 / 32767.0
#define MI_Bias_Speed 0x8000
#define MI_Gain_Torque 12 / 32767.0
#define MI_Bias_Torque 0x8000
#define MI_Temp_Gain 0.1

#define MI_Motor_Error 0x00
#define MI_Motor_OK 0X01

#define MI_MOTOR_CNT 2

enum CONTROL_MODE // 控制模式定义
{
    MI_Motion_mode = 0, // 运控模式
    MI_Position_mode,   // 位置模式
    MI_Speed_mode,      // 位置模式
    MI_Current_mode     // 电流模式
};
enum ERROR_TAG // 错误回传对照
{
    MI_OK = 0,                // 无故障
    MI_BAT_LOW_ERR = 1,       // 欠压故障
    MI_OVER_CURRENT_ERR = 2,  // 过流
    MI_OVER_TEMP_ERR = 3,     // 过温
    MI_MAGNETIC_ERR = 4,      // 磁编码故障
    MI_HALL_ERR_ERR = 5,      // HALL编码故障
    MI_NO_CALIBRATION_ERR = 6 // 未标定
};

typedef struct
{                   // 小米电机结构体
    uint8_t CAN_ID; // CAN ID
    uint8_t MCU_ID; // MCU唯一标识符[后8位，共64位]
    float Angle;    // 回传角度
    float Speed;    // 回传速度
    float Torque;   // 回传力矩
    float Temp;

    uint16_t set_current;
    uint16_t set_speed;
    uint16_t set_position;

    uint8_t error_code;

    float Angle_Bias;

    float feed_dt;
    uint32_t feed_cnt;
} MIMotor_Measure_t;

/* MI电机类型定义*/
typedef struct
{
    MIMotor_Measure_t measure;

    Motor_Control_Setting_s motor_settings;

    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;

    Motor_Working_Type_e stop_flag; // 启停标志

    CANInstance *motor_can_instace;

    DaemonInstance *motor_daemon;
    uint32_t lost_cnt;
} MIMotorInstance;

extern void MIMotorChack(MIMotorInstance *Motor, uint8_t ID);
extern void MIMotorEnable(MIMotorInstance *Motor);
extern void MIMotorStop(MIMotorInstance *Motor, uint8_t clear_error);
extern void MIMotorSetMode(MIMotorInstance *Motor, uint8_t Mode);
extern void MIMotorSetCurrent(MIMotorInstance *Motor, float Current);
extern void MIMotorSetZeropos(MIMotorInstance *Motor);
extern void MIMotorSetCANID(MIMotorInstance *Motor, uint8_t CAN_ID);
extern void _MIMotorInit(MIMotorInstance *Motor, uint8_t mode);
extern void MIMotorControlMode(MIMotorInstance *Motor, float torque, float MechPosition, float speed, float kp, float kd);
extern uint32_t MIMotorGetID(uint32_t CAN_ID_Frame);

MIMotorInstance *MIMotorInit(Motor_Init_Config_s *config);
void MIMotorControlInit(void);
void MIMotorSetRef(MIMotorInstance *motor, float ref);

#endif
