/**
  ****************************(C)SWJTU_ROBOTCON****************************
  * @file       MI.c/h
  * @brief      С�׵��������
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

// ���Ʋ�����ֵ����������
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
// ����CANID����
#define Master_CAN_ID 0x00 // ����ID
// ��������궨��
#define MI_Communication_Type_GetID 0x00         // ��ȡ�豸��ID��64λMCUΨһ��ʶ��
#define MI_Communication_Type_MotionControl 0x01 // �������������Ϳ���ָ��
#define MI_Communication_Type_MotorRequest 0x02  // ���������������������״̬
#define MI_Communication_Type_MotorEnable 0x03   // ���ʹ������
#define MI_Communication_Type_MotorStop 0x04     // ���ֹͣ����
#define MI_Communication_Type_SetPosZero 0x06    // ���õ����е��λ
#define MI_Communication_Type_CanID 0x07         // ���ĵ�ǰ���CAN_ID
#define MI_Communication_Type_Control_Mode 0x12
#define MI_Communication_Type_GetSingleParameter 0x11 // ��ȡ��������
#define MI_Communication_Type_SetSingleParameter 0x12 // �趨��������
#define MI_Communication_Type_ErrorFeedback 0x15      // ���Ϸ���֡
// ������ȡ�궨��
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

enum CONTROL_MODE // ����ģʽ����
{
    MI_Motion_mode = 0, // �˿�ģʽ
    MI_Position_mode,   // λ��ģʽ
    MI_Speed_mode,      // λ��ģʽ
    MI_Current_mode     // ����ģʽ
};
enum ERROR_TAG // ����ش�����
{
    MI_OK = 0,                // �޹���
    MI_BAT_LOW_ERR = 1,       // Ƿѹ����
    MI_OVER_CURRENT_ERR = 2,  // ����
    MI_OVER_TEMP_ERR = 3,     // ����
    MI_MAGNETIC_ERR = 4,      // �ű������
    MI_HALL_ERR_ERR = 5,      // HALL�������
    MI_NO_CALIBRATION_ERR = 6 // δ�궨
};

typedef struct
{                   // С�׵���ṹ��
    uint8_t CAN_ID; // CAN ID
    uint8_t MCU_ID; // MCUΨһ��ʶ��[��8λ����64λ]
    float Angle;    // �ش��Ƕ�
    float Speed;    // �ش��ٶ�
    float Torque;   // �ش�����
    float Temp;

    uint16_t set_current;
    uint16_t set_speed;
    uint16_t set_position;

    uint8_t error_code;

    float Angle_Bias;

    float feed_dt;
    uint32_t feed_cnt;
} MIMotor_Measure_t;

/* MI������Ͷ���*/
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

    Motor_Working_Type_e stop_flag; // ��ͣ��־

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
