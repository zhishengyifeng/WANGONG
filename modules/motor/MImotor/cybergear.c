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
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "general_def.h"
#include "daemon.h"
#include "can.h"
#include "cybergear.h"
#include "string.h"
#include "stdlib.h"

uint8_t byte[4];              // ת����ʱ����
uint32_t send_mail_box = {0}; // NONE

static uint8_t idx;
static MIMotorInstance *mi_motor_instance[MI_MOTOR_CNT];
static osThreadId mi_task_handle[MI_MOTOR_CNT];

/**
 * @brief          ������ת4�ֽں���
 * @param[in]      f:������
 * @retval         4�ֽ�����
 * @description  : IEEE 754 Э��
 */
static uint8_t *Float_to_Byte(float f)
{
  unsigned long longdata = 0;
  longdata = *(unsigned long *)&f;
  byte[0] = (longdata & 0xFF000000) >> 24;
  byte[1] = (longdata & 0x00FF0000) >> 16;
  byte[2] = (longdata & 0x0000FF00) >> 8;
  byte[3] = (longdata & 0x000000FF);
  return byte;
}

/**
 * @brief          С�׵������16λ����ת����
 * @param[in]      x:16λ����
 * @param[in]      x_min:��Ӧ��������
 * @param[in]      x_max:��Ӧ��������
 * @param[in]      bits:����λ��
 * @retval         ���ظ���ֵ
 */
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
  uint32_t span = (1 << bits) - 1;
  float offset = x_max - x_min;
  return offset * x / span + x_min;
}

/**
 * @brief          С�׵�����͸���ת16λ����
 * @param[in]      x:����
 * @param[in]      x_min:��Ӧ��������
 * @param[in]      x_max:��Ӧ��������
 * @param[in]      bits:����λ��
 * @retval         ���ظ���ֵ
 */
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief          д��������
 * @param[in]      Motor:��Ӧ���Ƶ���ṹ��
 * @param[in]      Index:д�������Ӧ��ַ
 * @param[in]      Value:д�����ֵ
 * @param[in]      Value_type:д�������������
 * @retval         none
 */
static void MIMotorSetParameter(MIMotorInstance *Motor, uint16_t Index, float Value, char Value_type)
{
  Motor->motor_can_instace->txconf.ExtId = MI_Communication_Type_SetSingleParameter << 24 | Master_CAN_ID << 8 | Motor->measure.CAN_ID;
  Motor->motor_can_instace->tx_buff[0] = Index;
  Motor->motor_can_instace->tx_buff[1] = Index >> 8;
  Motor->motor_can_instace->tx_buff[2] = 0x00;
  Motor->motor_can_instace->tx_buff[3] = 0x00;
  if (Value_type == 'f')
  {
    Float_to_Byte(Value);
    Motor->motor_can_instace->tx_buff[4] = byte[3];
    Motor->motor_can_instace->tx_buff[5] = byte[2];
    Motor->motor_can_instace->tx_buff[6] = byte[1];
    Motor->motor_can_instace->tx_buff[7] = byte[0];
  }
  else if (Value_type == 's')
  {
    Motor->motor_can_instace->tx_buff[4] = (uint8_t)Value;
    Motor->motor_can_instace->tx_buff[5] = 0x00;
    Motor->motor_can_instace->tx_buff[6] = 0x00;
    Motor->motor_can_instace->tx_buff[7] = 0x00;
  }
  CANTransmit(Motor->motor_can_instace, 0.5);
}

/**
 * @brief          ��ȡ����ظ�֡��չID�еĵ��CANID
 * @param[in]      CAN_ID_Frame:����ظ�֡�е���չCANID
 * @retval         ���CANID
 */
uint32_t MIMotorGetID(uint32_t CAN_ID_Frame)
{
  return (CAN_ID_Frame & 0xFFFF) >> 8;
}

/**
 * @brief          ����ظ�֡���ݴ�����
 * @param[in]      Motor:��Ӧ���Ƶ���ṹ��
 * @param[in]      DataFrame:����֡
 * @param[in]      IDFrame:��չID֡
 * @retval         None
 */
static void MIMotorDataHandler(MIMotorInstance *Motor, uint8_t DataFrame[8], uint32_t IDFrame)
{
  Motor->measure.Angle = uint16_to_float(DataFrame[0] << 8 | DataFrame[1], MI_MIN_P, MI_MAX_P, 16);
  Motor->measure.Speed = uint16_to_float(DataFrame[2] << 8 | DataFrame[3], MI_V_MIN, MI_V_MAX, 16);
  Motor->measure.Torque = uint16_to_float(DataFrame[4] << 8 | DataFrame[5], MI_T_MIN, MI_T_MAX, 16);
  Motor->measure.Temp = (DataFrame[6] << 8 | DataFrame[7]) * MI_Temp_Gain;
  Motor->measure.error_code = (IDFrame & 0x1F0000) >> 16;
}

/**
 * @brief          С�׵��ID���
 * @param[in]      id:  ��Ӧ���Ƶ���ṹ��
 * @retval         none
 */
void MIMotorChack(MIMotorInstance *Motor, uint8_t ID)
{
  for (uint8_t i = 0; i < 8; ++i)
    Motor->motor_can_instace->tx_buff[i] = 0;
  Motor->motor_can_instace->txconf.ExtId = MI_Communication_Type_GetID << 24 | Master_CAN_ID << 8 | ID;
  CANTransmit(Motor->motor_can_instace, 0.5);
}

/**
 * @brief          ʹ��С�׵��
 * @param[in]      Motor:��Ӧ���Ƶ���ṹ��
 * @retval         none
 */
void MIMotorEnable(MIMotorInstance *Motor)
{
  for (uint8_t i = 0; i < 8; ++i)
    Motor->motor_can_instace->tx_buff[i] = 0;
  Motor->motor_can_instace->txconf.ExtId = MI_Communication_Type_MotorEnable << 24 | Master_CAN_ID << 8 | Motor->measure.CAN_ID;
  CANTransmit(Motor->motor_can_instace, 0.5);
}

/**
 * @brief          ֹͣ���
 * @param[in]      Motor:��Ӧ���Ƶ���ṹ��
 * @param[in]      clear_error:�������λ��0 ����� 1�����
 * @retval         None
 */
void MIMotorStop(MIMotorInstance *Motor, uint8_t clear_error)
{
  for (uint8_t i = 0; i < 8; ++i)
    Motor->motor_can_instace->tx_buff[i] = 0;
  Motor->motor_can_instace->tx_buff[0] = clear_error; // �������λ����
  Motor->motor_can_instace->txconf.ExtId = MI_Communication_Type_MotorStop << 24 | Master_CAN_ID << 8 | Motor->measure.CAN_ID;
  CANTransmit(Motor->motor_can_instace, 0.5);
}

/**
 * @brief          ���õ��ģʽ(����ֹͣʱ������)
 * @param[in]      Motor:  ����ṹ��
 * @param[in]      Mode:   �������ģʽ��1.�˶�ģʽMotion_mode 2. λ��ģʽPosition_mode 3. �ٶ�ģʽSpeed_mode 4. ����ģʽCurrent_mode��
 * @retval         none
 */
void MIMotorSetMode(MIMotorInstance *Motor, uint8_t Mode)
{
  MIMotorSetParameter(Motor, MI_Run_mode, Mode, 's');
}

/**
 * @brief          ��������ģʽ�����õ���
 * @param[in]      Motor:  ����ṹ��
 * @param[in]      Current:��������
 * @retval         none
 */
void MIMotorSetCurrent(MIMotorInstance *Motor, float Current)
{
  MIMotorSetParameter(Motor, MI_Iq_Ref, Current, 'f');
}

/**
 * @brief          ���õ�����
 * @param[in]      Motor:  ����ṹ��
 * @retval         none
 */
void MIMotorSetZeropos(MIMotorInstance *Motor)
{
  for (uint8_t i = 0; i < 8; ++i)
    Motor->motor_can_instace->tx_buff[i] = 0;
  Motor->motor_can_instace->txconf.ExtId = MI_Communication_Type_SetPosZero << 24 | Master_CAN_ID << 8 | Motor->measure.CAN_ID;
  CANTransmit(Motor->motor_can_instace, 0.5);
}

/**
 * @brief          ���õ��CANID
 * @param[in]      Motor:  ����ṹ��
 * @param[in]      Motor:  ������ID
 * @retval         none
 */
void MIMotorSetCANID(MIMotorInstance *Motor, uint8_t CAN_ID)
{
  for (uint8_t i = 0; i < 8; ++i)
    Motor->motor_can_instace->tx_buff[i] = 0;
  Motor->motor_can_instace->txconf.ExtId = MI_Communication_Type_CanID << 24 | CAN_ID << 16 | Master_CAN_ID << 8 | Motor->measure.CAN_ID;
  Motor->measure.CAN_ID = CAN_ID; // ���µ�ID�������ṹ��
  CANTransmit(Motor->motor_can_instace, 0.5);
}

/**
 * @brief          С�׵����ʼ��
 * @param[in]      Motor:  ����ṹ��
 * @param[in]      Can_Id: С�׵��ID(Ĭ��0x7F)
 * @param[in]      Motor_Num: ������
 * @param[in]      mode: �������ģʽ��0.�˶�ģʽMotion_mode 1. λ��ģʽPosition_mode 2. �ٶ�ģʽSpeed_mode 3. ����ģʽCurrent_mode��
 * @retval         none
 */
void _MIMotorInit(MIMotorInstance *Motor, uint8_t mode)
{
  Motor->motor_can_instace->txconf.StdId = 0;          // ����CAN���ͣ���׼֡����
  Motor->motor_can_instace->txconf.ExtId = 0;          // ����CAN���ͣ���չ֡����
  Motor->motor_can_instace->txconf.IDE = CAN_ID_EXT;   // ����CAN���ͣ���չ֡
  Motor->motor_can_instace->txconf.RTR = CAN_RTR_DATA; // ����CAN���ͣ�����֡
  Motor->motor_can_instace->txconf.DLC = 0x08;         // ����CAN���ͣ����ݳ���

  Motor->measure.CAN_ID = MIMotorGetID(Motor->motor_can_instace->rx_id);       // ID����
  MIMotorSetMode(Motor, mode);          // ���õ��ģʽ
  MIMotorEnable(Motor);                 // ʹ�ܵ��
}

/**
 * @brief          С���˿�ģʽָ��
 * @param[in]      Motor:  Ŀ�����ṹ��
 * @param[in]      torque: ��������[-12,12] N*M
 * @param[in]      MechPosition: λ������[-12.5,12.5] rad
 * @param[in]      speed: �ٶ�����[-30,30] rpm
 * @param[in]      kp: ������������
 * @param[in]      kd: ΢�ֲ�������
 * @retval         none
 */
void MIMotorControlMode(MIMotorInstance *Motor, float torque, float MechPosition, float speed, float kp, float kd)
{
  // װ�������
  Motor->motor_can_instace->tx_buff[0] = float_to_uint(MechPosition, MI_P_MIN, MI_P_MAX, 16) >> 8;
  Motor->motor_can_instace->tx_buff[1] = float_to_uint(MechPosition, MI_P_MIN, MI_P_MAX, 16);
  Motor->motor_can_instace->tx_buff[2] = float_to_uint(speed, MI_V_MIN, MI_V_MAX, 16) >> 8;
  Motor->motor_can_instace->tx_buff[3] = float_to_uint(speed, MI_V_MIN, MI_V_MAX, 16);
  Motor->motor_can_instace->tx_buff[4] = float_to_uint(kp, MI_KP_MIN, MI_KP_MAX, 16) >> 8;
  Motor->motor_can_instace->tx_buff[5] = float_to_uint(kp, MI_KP_MIN, MI_KP_MAX, 16);
  Motor->motor_can_instace->tx_buff[6] = float_to_uint(kd, MI_KD_MIN, MI_KD_MAX, 16) >> 8;
  Motor->motor_can_instace->tx_buff[7] = float_to_uint(kd, MI_KD_MIN, MI_KD_MAX, 16);
  Motor->motor_can_instace->txconf.ExtId = MI_Communication_Type_MotionControl << 24 | float_to_uint(torque, MI_T_MIN, MI_T_MAX, 16) << 8 | Motor->measure.CAN_ID; // װ����չ֡����
  CANTransmit(Motor->motor_can_instace, 0.5);
}

/*****************************�ص����� ����ӻش���Ϣ ��ת������*****************************/
/**
 * @brief          С�׵�����ջص�����,���յ������
 * @param[in]      motor_can:CANʵ��
 * @retval         none
 */
static void MIMotorDecode(CANInstance *motor_can)
{
  uint16_t tmp; // �����ݴ����ֵ,�Ժ�ת����float����,�����δ�����ʱ����
  uint8_t *rxbuff = motor_can->rx_buff;
  MIMotorInstance *motor = (MIMotorInstance *)motor_can->id;
  MIMotor_Measure_t *measure = &(motor->measure); // ��canʵ���б����idת���ɵ��ʵ����ָ��

  DaemonReload(motor->motor_daemon);
  measure->feed_dt = DWT_GetDeltaT(&measure->feed_cnt);
  
  // if (motor_can->rxconf.ExtId >> 24 != 0) // ����Ƿ�Ϊ�㲥ģʽ
  if (1)
    MIMotorDataHandler(motor, motor->motor_can_instace->rx_buff, motor_can->rx_id);
  else
    motor->measure.MCU_ID = motor->motor_can_instace->rx_buff[0];
}

/**
 * @brief          �����ʧ�ص�����,��д����ʧ״̬�ķ���
 * @param[in]      motor_ptr:
 * @retval         none
 */
static void MIMotorLostCallback(void *motor_ptr)
{
  
}

MIMotorInstance *MIMotorInit(Motor_Init_Config_s *config)
{
    MIMotorInstance *motor = (MIMotorInstance *)malloc(sizeof(MIMotorInstance));
    memset(motor, 0, sizeof(MIMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = MIMotorDecode;
    config->can_init_config.id = motor;
    config->can_init_config.rx_id = (config->can_init_config.rx_id << 8) | 0x02000000;//���������õ�motor->motor_can_instace->rx_id�ĵط��������Ϊmotor->measure.CAN_ID
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = MIMotorLostCallback,
        .owner_id = motor,
        .reload_count = 5, // 20ms
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DWT_Delay(0.1);
    _MIMotorInit(motor, MI_Motion_mode);  //�������ó��˿�ģʽ����С���Դ���PID�㷨�������Լ�������ֻ��Ҫ�������������Ŀ���ٶȼ���
    DWT_Delay(0.1);

    mi_motor_instance[idx++] = motor;
    return motor;
}

/**
 * @brief          �ⲿ����С�׵���ο�ֵ����������С�׵���Դ�PID�㷨���������ֱ�Ӱ����ֵ������Ҫ���ٶȼ���
 * @param[in]      motor:
 * @param[in]      ref:
 * @retval         none
 */
void MIMotorSetRef(MIMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

/**
 * @brief Ϊ�˱������߶���,Ϊÿ���������һ����������
 * @param argument ����ĵ��ָ��
 */
void MIMotorTask(void const *argument)
{
    float set, pid_measure, pid_ref;
    MIMotorInstance *motor = (MIMotorInstance *)argument;
    MIMotor_Measure_t *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    CANInstance *motor_can = motor->motor_can_instace;
    uint16_t tmp;

    while (1)
    {
        pid_ref = motor->pid_ref;
        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_angle_feedback_ptr;
            else
                pid_measure = measure->Angle;
            // measure��λ��rad,ref�ǽǶ�,ͳһ��angle�¼���,���㽨ģ
            pid_ref = PIDCalculate(&motor->angle_PID, pid_measure * RAD_2_DEGREE, pid_ref);
        }

        if ((setting->close_loop_type & SPEED_LOOP) && setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))
        {
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor->speed_feedforward_ptr;

            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor->other_speed_feedback_ptr;
            else
                pid_measure = measure->Speed;
            // measure��λ��rad / s ,ref��angle per sec,ͳһ��angle�¼���
            pid_ref = PIDCalculate(&motor->speed_PID, pid_measure * RAD_2_DEGREE, pid_ref);
        }

        // С�׵������������ֵ
        // if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
        //     pid_ref += *motor->current_feedforward_ptr;
        // if (setting->close_loop_type & CURRENT_LOOP)
        // {
        //     pid_ref = PIDCalculate(&motor->current_PID, measure->real_current, pid_ref);
        // }

        set = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;

        MIMotorControlMode(motor, 0, 0, set, motor->speed_PID.Kp, motor->speed_PID.Kd);//��������ֻ��Ҫ����Ŀ���ٶȼ���(-30rad/s ~ 30red/s)

        osDelay(1);
    }
}

void MIMotorControlInit(void)
{
    char mi_task_name[5] = "mi";
    // �������е��ʵ��,��������
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char mi_id_buff[2] = {0};
        __itoa(i, mi_id_buff, 10);
        strcat(mi_task_name, mi_id_buff);
        osThreadDef(mi_task_name, MIMotorTask, osPriorityNormal, 0, 256);
        mi_task_handle[i] = osThreadCreate(osThread(mi_task_name), mi_motor_instance[i]);
    }
}
