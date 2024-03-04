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

uint8_t byte[4];              // 转换临时数据
uint32_t send_mail_box = {0}; // NONE

static uint8_t idx;
static MIMotorInstance *mi_motor_instance[MI_MOTOR_CNT];
static osThreadId mi_task_handle[MI_MOTOR_CNT];

/**
 * @brief          浮点数转4字节函数
 * @param[in]      f:浮点数
 * @retval         4字节数组
 * @description  : IEEE 754 协议
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
 * @brief          小米电机回文16位数据转浮点
 * @param[in]      x:16位回文
 * @param[in]      x_min:对应参数下限
 * @param[in]      x_max:对应参数上限
 * @param[in]      bits:参数位数
 * @retval         返回浮点值
 */
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
  uint32_t span = (1 << bits) - 1;
  float offset = x_max - x_min;
  return offset * x / span + x_min;
}

/**
 * @brief          小米电机发送浮点转16位数据
 * @param[in]      x:浮点
 * @param[in]      x_min:对应参数下限
 * @param[in]      x_max:对应参数上限
 * @param[in]      bits:参数位数
 * @retval         返回浮点值
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
 * @brief          写入电机参数
 * @param[in]      Motor:对应控制电机结构体
 * @param[in]      Index:写入参数对应地址
 * @param[in]      Value:写入参数值
 * @param[in]      Value_type:写入参数数据类型
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
 * @brief          提取电机回复帧扩展ID中的电机CANID
 * @param[in]      CAN_ID_Frame:电机回复帧中的扩展CANID
 * @retval         电机CANID
 */
uint32_t MIMotorGetID(uint32_t CAN_ID_Frame)
{
  return (CAN_ID_Frame & 0xFFFF) >> 8;
}

/**
 * @brief          电机回复帧数据处理函数
 * @param[in]      Motor:对应控制电机结构体
 * @param[in]      DataFrame:数据帧
 * @param[in]      IDFrame:扩展ID帧
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
 * @brief          小米电机ID检查
 * @param[in]      id:  对应控制电机结构体
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
 * @brief          使能小米电机
 * @param[in]      Motor:对应控制电机结构体
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
 * @brief          停止电机
 * @param[in]      Motor:对应控制电机结构体
 * @param[in]      clear_error:清除错误位（0 不清除 1清除）
 * @retval         None
 */
void MIMotorStop(MIMotorInstance *Motor, uint8_t clear_error)
{
  for (uint8_t i = 0; i < 8; ++i)
    Motor->motor_can_instace->tx_buff[i] = 0;
  Motor->motor_can_instace->tx_buff[0] = clear_error; // 清除错误位设置
  Motor->motor_can_instace->txconf.ExtId = MI_Communication_Type_MotorStop << 24 | Master_CAN_ID << 8 | Motor->measure.CAN_ID;
  CANTransmit(Motor->motor_can_instace, 0.5);
}

/**
 * @brief          设置电机模式(必须停止时调整！)
 * @param[in]      Motor:  电机结构体
 * @param[in]      Mode:   电机工作模式（1.运动模式Motion_mode 2. 位置模式Position_mode 3. 速度模式Speed_mode 4. 电流模式Current_mode）
 * @retval         none
 */
void MIMotorSetMode(MIMotorInstance *Motor, uint8_t Mode)
{
  MIMotorSetParameter(Motor, MI_Run_mode, Mode, 's');
}

/**
 * @brief          电流控制模式下设置电流
 * @param[in]      Motor:  电机结构体
 * @param[in]      Current:电流设置
 * @retval         none
 */
void MIMotorSetCurrent(MIMotorInstance *Motor, float Current)
{
  MIMotorSetParameter(Motor, MI_Iq_Ref, Current, 'f');
}

/**
 * @brief          设置电机零点
 * @param[in]      Motor:  电机结构体
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
 * @brief          设置电机CANID
 * @param[in]      Motor:  电机结构体
 * @param[in]      Motor:  设置新ID
 * @retval         none
 */
void MIMotorSetCANID(MIMotorInstance *Motor, uint8_t CAN_ID)
{
  for (uint8_t i = 0; i < 8; ++i)
    Motor->motor_can_instace->tx_buff[i] = 0;
  Motor->motor_can_instace->txconf.ExtId = MI_Communication_Type_CanID << 24 | CAN_ID << 16 | Master_CAN_ID << 8 | Motor->measure.CAN_ID;
  Motor->measure.CAN_ID = CAN_ID; // 将新的ID导入电机结构体
  CANTransmit(Motor->motor_can_instace, 0.5);
}

/**
 * @brief          小米电机初始化
 * @param[in]      Motor:  电机结构体
 * @param[in]      Can_Id: 小米电机ID(默认0x7F)
 * @param[in]      Motor_Num: 电机编号
 * @param[in]      mode: 电机工作模式（0.运动模式Motion_mode 1. 位置模式Position_mode 2. 速度模式Speed_mode 3. 电流模式Current_mode）
 * @retval         none
 */
void _MIMotorInit(MIMotorInstance *Motor, uint8_t mode)
{
  Motor->motor_can_instace->txconf.StdId = 0;          // 配置CAN发送：标准帧清零
  Motor->motor_can_instace->txconf.ExtId = 0;          // 配置CAN发送：扩展帧清零
  Motor->motor_can_instace->txconf.IDE = CAN_ID_EXT;   // 配置CAN发送：扩展帧
  Motor->motor_can_instace->txconf.RTR = CAN_RTR_DATA; // 配置CAN发送：数据帧
  Motor->motor_can_instace->txconf.DLC = 0x08;         // 配置CAN发送：数据长度

  Motor->measure.CAN_ID = MIMotorGetID(Motor->motor_can_instace->rx_id);       // ID设置
  MIMotorSetMode(Motor, mode);          // 设置电机模式
  MIMotorEnable(Motor);                 // 使能电机
}

/**
 * @brief          小米运控模式指令
 * @param[in]      Motor:  目标电机结构体
 * @param[in]      torque: 力矩设置[-12,12] N*M
 * @param[in]      MechPosition: 位置设置[-12.5,12.5] rad
 * @param[in]      speed: 速度设置[-30,30] rpm
 * @param[in]      kp: 比例参数设置
 * @param[in]      kd: 微分参数设置
 * @retval         none
 */
void MIMotorControlMode(MIMotorInstance *Motor, float torque, float MechPosition, float speed, float kp, float kd)
{
  // 装填发送数据
  Motor->motor_can_instace->tx_buff[0] = float_to_uint(MechPosition, MI_P_MIN, MI_P_MAX, 16) >> 8;
  Motor->motor_can_instace->tx_buff[1] = float_to_uint(MechPosition, MI_P_MIN, MI_P_MAX, 16);
  Motor->motor_can_instace->tx_buff[2] = float_to_uint(speed, MI_V_MIN, MI_V_MAX, 16) >> 8;
  Motor->motor_can_instace->tx_buff[3] = float_to_uint(speed, MI_V_MIN, MI_V_MAX, 16);
  Motor->motor_can_instace->tx_buff[4] = float_to_uint(kp, MI_KP_MIN, MI_KP_MAX, 16) >> 8;
  Motor->motor_can_instace->tx_buff[5] = float_to_uint(kp, MI_KP_MIN, MI_KP_MAX, 16);
  Motor->motor_can_instace->tx_buff[6] = float_to_uint(kd, MI_KD_MIN, MI_KD_MAX, 16) >> 8;
  Motor->motor_can_instace->tx_buff[7] = float_to_uint(kd, MI_KD_MIN, MI_KD_MAX, 16);
  Motor->motor_can_instace->txconf.ExtId = MI_Communication_Type_MotionControl << 24 | float_to_uint(torque, MI_T_MIN, MI_T_MAX, 16) << 8 | Motor->measure.CAN_ID; // 装填扩展帧数据
  CANTransmit(Motor->motor_can_instace, 0.5);
}

/*****************************回调函数 负责接回传信息 可转移至别处*****************************/
/**
 * @brief          小米电机接收回调函数,接收电机数据
 * @param[in]      motor_can:CAN实例
 * @retval         none
 */
static void MIMotorDecode(CANInstance *motor_can)
{
  uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
  uint8_t *rxbuff = motor_can->rx_buff;
  MIMotorInstance *motor = (MIMotorInstance *)motor_can->id;
  MIMotor_Measure_t *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

  DaemonReload(motor->motor_daemon);
  measure->feed_dt = DWT_GetDeltaT(&measure->feed_cnt);
  
  // if (motor_can->rxconf.ExtId >> 24 != 0) // 检查是否为广播模式
  if (1)
    MIMotorDataHandler(motor, motor->motor_can_instace->rx_buff, motor_can->rx_id);
  else
    motor->measure.MCU_ID = motor->motor_can_instace->rx_buff[0];
}

/**
 * @brief          电机丢失回调函数,可写处理丢失状态的方法
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
    config->can_init_config.rx_id = (config->can_init_config.rx_id << 8) | 0x02000000;//查找所有用到motor->motor_can_instace->rx_id的地方，必须改为motor->measure.CAN_ID
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = MIMotorLostCallback,
        .owner_id = motor,
        .reload_count = 5, // 20ms
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DWT_Delay(0.1);
    _MIMotorInit(motor, MI_Motion_mode);  //这里设置成运控模式，用小米自带的PID算法，不用自己算我们只需要在任务里面给定目标速度即可
    DWT_Delay(0.1);

    mi_motor_instance[idx++] = motor;
    return motor;
}

/**
 * @brief          外部设置小米电机参考值，这里由于小米电机自带PID算法，因此我们直接把这个值赋成想要的速度即可
 * @param[in]      motor:
 * @param[in]      ref:
 * @retval         none
 */
void MIMotorSetRef(MIMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

/**
 * @brief 为了避免总线堵塞,为每个电机创建一个发送任务
 * @param argument 传入的电机指针
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
            // measure单位是rad,ref是角度,统一到angle下计算,方便建模
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
            // measure单位是rad / s ,ref是angle per sec,统一到angle下计算
            pid_ref = PIDCalculate(&motor->speed_PID, pid_measure * RAD_2_DEGREE, pid_ref);
        }

        // 小米电机不反馈电流值
        // if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
        //     pid_ref += *motor->current_feedforward_ptr;
        // if (setting->close_loop_type & CURRENT_LOOP)
        // {
        //     pid_ref = PIDCalculate(&motor->current_PID, measure->real_current, pid_ref);
        // }

        set = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;

        MIMotorControlMode(motor, 0, 0, set, motor->speed_PID.Kp, motor->speed_PID.Kd);//这里我们只需要给出目标速度即可(-30rad/s ~ 30red/s)

        osDelay(1);
    }
}

void MIMotorControlInit(void)
{
    char mi_task_name[5] = "mi";
    // 遍历所有电机实例,创建任务
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
