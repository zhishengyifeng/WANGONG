/**
 * @file YSmotor.c
 * @author ZiFeng
 * @brief  module for YSmotor
 * @version beta
 * @date 2024-02-24
 * @copyright Copyright (c) 2024
 *
 */
#include "moto_control.h"
#include "bsp_usart.h"
#include "usart.h"
#include "YS_protocol.h"
#include "daemon.h"

static MotorData_t recv_data;
static USARTInstance *YS_usart_instance;
static DaemonInstance *YS_daemon_instance; 
/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeYS(MOTOR_recv* recv)
{
    uint16_t flag_register;
    DaemonReload(YS_daemon_instance); // 喂狗
    extract_data(recv);
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
    USARTServiceInit(YS_usart_instance);
}

/* 视觉通信初始化 */
MOTOR_recv *YSInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeYS;
    conf.recv_buff_size = 16;
    conf.usart_handle = _handle;
    YS_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = YS_usart_instance,
        .reload_count = 10,
    };
    YS_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void YSSend(MOTOR_send *send)
{
    modify_data(send);
    
	SET_485_DE_UP();
	SET_485_RE_UP();

    HAL_UART_Transmit(&huart1, (uint8_t *)send, sizeof(send->motor_send_data), 10);
}