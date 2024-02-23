#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include "usart.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "bsp_usart.h"
#include "FreeRTOS.h"
#include "super_cap.h"
#include "balance.h"
extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
	uint8_t Robot_Color;		// 机器人颜色
	uint16_t Robot_ID;			// 本机器人ID
	uint16_t Cilent_ID;			// 本机器人对应的客户端ID
	uint16_t Receiver_Robot_ID; // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
	referee_id_t referee_id;

	xFrameHeader FrameHeader; // 接收到的帧头信息
	uint16_t CmdID;
	ext_game_state_t GameState;							   // 0x0001
	ext_game_result_t GameResult;						   // 0x0002
	ext_game_robot_HP_t GameRobotHP;					   // 0x0003
	ext_event_data_t EventData;							   // 0x0101
	ext_supply_projectile_action_t SupplyProjectileAction; // 0x0102
	ext_game_robot_state_t GameRobotState;				   // 0x0201
	ext_power_heat_data_t PowerHeatData;				   // 0x0202
	ext_game_robot_pos_t GameRobotPos;					   // 0x0203
	ext_buff_musk_t BuffMusk;							   // 0x0204
	aerial_robot_energy_t AerialRobotEnergy;			   // 0x0205
	ext_robot_hurt_t RobotHurt;							   // 0x0206
	ext_shoot_data_t ShootData;							   // 0x0207

	// 自定义交互数据的接收
	Communicate_ReceiveData_t ReceiveData;

} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
	uint8_t lid_flag : 1;
	uint8_t friction_flag : 1;
	uint8_t loader_flag : 1;
	uint8_t target_flag : 1;
	uint8_t chassis_flag : 1;
	uint8_t cap_flag : 1;
	uint8_t leg_flag : 1;
	uint8_t direction_flag : 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
	Referee_Interactive_Flag_t Referee_Interactive_Flag;
	// 为UI绘制以及交互数据所用
	lid_mode_e lid_mode;		   // 弹舱盖
	friction_mode_e friction_mode; // 摩擦轮
	loader_mode_e loader_mode;	   // 拨盘
	chassis_mode_e chassis_mode;   // 底盘模式

	Target_State_e target_state; // 自瞄
	SuperCap_Msg_s cap_msg;		 // 超级电容信息
	float coord[6];
	chassis_direction_e direction; // 底盘方向

	// 上一次的模式，用于flag判断
	lid_mode_e lid_last_mode;			// 弹舱盖
	friction_mode_e friction_last_mode; // 摩擦轮
	Target_State_e target_last_state;	// 自瞄
	chassis_mode_e chassis_last_mode;	// 底盘模式
	loader_mode_e loader_last_mode;		// 拨盘
	SuperCap_Msg_s cap_last_msg;		// 超级电容信息
	chassis_direction_e last_direction; // 底盘方向

	uint8_t refresh_flag;

} Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

#endif // !REFEREE_H
