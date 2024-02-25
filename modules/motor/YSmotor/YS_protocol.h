#ifndef __YS_PROTOCOL_H
#define __YS_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>

#define PROTOCOL_TM_ID1 0XFE
#define PROTOCOL_TM_ID2 0XEE
#define PROTOCOL_RM_ID1 0XFD
#define PROTOCOL_RM_ID2 0XEE

#define OFFSET_BYTE 8 // 出数据段外，其他部分所占字节数

typedef struct
{
	struct
	{
		uint8_t sof[2];
		uint8_t crc_check; // 帧头CRC校验
	} header;			   // 数据帧头
	uint16_t frame_tail;   // 帧尾CRC校验
} protocol_struct;

typedef struct
{
	uint8_t cmd_id :4;	   // 电机idID
	uint8_t status :3;     //电机状态
	uint8_t none  :1;      //保留位
	
} Moto_mode;

typedef struct
{
    int16_t tor_des;        // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des;        // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des;        // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;          // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;          // 期望关节阻尼系数 unit: -1.0-1.0 (q15)
    
} RIS_Comd_t;   // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 * 
 */
typedef struct
{
    int16_t  torque;        // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t  speed;         // 实际关节输出速度 unit: rad/s   (q8)
    int32_t  pos;           // 实际关节输出位置 unit: rad     (q15)
    int8_t   temp;          // 电机温度: -128~127°C
    uint8_t  MError :3;     // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force  :12;    // 足端气压传感器数据 12bit (0-4095)
    uint8_t  none   :1;     // 保留位
} RIS_Fbk_t;   // 状态数据 11Byte


#endif
