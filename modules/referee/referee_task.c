/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
static void DeterminRobotID(void);                   // 判断各种ID，选择客户端ID
static void My_UI_Refresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void Mode_Change_Check(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
static void robot_mode_change(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化
referee_info_t *Referee_Interactive_init(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    return referee_recv_info;
}

void Referee_Interactive_task()
{
    // robot_mode_change(Interactive_data); // 测试用函数，实现模式自动变化
    My_UI_Refresh(referee_recv_info, Interactive_data);
}

/* 1-4图层给静态 5-8给动态 */

/*  左侧车辆状态
    lid_mode_e lid_mode;		            ss0 sd0
    friction_mode_e friction_mode;          ss1
    loader_mode_e loader_mode;	            ss2
 */
static String_Data_t Mode_String_sta[4];
static String_Data_t Mode_String_dyn[4];

/*  静态标线
    命名规则：ls0
*/
// static Graph_Data_t Shootline_Graph_sta[10];
// static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};

/*  电容状态，
    命名规则：es0 sd0
    "Power:"：es0
    外框：es1
*/
static String_Data_t Energy_String_sta[3];
static String_Data_t Energy_String_dyn[3];
static Graph_Data_t Energy_Graph_sta[3];
static Graph_Data_t Energy_Graph_dyn[3];

/*  视觉内容
    命名规则：vs0 vd0
    视觉外框：vs0
    视觉准信：vd0

 */
static Graph_Data_t Vision_Graph_sta[1];
static Graph_Data_t Vision_Graph_dyn[1];

/*  轮腿内容
    命名规则：ws wd
    底盘方向：ws0 wd0
    腿部运动： wd1-5

 */
#define Leg_X_Offset 1650
#define Leg_Y_Offset 500
#define Leg_Gain 500
static Graph_Data_t Leg_Graph_sta[2];
static Graph_Data_t Leg_Graph_dyn[6];
static uint32_t Processed_coord[6] = {1605, 455, 1695, 410, 1785, 455}; // Y坐标取反，放大，平移后的

void My_UI_init()
{
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100);
    DeterminRobotID();
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI
    /***********************************************************************静态部分********************************************************************************/
    // // 绘制发射基准线  Shootline_Graph_sta
    // Line_Draw(&Shootline_Graph_sta[0], "ls0", UI_Graph_ADD, 1, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    // Line_Draw(&Shootline_Graph_sta[1], "ls1", UI_Graph_ADD, 1, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
    // Line_Draw(&Shootline_Graph_sta[2], "ls2", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
    // Line_Draw(&Shootline_Graph_sta[3], "ls3", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
    // Line_Draw(&Shootline_Graph_sta[4], "ls4", UI_Graph_ADD, 1, UI_Color_Yellow, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
    // UI_ReFresh(&referee_recv_info->referee_id, 5, Shootline_Graph_sta[0], Shootline_Graph_sta[1], Shootline_Graph_sta[2], Shootline_Graph_sta[3], Shootline_Graph_sta[4]);

    // 左侧车辆状态
    Char_Draw(&Mode_String_sta[0], "ss0", UI_Graph_ADD, 2, UI_Color_Main, 15, 2, 150, 750, "lid:");
    Char_ReFresh(&referee_recv_info->referee_id, Mode_String_sta[0]);
    Char_Draw(&Mode_String_sta[1], "ss1", UI_Graph_ADD, 2, UI_Color_Yellow, 15, 2, 150, 700, "frict:");
    Char_ReFresh(&referee_recv_info->referee_id, Mode_String_sta[1]);
    Char_Draw(&Mode_String_sta[2], "ss2", UI_Graph_ADD, 2, UI_Color_Orange, 15, 2, 150, 650, "loader:");
    Char_ReFresh(&referee_recv_info->referee_id, Mode_String_sta[2]);
    Char_Draw(&Mode_String_sta[3], "ss3", UI_Graph_ADD, 2, UI_Color_Purplish_red, 15, 2, 150, 600, "chassis:");
    Char_ReFresh(&referee_recv_info->referee_id, Mode_String_sta[3]);

    // 底部功率显示
    Char_Draw(&Energy_String_sta[0], "es0", UI_Graph_ADD, 2, UI_Color_Green, 18, 2, 720, 210, "Power:");
    Char_ReFresh(&referee_recv_info->referee_id, Energy_String_sta[0]);
    // 底部能量条框
    Rectangle_Draw(&Energy_Graph_sta[0], "es1", UI_Graph_ADD, 2, UI_Color_Green, 2, 720, 140, 1220, 180);
    UI_ReFresh(&referee_recv_info->referee_id, 1, Energy_Graph_sta[0]);

    // 视觉外框
    Rectangle_Draw(&Vision_Graph_sta[0], "vs0", UI_Graph_ADD, 3, UI_Color_Cyan, 2, 660, 240, 1260, 840);
    UI_ReFresh(&referee_recv_info->referee_id, 1, Vision_Graph_sta[0]);

    // 底盘方向--指示云台方向
    Line_Draw(&Leg_Graph_sta[0], "ws0", UI_Graph_ADD, 3, UI_Color_Purplish_red, 4, 1700, 600, 1700, 705);
    UI_ReFresh(&referee_recv_info->referee_id, 1, Leg_Graph_sta[0]);

    /***********************************************************************动态部分********************************************************************************/
    // 左侧车辆状态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    Char_Draw(&Mode_String_dyn[0], "sd0", UI_Graph_ADD, 5, UI_Color_Main, 15, 2, 270, 750, "open ");
    Char_ReFresh(&referee_recv_info->referee_id, Mode_String_dyn[0]);
    Char_Draw(&Mode_String_dyn[1], "sd1", UI_Graph_ADD, 5, UI_Color_Yellow, 15, 2, 270, 700, "off");
    Char_ReFresh(&referee_recv_info->referee_id, Mode_String_dyn[1]);
    Char_Draw(&Mode_String_dyn[2], "sd2", UI_Graph_ADD, 5, UI_Color_Orange, 15, 2, 270, 650, "stop ");
    Char_ReFresh(&referee_recv_info->referee_id, Mode_String_dyn[2]);
    Char_Draw(&Mode_String_dyn[3], "sd3", UI_Graph_ADD, 5, UI_Color_Purplish_red, 15, 2, 270, 600, "reset");
    Char_ReFresh(&referee_recv_info->referee_id, Mode_String_dyn[3]);

    // 底盘功率显示
    Float_Draw(&Energy_Graph_dyn[0], "ed0", UI_Graph_ADD, 5, UI_Color_Green, 18, 2, 2, 820, 210, 24000);
    // 能量条初始状态
    Line_Draw(&Energy_Graph_dyn[1], "ed1", UI_Graph_ADD, 5, UI_Color_Pink, 30, 720, 160, 1020, 160);
    UI_ReFresh(&referee_recv_info->referee_id, 2, Energy_Graph_dyn[0], Energy_Graph_dyn[1]);

    // 视觉
    Circle_Draw(&Vision_Graph_dyn[0], "vd0", UI_Graph_ADD, 6, UI_Color_White, 4, 960, 540, 20);
    UI_ReFresh(&referee_recv_info->referee_id, 1, Vision_Graph_dyn[0]);

    // 底盘方向
    Line_Draw(&Leg_Graph_dyn[0], "wd0", UI_Graph_ADD, 6, UI_Color_Green, 7, 1650, 600, 1750, 600);
    UI_ReFresh(&referee_recv_info->referee_id, 1, Leg_Graph_dyn[0]);

    // 腿部运动，五连杆
    // Line_Draw(&Leg_Graph_sta[1], "wd1", UI_Graph_ADD, 3, UI_Color_Yellow, 5, 1620, 500, 1780, 500);//水平线
    Line_Draw(&Leg_Graph_dyn[1], "wd1", UI_Graph_ADD, 7, UI_Color_Yellow, 5,
              0 + Leg_X_Offset, 0 + Leg_Y_Offset, (uint32_t)(0.108 * Leg_Gain + Leg_X_Offset), 0 + Leg_Y_Offset); // 水平线
    Line_Draw(&Leg_Graph_dyn[2], "wd2", UI_Graph_ADD, 7, UI_Color_Green, 5,
              0 + Leg_X_Offset, 0 + Leg_Y_Offset, Processed_coord[0], Processed_coord[1]); // 左大腿
    Line_Draw(&Leg_Graph_dyn[3], "wd3", UI_Graph_ADD, 7, UI_Color_Green, 5,
              Processed_coord[0], Processed_coord[1], Processed_coord[2], Processed_coord[3]); // 左小腿
    Line_Draw(&Leg_Graph_dyn[4], "wd4", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5,
              (uint32_t)(0.108 * Leg_Gain + Leg_X_Offset), 0 + Leg_Y_Offset, Processed_coord[4], Processed_coord[5]); // 右大腿
    Line_Draw(&Leg_Graph_dyn[5], "wd5", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5,
              Processed_coord[2], Processed_coord[3], Processed_coord[4], Processed_coord[5]); // 右小腿
    UI_ReFresh(&referee_recv_info->referee_id, 5, Leg_Graph_dyn[1], Leg_Graph_dyn[2], Leg_Graph_dyn[3], Leg_Graph_dyn[4], Leg_Graph_dyn[5]);
}

static void My_UI_Refresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->refresh_flag)
    {
        _Interactive_data->refresh_flag = 0;
        My_UI_init(referee_recv_info);
    }
    Mode_Change_Check(_Interactive_data);
    /* 左侧车辆状态 */
    // lid
    if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1)
    {
        Char_Draw(&Mode_String_dyn[0], "sd0", UI_Graph_Change, 5, UI_Color_Main, 15, 2, 270, 750,
                  _Interactive_data->lid_mode == LID_OPEN ? "open " : "close");
        Char_ReFresh(&referee_recv_info->referee_id, Mode_String_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    }

    // friction
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        Char_Draw(&Mode_String_dyn[1], "sd1", UI_Graph_Change, 5, UI_Color_Yellow, 15, 2, 270, 700,
                  _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
        Char_ReFresh(&referee_recv_info->referee_id, Mode_String_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    }

    // loader
    if (_Interactive_data->Referee_Interactive_Flag.loader_flag == 1)
    {
        switch (_Interactive_data->loader_mode)
        {
        case LOAD_STOP:
        {
            Char_Draw(&Mode_String_dyn[2], "sd2", UI_Graph_Change, 5, UI_Color_Orange, 15, 2, 270, 650, "stop ");
            break;
        }
        case LOAD_REVERSE:
        {
            Char_Draw(&Mode_String_dyn[2], "sd2", UI_Graph_Change, 5, UI_Color_Orange, 15, 2, 270, 650, "rever");
            break;
        }
        case LOAD_1_BULLET:
        {
            Char_Draw(&Mode_String_dyn[2], "sd2", UI_Graph_Change, 5, UI_Color_Orange, 15, 2, 270, 650, "1bull");
            break;
        }
        case LOAD_3_BULLET:
        {
            Char_Draw(&Mode_String_dyn[2], "sd2", UI_Graph_Change, 5, UI_Color_Orange, 15, 2, 270, 650, "3bull");
            break;
        }
        case LOAD_BURSTFIRE:
        {
            Char_Draw(&Mode_String_dyn[2], "sd2", UI_Graph_Change, 5, UI_Color_Orange, 15, 2, 270, 650, "brust");
            break;
        }
        }
        Char_ReFresh(&referee_recv_info->referee_id, Mode_String_dyn[2]);
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 0;
    }

    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
        case CHASSIS_ROTATE:
        {
            Char_Draw(&Mode_String_dyn[3], "sd3", UI_Graph_Change, 5, UI_Color_Purplish_red, 15, 2, 270, 600, "rotate");
            break;
        }
        case CHASSIS_RESET:
        {
            Char_Draw(&Mode_String_dyn[3], "sd3", UI_Graph_Change, 5, UI_Color_Purplish_red, 15, 2, 270, 600, "reset ");
            break;
        }
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        {
            Char_Draw(&Mode_String_dyn[3], "sd3", UI_Graph_Change, 5, UI_Color_Purplish_red, 15, 2, 270, 650, "follow");
            break;
        }
        }
        Char_ReFresh(&referee_recv_info->referee_id, Mode_String_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }

    // 视觉
    if (_Interactive_data->Referee_Interactive_Flag.target_flag == 1)
    {
        switch (_Interactive_data->target_state)
        {
        case NO_TARGET:
        {
            Circle_Draw(&Vision_Graph_dyn[0], "vd0", UI_Graph_Change, 6, UI_Color_White, 4, 960, 540, 20);
            break;
        }
        case TARGET_CONVERGING:
        {
            Circle_Draw(&Vision_Graph_dyn[0], "vd0", UI_Graph_Change, 6, UI_Color_Yellow, 4, 960, 540, 20);
            break;
        }
        case READY_TO_FIRE:
        {
            Circle_Draw(&Vision_Graph_dyn[0], "vd0", UI_Graph_Change, 6, UI_Color_Green, 4, 960, 540, 20);
            break;
        }
        }
        UI_ReFresh(&referee_recv_info->referee_id, 1, Vision_Graph_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.target_flag = 0;
    }

    // 电容
    if (_Interactive_data->Referee_Interactive_Flag.cap_flag == 1)
    {
        Float_Draw(&Energy_Graph_dyn[0], "ed0", UI_Graph_Change, 5, UI_Color_Green, 18, 2, 2, 820, 210, _Interactive_data->cap_msg.vol);
        Line_Draw(&Energy_Graph_dyn[1], "ed1", UI_Graph_Change, 5, UI_Color_Pink, 30, 720, 160, (uint32_t)1020 + _Interactive_data->cap_msg.vol * 30, 160);
        UI_ReFresh(&referee_recv_info->referee_id, 2, Energy_Graph_dyn[0], Energy_Graph_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.cap_flag = 0;
    }

    // 底盘方向
    if (_Interactive_data->Referee_Interactive_Flag.direction_flag == 1)
    {
        switch (_Interactive_data->direction)
        {
        case CHASSIS_SIDLE:
        {
            Line_Draw(&Leg_Graph_dyn[0], "wd0", UI_Graph_Change, 6, UI_Color_Green, 7, 1700, 600, 1700, 700);
            break;
        }
        case CAHSSIS_ALIGN:
        {
            Line_Draw(&Leg_Graph_dyn[0], "wd0", UI_Graph_Change, 6, UI_Color_Green, 7, 1650, 600, 1750, 600);
            break;
        }
        }
        UI_ReFresh(&referee_recv_info->referee_id, 1, Leg_Graph_dyn[0]);
        _Interactive_data->Referee_Interactive_Flag.direction_flag = 0;
    }
    if (Processed_coord[0] != (uint32_t)(_Interactive_data->coord[0] * Leg_Gain + Leg_X_Offset))
    {
        Processed_coord[0] = (uint32_t)(_Interactive_data->coord[0] * Leg_Gain + Leg_X_Offset);
        Processed_coord[1] = (uint32_t)(_Interactive_data->coord[1] * -Leg_Gain + Leg_Y_Offset);
        Processed_coord[2] = (uint32_t)(_Interactive_data->coord[2] * Leg_Gain + Leg_X_Offset);
        Processed_coord[3] = (uint32_t)(_Interactive_data->coord[3] * -Leg_Gain + Leg_Y_Offset);
        Processed_coord[4] = (uint32_t)(_Interactive_data->coord[4] * Leg_Gain + Leg_X_Offset);
        Processed_coord[5] = (uint32_t)(_Interactive_data->coord[5] * -Leg_Gain + Leg_Y_Offset);

        // 腿部运动，五连杆，不需要检测变更，实时显示变化
        Line_Draw(&Leg_Graph_dyn[2], "wd2", UI_Graph_Change, 7, UI_Color_Green, 5,
                  0 + Leg_X_Offset, 0 + Leg_Y_Offset, Processed_coord[0], Processed_coord[1]); // 左大腿

        Line_Draw(&Leg_Graph_dyn[3], "wd3", UI_Graph_Change, 7, UI_Color_Green, 5,
                  Processed_coord[0], Processed_coord[1], Processed_coord[2], Processed_coord[3]); // 左小腿

        Line_Draw(&Leg_Graph_dyn[4], "wd4", UI_Graph_Change, 7, UI_Color_Purplish_red, 5,
                  (uint32_t)(0.108 * Leg_Gain + Leg_X_Offset), 0 + Leg_Y_Offset, Processed_coord[4], Processed_coord[5]); // 右大腿

        Line_Draw(&Leg_Graph_dyn[5], "wd5", UI_Graph_Change, 7, UI_Color_Purplish_red, 5,
                  Processed_coord[2], Processed_coord[3], Processed_coord[4], Processed_coord[5]); // 右小腿
        UI_ReFresh(&referee_recv_info->referee_id, 5, Leg_Graph_dyn[1], Leg_Graph_dyn[2], Leg_Graph_dyn[3], Leg_Graph_dyn[4], Leg_Graph_dyn[5]);
    }
}

static uint8_t count = 0;
static uint16_t count1 = 0;
static void robot_mode_change(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    count++;
    if (count >= 50)
    {
        count = 0;
        count1++;
    }
    switch (count1 % 4)
    {
    case 0:
    {
        _Interactive_data->lid_mode = LID_OPEN;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->loader_mode = LOAD_STOP;
        _Interactive_data->target_state = NO_TARGET;
        _Interactive_data->direction = CAHSSIS_ALIGN;

        _Interactive_data->cap_msg.vol += 3.5;
        if (_Interactive_data->cap_msg.vol >= 18)
            _Interactive_data->cap_msg.vol = 0;
        break;
    }
    case 1:
    {
        _Interactive_data->lid_mode = LID_CLOSE;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->loader_mode = LOAD_REVERSE;
        _Interactive_data->target_state = TARGET_CONVERGING;
        _Interactive_data->direction = CHASSIS_SIDLE;
        break;
    }
    case 2:
    {
        _Interactive_data->lid_mode = LID_OPEN;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->loader_mode = LOAD_1_BULLET;
        _Interactive_data->target_state = READY_TO_FIRE;
        _Interactive_data->direction = CAHSSIS_ALIGN;
        break;
    }
    case 3:
    {
        _Interactive_data->lid_mode = LID_CLOSE;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->loader_mode = LOAD_3_BULLET;
        _Interactive_data->target_state = NO_TARGET;
        _Interactive_data->direction = CHASSIS_SIDLE;
        break;
    }
    default:
        break;
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void Mode_Change_Check(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->lid_mode != _Interactive_data->lid_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.lid_flag = 1;
        _Interactive_data->lid_last_mode = _Interactive_data->lid_mode;
    }
    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }
    if (_Interactive_data->loader_mode != _Interactive_data->loader_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 1;
        _Interactive_data->loader_last_mode = _Interactive_data->loader_mode;
    }
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }
    if (_Interactive_data->target_state != _Interactive_data->target_last_state)
    {
        _Interactive_data->Referee_Interactive_Flag.target_flag = 1;
        _Interactive_data->target_last_state = _Interactive_data->target_state;
    }
    if (_Interactive_data->cap_msg.vol != _Interactive_data->cap_last_msg.vol)
    {
        _Interactive_data->Referee_Interactive_Flag.cap_flag = 1;
        _Interactive_data->cap_last_msg.vol = _Interactive_data->cap_msg.vol;
    }
    if (_Interactive_data->direction != _Interactive_data->last_direction)
    {
        _Interactive_data->Referee_Interactive_Flag.direction_flag = 1;
        _Interactive_data->last_direction = _Interactive_data->direction;
    }
}

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}
