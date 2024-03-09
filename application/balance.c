// app
#include "balance.h"
#include "speed_estimation.h"
#include "linkNleg.h"
#include "fly_detection.h"
#include "robot_def.h"
#include "general_def.h"
#include "ins_task.h"
#include "HT04.h"
#include "LK9025.h"
#include "controller.h"
#include "can_comm.h"
#include "super_cap.h"
#include "user_lib.h"
#include "remote_control.h"
#include "referee_task.h"
#include "stdint.h"
#include "arm_math.h" // 需要用到较多三角函数
#include "bsp_dwt.h"
#include "bsp_log.h"

static uint32_t balance_dwt_cnt;
static float del_t;

/* 底盘拥有的模块实例 */
static INS_t *imu_data;
static RC_ctrl_t *rc_data; // 调试用
static Referee_Interactive_info_t my_ui;
static referee_info_t *referee_data;
static Chassis_Ctrl_Cmd_s chassis_cmd_recv; // syh
static Chassis_Upload_Data_s chassis_feed;
static CANCommInstance *ci;
static SuperCapInstance *cap; // syh
// 四个关节电机和两个驱动轮电机
static HTMotorInstance *lf, *lb, *rf, *rb, *joint[4]; // 指针数组方便传参和调试
static LKMotorInstance *l_driven, *r_driven, *driven[2];
// 两个腿的参数,0为左腿,1为右腿
static LinkNPodParam l_side, r_side; // syh  phi5
static ChassisParam chassis;
// 综合运动补偿的PID控制器
static PIDInstance steer_p_pid, steer_v_pid;   // 转向PID,有转向指令时使用IMU的加速度反馈积分以获取速度和位置状态量
static PIDInstance anti_crash_pid, phi5_pid;   // 抗劈叉,将输出以相反的方向叠加到左右腿的上
static PIDInstance leglen_pid_l, leglen_pid_r; // 用PD模拟弹簧,不要积分(弹簧是无积分二阶系统),增益不可过大否则抗外界冲击响应时太"硬"
static PIDInstance legdot_pid_l, legdot_pid_r;
static PIDInstance roll_compensate_pid, rolldot_pid, roll_comp_pid; // roll轴补偿,用于保持机体水平

static Robot_Status_e chassis_status;

void BalanceInit()
{
    referee_data = Referee_Interactive_init(&huart6, &my_ui);
    rc_data = RemoteControlInit(&huart3);
    CANComm_Init_Config_s commconf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x40,
            .rx_id = 0x41},
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s)};
    ci = CANCommInit(&commconf);
    imu_data = INS_Init();

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x302, // todo 电容id
            .rx_id = 0x301}};
    cap = SuperCapInit(&cap_conf);

    Motor_Init_Config_s joint_conf = {
        // 写一个,剩下的修改方向和id即可
        .can_init_config = {
            .can_handle = &hcan1},
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.3,
                .Kd = 0.1,
                .Ki = 0,
                .DeadBand = 0.0001,
                .Improve = PID_DerivativeFilter,
                .MaxOut = 4,
                .Derivative_LPF_RC = 0.05,
            }, // 仅用于复位腿
        },
        .controller_setting_init_config = {
            .close_loop_type = ANGLE_LOOP,
            .outer_loop_type = OPEN_LOOP,
            .motor_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
        },
        .motor_type = HT04};
    joint_conf.can_init_config.tx_id = 4;
    joint_conf.can_init_config.rx_id = 14;
    joint[LF] = lf = HTMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 3;
    joint_conf.can_init_config.rx_id = 13;
    joint[LB] = lb = HTMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 2;
    joint_conf.can_init_config.rx_id = 12;
    joint[RF] = rf = HTMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 1;
    joint_conf.can_init_config.rx_id = 11;
    joint[RB] = rb = HTMotorInit(&joint_conf);

    Motor_Init_Config_s driven_conf = {
        // 写一个,剩下的修改方向和id即可
        .can_init_config.can_handle = &hcan2,
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = LK9025,
    };
    driven_conf.can_init_config.tx_id = 1;
    driven[LD] = l_driven = LKMotorInit(&driven_conf);
    driven_conf.can_init_config.tx_id = 2;
    driven[RD] = r_driven = LKMotorInit(&driven_conf);

    PID_Init_Config_s steer_p_pid_conf = {
        .Kp = 2,
        .Kd = 1,
        .Ki = 0.0f,
        .MaxOut = 4,
        .DeadBand = 0.01f,
        .Improve = PID_DerivativeFilter,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&steer_p_pid, &steer_p_pid_conf);
    PID_Init_Config_s steer_v_pid_conf = {
        .Kp = 2,
        .Kd = 0.0f,
        .Ki = 0.0f,
        .MaxOut = 100,
        .DeadBand = 0.0f,
        .Improve = PID_DerivativeFilter | PID_Integral_Limit,
        .Derivative_LPF_RC = 0.05,
        .IntegralLimit = 2,
    };
    PIDInit(&steer_v_pid, &steer_v_pid_conf);

    PID_Init_Config_s anti_crash_pid_conf = {
        .Kp = 8,
        .Kd = 2.5,
        .Ki = 0.4,
        .MaxOut = 45,
        .DeadBand = 0.01f,
        .Improve = PID_DerivativeFilter | PID_ChangingIntegrationRate | PID_Integral_Limit,
        .Derivative_LPF_RC = 0.05,
        .CoefA = 0.05,
        .CoefB = 0.05,
        .IntegralLimit = 2,
    };
    PIDInit(&anti_crash_pid, &anti_crash_pid_conf);

    PID_Init_Config_s leg_length_pid_conf = {
        .Kp = 450,
        .Kd = 150,
        .Ki = 5,
        .MaxOut = 60,
        .DeadBand = 0.0001f,
        .Improve = PID_ChangingIntegrationRate | PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .CoefA = 0.01,
        .CoefB = 0.02,
        .Derivative_LPF_RC = 0.08,
    };
    PIDInit(&leglen_pid_l, &leg_length_pid_conf);
    PIDInit(&leglen_pid_r, &leg_length_pid_conf);

    PID_Init_Config_s roll_compensate_pid_conf = {
        .Kp = 0.0008f,
        .Kd = 0.00065f,
        .Ki = 0.0f,
        .MaxOut = 0.04,
        .DeadBand = 0.005f,
        .Improve = PID_DerivativeFilter,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&roll_compensate_pid, &roll_compensate_pid_conf);
     PID_Init_Config_s roll_comp_pid_conf = {
        .Kp = 0.8f,
        .Kd = 0.65f,
        .Ki = 0.0f,
        .MaxOut = 1,
        .DeadBand = 0.005f,
        .Improve = PID_DerivativeFilter,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&roll_comp_pid, &roll_comp_pid_conf);


    l_side.target_len = r_side.target_len = 0.23;
    chassis.vel_cov = 1000; // 初始化速度协方差
    chassis_status = ROBOT_READY;
    DWT_GetDeltaT(&balance_dwt_cnt);
}

static void EnableAllMotor() /* 打开所有电机 */
{
    for (uint8_t i = 0; i < JOINT_CNT; i++) // 打开关节电机
        HTMotorEnable(joint[i]);
    for (uint8_t i = 0; i < DRIVEN_CNT; i++) // 打开驱动电机
        LKMotorEnable(driven[i]);
}

/* 切换底盘遥控器控制和云台双板控制 */
static void ControlSwitch()
{  
     if (switch_is_down(rc_data->rc.switch_right) && RemoteControlIsOnline())
    {
         chassis_cmd_recv.chassis_mode = CHASSIS_STOP;
         chassis_status = ROBOT_STOP;
    }else if (switch_is_up(rc_data->rc.switch_right) && RemoteControlIsOnline())
    {
        chassis_cmd_recv.chassis_mode = CHASSIS_ROTATE;
    }else                                                     //switch_is_mid(rc_data->rc.switch_right)
    {
        
        if (chassis_status == ROBOT_STOP)
        {
            chassis_cmd_recv.chassis_mode = CHASSIS_RESET;
        }else
        {
            chassis_cmd_recv.chassis_mode = CHASSIS_FREE_DEBUG;
        };
        
    }
}

/* 腿缩回复位,只允许驱动轮电机移动 */
static void ResetChassis()
{
    EnableAllMotor(); // 打开全部电机,关节复位到起始角度,驱动电机响应速度输入以从墙角或固连中脱身
    // 复位时清空距离和腿长积累量,保证顺利站起
    chassis.dist = chassis.target_dist = 0;
    chassis.target_yaw= chassis.yaw ;
    l_side.target_len = r_side.target_len = 0.24;
    // 撞墙时前后移动保证能重新站立,执行速度输入
    LKMotorSetRef(l_driven, chassis.target_v * 2);
    LKMotorSetRef(r_driven, chassis.target_v * 2);
    // 若关节完成复位,进入ready态
    if (abs(lf->measure.total_angle) < 0.05 && abs(lf->measure.total_angle) > 0.02 &&
        abs(lb->measure.total_angle) < 0.05 && abs(lb->measure.total_angle) > 0.02 &&
        abs(rf->measure.total_angle) < 0.05 && abs(rf->measure.total_angle) > 0.02 &&
        abs(rb->measure.total_angle) < 0.05 && abs(rb->measure.total_angle) > 0.02)
    {
        chassis_status = ROBOT_READY; // 底盘已经准备好重新站立
    }
    else if (abs(lf->measure.total_angle) <= 0.02 &&
             abs(lb->measure.total_angle) <= 0.02 &&
             abs(rf->measure.total_angle) <= 0.02 &&
             abs(rb->measure.total_angle) <= 0.02)
    {                                 // 双阈值保证关节能够复位而不会进入死区
        chassis_status = ROBOT_READY; // 底盘已经准备好重新站立
        for (uint8_t i = 0; i < JOINT_CNT; i++)
            HTMotorOuterLoop(joint[i], OPEN_LOOP); // 改回直接开环扭矩输入,让电调对扭矩闭环
        return;                                    // 退出函数不再执行关节指令
    }
    else
        chassis_status = ROBOT_STOP;

    // 还在复位中,关节改为位置环,执行复位
    for (uint8_t i = 0; i < JOINT_CNT; i++)
    {
        HTMotorOuterLoop(joint[i], ANGLE_LOOP);
        HTMotorSetRef(joint[i], 0);
    }
}

static void WokingStateSet()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_RESET) // 复位模式
    {
        ResetChassis();
        return;
    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_STOP) // 未收到遥控器和云台指令底盘进入急停
    {
        for (uint8_t i = 0; i < JOINT_CNT; i++)
            HTMotorStop(joint[i]);
        for (uint8_t i = 0; i < DRIVEN_CNT; i++)
            LKMotorStop(driven[i]);
        return; // 关闭所有电机,发送的指令为零
    }
    // 运动模式
        EnableAllMotor();
    if (chassis_cmd_recv.chassis_mode == CHASSIS_FREE_DEBUG)
    {
        
        // 分模式设置目标速度/腿长/距离
        if (switch_is_mid(rc_data->rc.switch_right) && RemoteControlIsOnline())
        {
            if (switch_is_down(rc_data->rc.switch_left) && RemoteControlIsOnline())
            {
                chassis.target_v = 1 * (float)rc_data[TEMP].rc.rocker_l1/660;
                chassis.target_yaw_w = 0.8 * (float)rc_data[TEMP].rc.rocker_r_/660;
            } else if (switch_is_mid(rc_data->rc.switch_left) && RemoteControlIsOnline())
            {
                 l_side.target_len =  r_side.target_len = 0.13 * (float)rc_data[TEMP].rc.rocker_r1/660 + 0.24;
            } else if (switch_is_up(rc_data->rc.switch_left) && RemoteControlIsOnline())
            {
                l_side.target_len =  r_side.target_len += 0.13 * (float)rc_data[TEMP].rc.rocker_r1/660 + 0.24;
                chassis.target_v = 1 * (float)rc_data[TEMP].rc.rocker_l1/660;
                chassis.target_yaw_w = 0.8 * (float)rc_data[TEMP].rc.rocker_l_/660;
            }
            
        }
     }else  if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE)
    {
          chassis.target_yaw_w = 1.6 * (float)rc_data[TEMP].rc.rocker_l_/660;
     }
      
          // 模型距离参考输入
        chassis.target_dist += chassis.target_v * del_t;
        chassis.target_yaw += chassis.target_yaw_w* del_t; 
    
        VAL_LIMIT(l_side.target_len, 0.13, 0.4); // 腿长限幅
        VAL_LIMIT(r_side.target_len, 0.13, 0.4);
      
    
      
}
/**
 * @brief 将电机和imu的数据组装为LinkNPodParam结构体和chassisParam结构体
 *
 * @note HT04电机上电的编码器位置为零(校准过),请看Link2Pod()的note,以及HT04.c中的电机解码部分
 * @note 海泰04电机顺时针旋转为正; LK9025电机逆时针旋转为正,此处皆需要转换为模型中给定的正方向
 *
 */
static void ParamAssemble()
{ // 机体参数,视为平面刚体
    chassis.pitch = (-imu_data->Pitch + BALANCE_GRAVITY_BIAS) * DEGREE_2_RAD;
    chassis.pitch_w = -imu_data->Gyro[0];
    chassis.yaw = imu_data->YawTotalAngle * DEGREE_2_RAD;
    chassis.wz = imu_data->Gyro[2];
    chassis.roll = imu_data->Roll * DEGREE_2_RAD + ROLL_GRAVITY_BIAS;
    chassis.roll_w = imu_data->Gyro[1];
    // HT04电机的角度是顺时针为正,LK9025电机的角度是逆时针为正
    l_side.phi1 = PI + LIMIT_LINK_RAD - lb->measure.total_angle;
    l_side.phi4 = -lf->measure.total_angle - LIMIT_LINK_RAD;
    l_side.phi1_w = -lb->measure.speed_rads;
    l_side.phi4_w = -lf->measure.speed_rads;
    l_side.w_ecd = l_driven->measure.speed_rads;
    r_side.phi1 = PI + LIMIT_LINK_RAD + rb->measure.total_angle;
    r_side.phi4 = rf->measure.total_angle - LIMIT_LINK_RAD;
    r_side.phi1_w = rb->measure.speed_rads;
    r_side.phi4_w = rf->measure.speed_rads;
    r_side.w_ecd = -r_driven->measure.speed_rads;
}

/**
 * @brief 根据状态反馈计算当前腿长,查表获得LQR的反馈增益,并列式计算LQR的输出
 * @note 得到的腿部力矩输出还要经过综合运动控制系统补偿后映射为两个关节电机输出
 *
 */
static void CalcLQR(float delta_t)
{
    //通过多项式拟合计算K矩阵
    static float lqr_k[40][10] = {60.071162, -57.303242, -8.802552, -0.219882, -1.390464, -0.951558, 32.409644, -23.635877, -9.253521, 12.436266, -10.318639, -7.529540, 135.956804, -124.044032, 36.093582, 13.977819, -12.325081, 3.766791, 32.632159, -39.012888, 14.483707, 7.204778, -5.973109, 1.551763, 78.723013, -73.096567, 21.701734, 63.798027, -55.564993, 15.318437, -195.813486, 140.134182, 60.699132, -18.357761, 13.165559, 2.581731};
    float k[4][10]={0};
    float T[4] = {0}; //
    float l = l_side.leg_len, r = r_side.leg_len;
    float l_2 = l * l,r_2 = r * r;
    float l_3 = l_2 * l, r_3 = r_2 * r;
   

    for (uint8_t i = 0; i < 4; i++)
    {
        for (uint8_t j = 0; j < 10; j++)
        {
            uint8_t n = i * 10 + j;
            
            k[i][j] = lqr_k[n][0]+lqr_k[n][1]*l+lqr_k[n][2]*r+lqr_k[n][3]*l_2+lqr_k[n][4]*l*r+lqr_k[n][5]*r_2+lqr_k[n][6]*l_3+lqr_k[n][7]*l_2*r+lqr_k[n][8]*l*r_2+lqr_k[n][9]*r_3;
        }
          
    }
    for (uint8_t c = 0; c < 4; c++)
    {
        T[c] = k[c][0]*(chassis.target_dist - chassis.dist)+k[c][1]*(chassis.target_v - chassis.vel)+k[c][2]*(chassis.target_yaw - chassis.yaw )+k[c][3]*(chassis.target_yaw_w-chassis.wz)+k[c][4]*(-l_side.theta)+k[c][5]*(-l_side.theta_w)+k[c][6]*(-r_side.theta)+k[c][7]*(-r_side.theta_w)+k[c][8]*( -chassis.pitch)+k[c][9]*( -chassis.pitch_w);
    }

    //通过预测电机角速度与当前轮毂电机角速度进行比较，如果出现偏差则认为是打滑踩小弹丸等外界干扰造成的，这时候对轮速进行补偿
    static float AB[12][10]={0};//有两项为0，舍去这两行
    float A[2][2]={0};
    float B[2][4]={0};
    static float K_adapt=0.2;
    //与其for循环，不如直接自动补全来的省事
    A[0][0] = AB[0][0]+AB[0][1]*l+AB[0][2]*r+AB[0][3]*l_2+AB[0][4]*l*r+AB[0][5]*r_2+AB[0][6]*l_3+AB[0][7]*l_2*r+AB[0][8]*l*r_2+AB[0][9]*r_3;
    A[0][1] = AB[1][0]+AB[1][1]*l+AB[1][2]*r+AB[1][3]*l_2+AB[1][4]*l*r+AB[1][5]*r_2
                +AB[1][6]*l_3+AB[1][7]*l_2*r+AB[1][8]*l*r_2+AB[1][9]*r_3;         
    B[0][0] = AB[2][0]+AB[2][1]*l+AB[2][2]*r+AB[2][3]*l_2+AB[2][4]*l*r+AB[2][5]*r_2
                +AB[2][6]*l_3+AB[2][7]*l_2*r+AB[2][8]*l*r_2+AB[2][9]*r_3;
    B[0][1] = AB[3][0]+AB[3][1]*l+AB[3][2]*r+AB[3][3]*l_2+AB[3][4]*l*r+AB[3][5]*r_2
                +AB[3][6]*l_3+AB[3][7]*l_2*r+AB[3][8]*l*r_2+AB[3][9]*r_3;
    B[0][2] = AB[4][0]+AB[4][1]*l+AB[4][2]*r+AB[4][3]*l_2+AB[4][4]*l*r+AB[4][5]*r_2
                +AB[4][6]*l_3+AB[4][7]*l_2*r+AB[4][8]*l*r_2+AB[4][9]*r_3;
    B[0][3] = AB[5][0]+AB[5][1]*l+AB[5][2]*r+AB[5][3]*l_2+AB[5][4]*l*r+AB[5][5]*r_2
                +AB[5][6]*l_3+AB[5][7]*l_2*r+AB[5][8]*l*r_2+AB[5][9]*r_3;
    A[1][0] = AB[6][0]+AB[6][1]*l+AB[6][2]*r+AB[6][3]*l_2+AB[6][4]*l*r+AB[6][5]*r_2
                +AB[6][6]*l_3+AB[6][7]*l_2*r+AB[6][8]*l*r_2+AB[6][9]*r_3;
    A[1][1] = AB[7][0]+AB[7][1]*l+AB[7][2]*r+AB[7][3]*l_2+AB[7][4]*l*r+AB[7][5]*r_2
                +AB[7][6]*l_3+AB[7][7]*l_2*r+AB[7][8]*l*r_2+AB[7][9]*r_3;
                B[1][0] = AB[8][0]+AB[8][1]*l+AB[8][2]*r+AB[8][3]*l_2+AB[8][4]*l*r+AB[8][5]*r_2
                +AB[8][6]*l_3+AB[8][7]*l_2*r+AB[8][8]*l*r_2+AB[8][9]*r_3;
    B[1][1] = AB[9][0]+AB[9][1]*l+AB[9][2]*r
                +AB[9][3]*l_2+AB[9][4]*l*r+AB[9][5]*r_2
                +AB[9][6]*l_3+AB[9][7]*l_2*r+AB[9][8]*l*r_2+AB[9][9]*r_3;
    B[1][2] = AB[10][0]+AB[10][1]*l+AB[10][2]*r
                +AB[10][3]*l_2+AB[10][4]*l*r+AB[10][5]*r_2
                +AB[10][6]*l_3+AB[10][7]*l_2*r+AB[10][8]*l*r_2+AB[10][9]*r_3;
    B[1][3] = AB[11][0]+AB[11][1]*l+AB[11][2]*r
                +AB[11][3]*l_2+AB[11][4]*l*r+AB[11][5]*r_2
                +AB[11][6]*l_3+AB[11][7]*l_2*r+AB[11][8]*l*r_2+AB[11][9]*r_3;
    
    //对机体速度与机体yaw角速度进行预测 delta_t
    chassis.vel_hat=(1+A[0][0]*delta_t)*l_side.theta+(1+A[0][1]*delta_t)*r_side.theta+B[0][0]*delta_t*T[0]+B[0][1]*delta_t*T[1]+B[0][2]*delta_t*T[2]+B[0][3]*delta_t*T[3];
    chassis.wz_hat=(1+A[1][0]*delta_t)*l_side.theta+(1+A[1][1]*delta_t)*r_side.theta+B[1][0]*delta_t*T[0]+B[1][1]*delta_t*T[1]+B[1][2]*delta_t*T[2]+B[1][3]*delta_t*T[3];
    //计算电机角速度
    l_side.w_ecd_hat=( chassis.vel_hat-Rl* chassis.wz_hat)/WHEEL_RADIUS;
    r_side.w_ecd_hat=( chassis.vel_hat+Rl* chassis.wz_hat)/WHEEL_RADIUS;
    //补偿
    l_side.T_wheel_hat=K_adapt(l_side.w_ecd_hat-l_side.w_ecd);
    r_side.T_wheel_hat=K_adapt(r_side.w_ecd_hat-r_side.w_ecd);
    //离地检测
    // if (l_side.FN< 10)
    // {
    //     l_side.T_wheel = 0;
    //     l_side.T_hip =k[2][4]*(-l_side.theta)+k[2][5]*(-l_side.theta_w) ;
    // }else
    // {
        // l_side.T_wheel = T[0]+l_side.T_wheel_hat;
        l_side.T_wheel = T[0];
        l_side.T_hip = T[2]; 
   // }
    // if (r_side.FN< 10)
    // {
    //     r_side.T_wheel = 0;
    //     r_side.T_hip =k[3][6]*(-r_side.theta)+k[3][7]*(-r_side.theta_w) ;
    // }
    // else
    // {
    // r_side.T_wheel = T[1]+r_side.T_wheel_hat;
    r_side.T_wheel = T[1];
    r_side.T_hip = T[3];
    //}
}


static void LegControl() /* 腿长控制和Roll补偿 */
{
    PIDCalculate(&roll_compensate_pid, chassis.roll, 0);
    l_side.target_len -= roll_compensate_pid.Output;
    r_side.target_len += roll_compensate_pid.Output;
    //腿部质心位置系数认为是1/2
    float gravity_comp =0.5 * (Mb+Ml)*9.8 ;
    l_side.inertial = 0.5 * (Mb+Ml)*l_side.leg_len*chassis.wz*chassis.vel/(2*Rl);
    r_side.inertial = 0.5 * (Mb+Ml)*r_side.leg_len*chassis.wz*chassis.vel/(2*Rl);
    //roll力矩前馈
    PIDCalculate(&roll_comp_pid, chassis.roll, 0);
    l_side.roll_comp=roll_comp_pid.Output;
    r_side.roll_comp=roll_comp_pid.Output;
    //z轴加速度补偿暂不考虑
    l_side.F_leg = PIDCalculate(&leglen_pid_l, l_side.height, l_side.target_len) + gravity_comp - l_side.inertial+l_side.roll_comp;
    r_side.F_leg = PIDCalculate(&leglen_pid_r, r_side.height, r_side.target_len) + gravity_comp + r_side.inertial-r_side.roll_comp; 
    //计算支持力
    l_side.FN=l_side.F_leg*mcos(l_side.phi5) +l_side.T_hip*msin(l_side.phi5)/l_side.leg_len+Mw*(9.8+imu_data->Accel[2]-l_side.legdd);
    r_side.FN=r_side.F_leg*mcos(r_side.phi5) +r_side.T_hip*msin(r_side.phi5)/r_side.leg_len+Mw*(9.8+imu_data->Accel[2]-r_side.legdd);
}

static void WattLimitSet() /* 设定运动模态的输出 */
{
    HTMotorSetRef(lf, 0.285f * l_side.T_front); // 根据扭矩常数计算得到的系数
    HTMotorSetRef(lb, 0.285f * l_side.T_back);
    HTMotorSetRef(rf, 0.285f * r_side.T_front);
    HTMotorSetRef(rb, 0.285f * r_side.T_back);
    LKMotorSetRef(l_driven, 274.348 * l_side.T_wheel);
    LKMotorSetRef(r_driven, 274.348 * r_side.T_wheel);
}

// 裁判系统,双板通信,电容功率控制等
static void CommNPower()
{
    static uint8_t excute_flag;
    ++excute_flag; // 用于一些低速率的任务

    if (excute_flag % 4 == 0) // 250Hz反馈频率
    {
        chassis_feed.yaw_w = chassis.wz;
        chassis_feed.bullet_speed = referee_data->ShootData.bullet_speed;
        chassis_feed.state = chassis_status; // 反馈底盘是否准备好站立
        // chassis_feed.enemy_color = referee_data
        CANCommSend(ci, (uint8_t *)&chassis_feed);
    }

    static uint16_t cap_data[4] = {0}; // 电容数据
    if (excute_flag % 40 == 0)
    { // 25Hz电容控制频率
        cap_data[0] = referee_data->PowerHeatData.chassis_power_buffer;
        cap_data[1] = referee_data->GameRobotState.chassis_power_limit;
        // SuperCapSend(cap, (uint8_t *)cap_data);
    }

    /* 更新ui数据 */
    my_ui.cap_msg = cap->cap_msg;
    my_ui.direction = chassis_cmd_recv.direction;
    my_ui.friction_mode = chassis_cmd_recv.friction_mode;
    my_ui.loader_mode = chassis_cmd_recv.loader_mode;
    my_ui.lid_mode = chassis_cmd_recv.lid_mode;
    my_ui.target_state = chassis_cmd_recv.target_state;
    my_ui.chassis_mode = chassis_cmd_recv.chassis_mode;
    my_ui.refresh_flag = chassis_cmd_recv.ui_refresh_flag;
    memcpy(my_ui.coord, l_side.coord, sizeof(l_side.coord));
}

void BalanceTask()
{
    del_t = DWT_GetDeltaT(&balance_dwt_cnt);

    // 切换遥控器控制or云台板控制
    ControlSwitch();
    // 设置目标参数和工作模式
    WokingStateSet();
    // 双板通信,电容功率控制,裁判系统
    CommNPower();

    // 参数组装
    ParamAssemble();
    // 将五连杆映射成单杆
    Link2Leg(&l_side, &chassis);
    Link2Leg(&r_side, &chassis);
    // 通过卡尔曼滤波估计机体速度
    SpeedEstimation(&l_side, &r_side, &chassis, imu_data, del_t);

    // 根据单杆计算处的角度和杆长,计算反馈增益
    CalcLQR(del_t);
  
    // 腿长控制,保持机体水平
    LegControl();
    // VMC映射成关节输出
    VMCProject(&l_side);
    VMCProject(&r_side);

    // stop表示复位尚未完成,reset表明还未切换到其他模式,故都不执行运动模态的代码
    if (chassis_status == ROBOT_STOP ||
        chassis_cmd_recv.chassis_mode == CHASSIS_RESET ||
        chassis_cmd_recv.chassis_mode == CHASSIS_STOP)
        return; // 复位模态或急停,直接退出

    // 运动模态,电机输出映射和限幅
    WattLimitSet();
}