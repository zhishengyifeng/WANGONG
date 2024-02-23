/* speed_estimation 速度估计,单变量KF*/
// 融合加速度计的数据和机体速度
// static float f, k, prior, measure, cov;
// f = (cp->acc + cp->acc_last) / 2;    // 速度梯形积分
// prior = cp->vel + cp->acc * delta_t; // x' = Fx,先验
// cp->vel_predict = prior;
// measure = cp->vel_m;
// cov = cp->vel_cov + VEL_PROCESS_NOISE * powf(cp->acc, 2); // P' = P + Q ,先验协方差
// cp->vel_cov = cov;
// k = cov / (cov + VEL_MEASURE_NOISE);     // K = P'/(P'+R),卡尔曼增益
// cp->vel = prior + k * (measure - prior); // x^ = x'+K(z-x'),后验估计
// cp->vel_cov *= (1 - k);                  // P^ = (1-K)P',后验协方差
// VAL_LIMIT(cp->vel_cov, 0.0001, 100);     // 协方差限幅
// cp->dist = cp->dist + cp->vel * delta_t;

/* LQR单项输出debug */
static void CalcLQR(LinkNPodParam *p)
{
    static float coef[12][3] = {93.065915, -164.960003, -0.645929, -12.016872, -22.733167, 0.443621, 84.994289, -60.013517, -9.581400, 57.527405, -51.082121, -6.867215, 221.334661, -193.883612, 58.958659, 10.155117, -10.008061, 3.846870, -87.143272, 43.031820, 18.121294, -16.054507, 13.466020, 1.870082, 116.154249, -109.805729, 36.354686, 71.098977, -68.790819, 25.358250, -393.750176, 279.649410, 67.564537, -24.113663, 18.058740, 0.755108};
    float T[2] = {0}; // 0 T_wheel, 1 T_hip;
    float l = p->leg_len;
    float lsqr = l * l;
    for (uint8_t i = 0; i < 2; ++i)
    {
        uint8_t pos = i * 6;
        T[i] += (coef[pos + 0][0] * lsqr + coef[pos + 0][1] * l + coef[pos + 0][2]) * -p->theta;
        T[i] += (coef[pos + 1][0] * lsqr + coef[pos + 1][1] * l + coef[pos + 1][2]) * -p->theta_w;
        T[i] += (coef[pos + 2][0] * lsqr + coef[pos + 2][1] * l + coef[pos + 2][2]) * (target_dist - chassis.dist);
        T[i] += (coef[pos + 3][0] * lsqr + coef[pos + 3][1] * l + coef[pos + 3][2]) * (chassis_cmd_recv.vx - chassis.vel);
        T[i] += (coef[pos + 4][0] * lsqr + coef[pos + 4][1] * l + coef[pos + 4][2]) * -chassis.pitch;
        T[i] += (coef[pos + 5][0] * lsqr + coef[pos + 5][1] * l + coef[pos + 5][2]) * -chassis.pitch_w;
    }
    p->T_wheel = T[0];
    p->T_hip = T[1];

    // p->wheel_out[0] = (coef[0][0] * lsqr + coef[0][1] * l + coef[0][2]) * -p->theta;
    // p->wheel_out[1] = (coef[1][0] * lsqr + coef[1][1] * l + coef[1][2]) * -p->theta_w ;
    // p->wheel_out[2] = (coef[2][0] * lsqr + coef[2][1] * l + coef[2][2]) * (target_dist - chassis.dist);
    // p->wheel_out[3] = (coef[3][0] * lsqr + coef[3][1] * l + coef[3][2]) * (chassis_cmd_recv.vx - chassis.vel);
    // p->wheel_out[4] = (coef[4][0] * lsqr + coef[4][1] * l + coef[4][2]) * -chassis.pitch;
    // p->wheel_out[5] = (coef[5][0] * lsqr + coef[5][1] * l + coef[5][2]) * -chassis.pitch_w;
    // p->hip_out[0] = (coef[6][0] * lsqr + coef[6][1] * l + coef[6][2]) * -p->theta;
    // p->hip_out[1] = (coef[7][0] * lsqr + coef[7][1] * l + coef[7][2]) * -p->theta_w ;
    // p->hip_out[2] = (coef[8][0] * lsqr + coef[8][1] * l + coef[8][2]) * (target_dist - chassis.dist);
    // p->hip_out[3] = (coef[9][0] * lsqr + coef[9][1] * l + coef[9][2]) * (chassis_cmd_recv.vx - chassis.vel);
    // p->hip_out[4] = (coef[10][0] * lsqr + coef[10][1] * l + coef[10][2]) * -chassis.pitch;
    // p->hip_out[5] = (coef[11][0] * lsqr + coef[11][1] * l + coef[11][2]) * -chassis.pitch_w;
    // for (uint8_t i = 0; i < 6; i++)
    // {
    //     T[0] += p->wheel_out[i];
    //     T[1] += p->hip_out[i];
    // }
    // p->T_wheel = T[0];
    // p->T_hip = T[1];

    // static float gain_list[12] = {-20, -4, -10, -9, 15, 4,
    //                               10, 0.2, 2.2, 2.1, 60, 2};
    // float T[2] = {0}; // 0 T_wheel, 1 T_hip;
    // for (uint8_t i = 0; i < 2; ++i)
    // {
    //     uint8_t pos = i * 6;
    //     // T[i] += gain_list[pos + 0] * -p->theta;
    //     // T[i] += gain_list[pos + 1] * -p->theta_w;
    //     // T[i] += gain_list[pos + 2] * (target_dist - chassis.dist);
    //     // T[i] += gain_list[pos + 3] * chassis_cmd_recv.vx - chassis.vel;
    //     T[i] += gain_list[pos + 4] * -chassis.pitch;
    //     T[i] += gain_list[pos + 5] * -chassis.pitch_w;
    // }
    // p->T_wheel = T[0];
    // p->T_hip = T[1];
}