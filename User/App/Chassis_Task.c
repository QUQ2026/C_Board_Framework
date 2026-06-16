#include "Chassis_Task.h"

static float NormalizeAngle(float angle)//角度归一化
{
    angle = fmodf(angle, 360.0f);
    if (angle >  180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}


static void ApplyGimbalTransform(CONTAL_Typedef *CONTAL,
                                 DBUS_Typedef   *DBUS,
                                 float           gimbal_deg)//云台坐标系向底盘转换
{
    /* 前馈：预测下一周期底盘转到哪 */
    float angle_rad = (gimbal_deg + CONTAL->BOTTOM.VW * CHASSIS_LOOP_TIME_S * (180.0f / 3.14159265f))
                      * (3.14159265f / 180.0f);

    float vx_rc = DBUS->Remote.CH1 * (VX_MAX / REMOTE_SCALE);  // 遥控 → m/s
    float vy_rc = DBUS->Remote.CH0 * (VY_MAX / REMOTE_SCALE);

    /* 旋转矩阵：将遥控输入旋转到底盘系 */
    CONTAL->BOTTOM.VX =  vx_rc * cosf(angle_rad) - vy_rc * sinf(angle_rad);
    CONTAL->BOTTOM.VY =  vx_rc * sinf(angle_rad) + vy_rc * cosf(angle_rad);
}
uint8_t Motor_PID_Chassis_Init(MOTOR_Typdef *MOTOR)
{
    float PID_S_1[3] = {   3.0f,   0.0f,   0.0f   };
    float PID_S_2[3] = {   3.0f,   0.0f,   0.0f   };
    float PID_S_3[3] = {   3.0f,   0.0f,   0.0f   };
    float PID_S_4[3] = {   3.0f,   0.0f,   0.0f   };
    PID_Init(&MOTOR->DJI_3508_Chassis_1.PID_S, 16384.0f, 1000.0f,
            PID_S_1, 0, 0,
            0, 0, 0,
            Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
            //梯形积分,变速积分
            );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis_2.PID_S, 16384.0f, 1000.0f,
            PID_S_2, 0, 0,
            0, 0, 0,
            Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
            //梯形积分,变速积分
            );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis_3.PID_S, 16384.0f, 1000.0f,
             PID_S_3, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    PID_Init(&MOTOR->DJI_3508_Chassis_4.PID_S, 16384.0f, 1000.0f,
             PID_S_4, 0, 0,
             0, 0, 0,
             Integral_Limit|ErrorHandle//积分限幅,输出滤波,堵转监测
             //梯形积分,变速积分
             );//微分先行,微分滤波器
    return RUI_DF_READY;

}
uint8_t Chassis_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typdef *MOTOR)
{
    //检查离线
    if (Root->MOTOR_Chassis_1     == RUI_DF_OFFLINE ||
        Root->MOTOR_Chassis_2     == RUI_DF_OFFLINE ||
        Root->MOTOR_Chassis_3     == RUI_DF_OFFLINE ||
        Root->MOTOR_Chassis_4     == RUI_DF_OFFLINE)
        return RUI_DF_ERROR;

    //电机清空
    HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Chassis_1, 1);
    HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Chassis_2, 1);
    HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Chassis_3, 1);
    HEAD_MOTOR_CLEAR(&MOTOR->DJI_3508_Chassis_4, 1);

    return RUI_DF_READY;
}

uint8_t chassis_task(CONTAL_Typedef *CONTAL,
                   RUI_ROOT_STATUS_Typedef *Root,
                   User_Data_T *User_data,
                   model_t *model,
                   CAP_RXDATA *CAP_GET,
                   MOTOR_Typdef *MOTOR) {
    static uint8_t PID_INIT = RUI_DF_ERROR;
    static uint8_t AIM_INIT = RUI_DF_ERROR;

    //电机PID赋值
    if (PID_INIT != RUI_DF_READY)
    {
        PID_INIT = Motor_PID_Chassis_Init(MOTOR);
        return RUI_DF_ERROR;
    }
    MOTOR->DJI_3508_Chassis_3.DATA.Aim = CONTAL->BOTTOM.wheel1*4.0f;
    MOTOR->DJI_3508_Chassis_4.DATA.Aim = CONTAL->BOTTOM.wheel2*4.0f;
    MOTOR->DJI_3508_Chassis_1.DATA.Aim = CONTAL->BOTTOM.wheel3*4.0f;
    MOTOR->DJI_3508_Chassis_2.DATA.Aim = CONTAL->BOTTOM.wheel4*4.0f;
    //		}
    //    /*遥控离线保护
    if(Root->RM_DBUS==0)
    {
        CONTAL->BOTTOM.wheel1 = 0;
        CONTAL->BOTTOM.wheel2 =0;
        CONTAL->BOTTOM.wheel3 = 0;
        CONTAL->BOTTOM.wheel4 =0;
    }
    PID_Calculate(&MOTOR->DJI_3508_Chassis_1.PID_S,
                     (float)MOTOR->DJI_3508_Chassis_1.DATA.Speed_now,
                     MOTOR->DJI_3508_Chassis_1.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_2.PID_S,
                 (float)MOTOR->DJI_3508_Chassis_2.DATA.Speed_now,
                 MOTOR->DJI_3508_Chassis_2.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_3.PID_S,
                 (float)MOTOR->DJI_3508_Chassis_3.DATA.Speed_now,
                 MOTOR->DJI_3508_Chassis_3.DATA.Aim);
    PID_Calculate(&MOTOR->DJI_3508_Chassis_3.PID_S,
                 (float)MOTOR->DJI_3508_Chassis_3.DATA.Speed_now,
                 MOTOR->DJI_3508_Chassis_3.DATA.Aim);
   //功率控制
    chassis_power_control(CONTAL,
                             User_data,
                             model,
                             CAP_GET,
                             MOTOR);

    /*总输出计算*/
    float Out_put[4];
    Out_put[0] = /*MOTOR->DJI_3508_Chassis_1.PID_F.Output*/0 +
               MOTOR->DJI_3508_Chassis_1.PID_S.Output;

    Out_put[1] = /*MOTOR->DJI_3508_Chassis_2.PID_F.Output*/0 +
               MOTOR->DJI_3508_Chassis_2.PID_S.Output;

    Out_put[2] = /*MOTOR->DJI_3508_Chassis_3.PID_F.Output*/0 +
               MOTOR->DJI_3508_Chassis_3.PID_S.Output;

    Out_put[3] = /*MOTOR->DJI_3508_Chassis_4.PID_F.Output*/0 +
               MOTOR->DJI_3508_Chassis_4.PID_S.Output;

    /*CAN发送*/
    DJI_Current_Ctrl(&hcan1,
                     0x200,
                     (int16_t)Out_put[0],
                     (int16_t)Out_put[1],
                     (int16_t)Out_put[2],
                     (int16_t)Out_put[3]
                                            );
    return RUI_DF_READY;
}


//底盘mode
void Chassis_Normal(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, MOTOR_Typdef *MOTOR)//普通模式
{
    /* 旋转速度：拨轮映射 */
    CONTAL->BOTTOM.VW = DBUS->Remote.CH3 * (VW_MAX / REMOTE_SCALE);

    /* 读取 Yaw 电机编码器，计算云台相对底盘偏角（度） */
    float raw_angle = fmodf(MOTOR->DJI_6020_Yaw.DATA.Angle_now * 360.0f / 8192.0f, 360.0f);
    float gimbal_deg = NormalizeAngle(raw_angle);

    /* 坐标变换 + 全向轮逆解 */
    ApplyGimbalTransform(CONTAL, DBUS, gimbal_deg);
    OmniResolve(CONTAL);
}


void Chassis_Gyroscope(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, IMU_Data_t *IMU)//小陀螺
{
    /* 固定旋转速度 */
    CONTAL->BOTTOM.VW = GYROSCOPE_W;

    /* 使用陀螺仪 yaw（不修改原始值，局部变量归一化）*/
    float gimbal_deg = NormalizeAngle(IMU->yaw);

    /* 坐标变换 + 全向轮逆解 */
    ApplyGimbalTransform(CONTAL, DBUS, gimbal_deg);
    OmniResolve(CONTAL);
}


void Chassis_Follow_Gimbal(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, IMU_Data_t *IMU)//底盘跟随
{
    /* 读取云台偏角（陀螺仪 yaw，归一化到 [-180, 180]）*/
    float gimbal_deg = NormalizeAngle(IMU->yaw);

    /* 期望云台对准的角度：通常为 0（正前方）
     * 若需要遥控拨轮微调方向，可用 CH2 映射；默认跟随正前方，target=0 */
    float target_deg = 0.0f;
    /* 如需拨轮微调，取消下行注释：
     * target_deg = DBUS->Remote.CH2 * (180.0f / REMOTE_SCALE); */

    /* 角度误差，映射到 [-180, 180] 防止反绕 */
    float angle_err = NormalizeAngle(target_deg - gimbal_deg);

    /* PD 控制计算期望底盘转速 */
    float vw = FOLLOW_KP * angle_err
             + FOLLOW_KD * (angle_err - s_last_angle_err);
    s_last_angle_err = angle_err;

    /* 限幅 */
    CONTAL->BOTTOM.VW = Clamp(vw, VW_MAX);

    /* 坐标变换 + 全向轮逆解 */
    ApplyGimbalTransform(CONTAL, DBUS, gimbal_deg);
    OmniResolve(CONTAL);
}
