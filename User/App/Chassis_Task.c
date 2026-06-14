#include "Chassis_Task.h"


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


