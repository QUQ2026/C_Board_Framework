#include "Robot.h"
float monitor_X;
float monitor_Y;
float monitor_W;
float yaw_TD;
float pitch_TD;
static float counttt=0.0f;
float VISION_connect;
void RobotTask(uint8_t mode,
               DBUS_Typedef *DBUS,
               CONTAL_Typedef *CONTAL,
               User_Data_T *User_data,
               CAPDATE_TYPDEF *CAP_DATA,
               TYPEDEF_VISION *Vision/* 普通视觉*/
							/*	VisionRxDataUnion *Vision 加预测视觉*/,
               RUI_ROOT_STATUS_Typedef *Root,
               MOTOR_Typdef *MOTOR,
               IMU_Data_t *IMU_Data,
							 TD_t *TDDD,
							VT13_Typedef* VT13_DBUS)
{
    switch (mode) {
        case 1://底盘
         if (Root->RM_DBUS == RUI_DF_OFFLINE)
            {
                CONTAL->BOTTOM.VX    = 0.0f;
                CONTAL->BOTTOM.VY    = 0.0f;
                CONTAL->BOTTOM.VW    = 0.0f;
                CONTAL->BOTTOM.wheel1 = 0.0f;
                CONTAL->BOTTOM.wheel2 = 0.0f;
                CONTAL->BOTTOM.wheel3 = 0.0f;
                CONTAL->BOTTOM.wheel4 = 0.0f;
                break;
            }

            /* -------- 底盘模式选择（S2 拨轮）-------- */
            switch ((Chassis_Mode_e)DBUS->Remote.S2)
            {
                case CHASSIS_MODE_FOLLOW:
                    /* 底盘跟随云台：PID 驱动底盘对准云台方向，平移仍以云台视角 */
                    Chassis_Follow_Gimbal(CONTAL, DBUS, IMU_Data);
                    break;

                case CHASSIS_MODE_NORMAL:
                    /* 普通模式：拨轮控制旋转，平移以云台视角（编码器）*/
                    Chassis_Normal(CONTAL, DBUS, MOTOR);
                    break;

                case CHASSIS_MODE_GYRO:
                    /* 小陀螺：底盘固定转速自旋，操作手仍可平移 */
                    Chassis_Gyroscope(CONTAL, DBUS, IMU_Data);
                    break;

                default:
                    /* 未知模式：停止 */
                    CONTAL->BOTTOM.VX = 0.0f;
                    CONTAL->BOTTOM.VY = 0.0f;
                    CONTAL->BOTTOM.VW = 0.0f;
                    break;
            }

            /* -------- 调试监视 -------- */
            monitor_X = CONTAL->BOTTOM.VX;
            monitor_Y = CONTAL->BOTTOM.VY;
            monitor_W = CONTAL->BOTTOM.VW;
        } break;
        case 2://云台
        {
            switch (DBUS->Remote.S2)
            {
                case 1:
                    /* 底盘跟随模式：云台目标锁定，底盘 PID 跟随 */
                    Gimbal_Set_Target_Follow(CONTAL, DBUS, IMU_Data);
                    break;

                case 2:
                    /* 普通模式：摇杆累加控制云台目标角度 */
                    Gimbal_Set_Target_RC(CONTAL, DBUS, IMU_Data);
                    break;

                case 3:
                    /* 小陀螺模式：摇杆控制云台，陀螺仪稳定云台绝对方向不变 */
                    Gimbal_Set_Target_RC(CONTAL, DBUS, IMU_Data);
                    break;

                default:
                    break;
            }

            /* -------- 串级PID计算 + CAN发送 -------- */
            gimbal_task(CONTAL, Root, MOTOR, IMU_Data);

            /* -------- 调试监视 -------- */
            yaw_TD   = CONTAL->HEAD.Yaw;
            pitch_TD = CONTAL->HEAD.Pitch;

        } break;

        case 3://电容
        {

        } break;

        case 4://发射
        {
					
        } break;
    }
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//24-2-25 下午1:24
 *	@ReadMe:			//获取摩擦轮的目标值
 ************************************************************万能分隔符**************************************************************/
float Shoot_Speed_P(float Kp, float measure, float ref, float OUT_Lim)
{
    float error = ref - measure;
    /*比例输出*/
    float ALL_Out = error * Kp;
    /*总输出限幅*/
    ALL_Out = MATH_Limit_float(OUT_Lim, -OUT_Lim, ALL_Out);

    return ALL_Out;
}

float RUI_F_GET_FIRE_WIPE_SPEED(CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS, User_Data_T *User_data, RUI_ROOT_STATUS_Typedef *Root)
{
    static uint8_t LOCK = 0, MOD = 0, KEYBOARD_LOCK = 0, JUDGE_LOCK = 0;
    static float AIM = 0.0f, TEMP = 0.0f, SPEED_NOW = 0.0f, SPEED_LAST = 0.0f;
    MOD = DBUS->Remote.S2-1 | DBUS->Mouse.L_State | LOCK;
    //停止
    if (MOD == 0 || DBUS->Remote.S2-1 == 1 || Root->RM_DBUS == RUI_DF_OFFLINE)
    {
        AIM = 0.0f;//0.0
        LOCK = 0;
    } else
    {
        LOCK = 1;
        SPEED_LAST = SPEED_NOW;
        SPEED_NOW = User_data->shoot_data.initial_speed;

        /*弹速PID,仅限于修正摩擦轮发热所带来的弹速变化,优化为打一发弹算一次*/
        if (CONTAL->SHOOT_Bask.Shoot_Number - CONTAL->SHOOT_Bask.Shoot_Number_Last > 0)
        {
            if(MATH_ABS_float(SPEED_NOW - SPEED_LAST) > 0.1f)
                TEMP = Shoot_Speed_P(5.0f, SPEED_NOW, 29.5f, 50);
            CONTAL->SHOOT_Bask.Shoot_Number_Last = CONTAL->SHOOT_Bask.Shoot_Number;
        }

        AIM = (float) CONTAL->SHOOT.Shoot_Speed + TEMP;

    }
    return AIM;
}


/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //
 *	@parameter:		    //
 *	@time:				//24-5-8 上午9:44
 *	@ReadMe:			//获取最大目标值
 ************************************************************万能分隔符**************************************************************/
float RUI_F_CHASSIS_GET_MAX_TARGET(float MAX_POWER)
{
    // 200w 0.04f
    // 100w 0.065f
    //  90w 0.065f
    //  80w 0.07f
    //  75w 0.08f
    //  70w 0.09f
    //  65w 0.1f
    //  60w 0.08f
    //  55w 0.06f
    //  50w 0.04f
    //  45w 0.02f
    if (MAX_POWER == 45)
    {
        return 0.03f * MAX_POWER;
    } else if (MAX_POWER == 50 || MAX_POWER == 200)
    {
        return 0.04f * MAX_POWER;
    } else if (MAX_POWER == 55)
    {
        return 0.06f * MAX_POWER;
    } else if (MAX_POWER == 60 || MAX_POWER == 75)
    {
        return 0.08f * MAX_POWER;
    } else if (MAX_POWER == 65)
    {
        return 0.1f * MAX_POWER;
    } else if (MAX_POWER == 70)
    {
        return 0.09f * MAX_POWER;
    } else if (MAX_POWER == 80)
    {
        return 0.07f * MAX_POWER;
    } else if (MAX_POWER == 90 || MAX_POWER == 100)
    {
        return 0.065f * MAX_POWER;
    } else
    {
        return 0.1f * MAX_POWER;
    }
}

/************************************************************万能分隔符**************************************************************
 * 	@author:			//瑞
 *	@performance:	    //底盘走直线单环PID
 *	@parameter:		    //
 *	@time:				//23-12-17 18:08
 *	@ReadMe:			//
 ************************************************************万能分隔符**************************************************************/
float RUI_F_CHASSIS_PID(int16_t RELATIVE_ANGLE, float KP, float KI, float KD)
{
    static float INTEGRAL = 0.0;
    float ERROR[2] = { 0 }, DERIVATIVE;
    ERROR[ 1 ] = (float) RELATIVE_ANGLE;
    //积分
    INTEGRAL += ( ERROR[ 1 ] * KI );
    INTEGRAL = MATH_Limit_float(100, -100, INTEGRAL);

    //微分
    DERIVATIVE = ( ERROR[ 1 ] - ERROR[ 0 ] ) * KD;

    ERROR[ 0 ] = ERROR[ 1 ];
    float OUTPUT = MATH_Limit_float(3000, -3000, ( KP * ERROR[ 1 ] + INTEGRAL + DERIVATIVE ));

    return OUTPUT;
}
