/*!
 * @author LiNY2 
 * @date 2023/03/26 0026
 */


#include "steer_wheel.h"

#ifdef USE_CHASSIS_RUDDER

#ifdef __cplusplus
extern "C"
{
#endif
///CStyle-Link inc,func and var begin

#include "../MotorLib/dji_boardv2_can.h"
#include "../MotorLib/motor_driver.h"
#include "../MotorLib/vesc_can.h"
#include "../SimpleLib/utils/utils.h"
#include "BUPTLib_port.h"
extern RM_MotorStatus_t RM_MotorStatus[8];

///CStyle-Link func and var end
#ifdef __cplusplus
}
#endif

SW_SteerMotor::SW_SteerMotor() = default;

void SW_SteerMotor::motorInit()
{
    for (uint16_t i = 0; i < 4; ++i)
    {
        dji_MotorOn(STEER_WHEEL_DJIV2_BOARDID, i + 1);
        dji_PosCfg(STEER_WHEEL_DJIV2_BOARDID, i + 1, STEER_WHEEL_MAX_SPEED);
    }
}

void SW_SteerMotor::SetPos(float pos[4])
{
    SW_SteerMotors_LimitPos(pos);
    static int16_t target_pos_int[4];
    for (int i = 0; i < 4; i++)
    {
        target_pos_int[i] = (int16_t) __RAD2ANGLE(pos[i]);
    }
    dji_PosCtrlAll(1, target_pos_int);
}
void SW_SteerMotor::SW_SteerMotors_LimitPos(float pos[4])
{
    for (int i = 0; i < 4; i++)
    {
        __LIMIT_FROM_TO(pos[i], STEER_WHEEL_MIN_POS, STEER_WHEEL_MAX_POS);
    }
}
void SW_SteerMotor::SW_SteerMotors_GetMTRInfo()
{
    for (int i = 0; i < 4; i++)
    {
//        dji_ReadInfo(STEER_WHEEL_DJIV2_BOARDID, i + 1);//此函数返回的量纲不对，而且不能保证每个电机信息都被收到
    }
}
///<以下是驱动轮成员函数

void SW_DriveMotor::motorInit()
{
    for (int i = 0; i < 4; ++i)
    {
        can_id[i] = i + DRIVE_WHEEL_VESC_ID_BASE;
    }
}
void SW_DriveMotor::SetSpeed(float *speed)
{
    SW_DriveMotors_LimitSpeed(speed);
    for (int i = 0; i < 4; i++)
    {
        speed[i] = SW_Speed2eRPM(speed[i]);
        if ( fabs(speed[i]) <= DRIVE_WHEEL_MIN_ERPM)
        {
            comm_can_set_duty(can_id[i],0);
        }
        else
        {
            comm_can_set_rpm(can_id[i], speed[i]);
        }
        if (1 == i)
            Delay_msec(1); // 发送间隔1ms
    }
}
void SW_DriveMotor::Handbrake(float hand_brake_current)
{
    for (int i = 0; i < 4; i++)
    {
        comm_can_set_handbrake(can_id[i], hand_brake_current);
        if (2 == i)
            Delay_msec(1); // 发送间隔1ms
    }
}

void SW_DriveMotor::SW_DriveMotors_LimitSpeed(float speed[4])
{
    for (int i = 0; i < 4; i++)
    {
        // if (speed[i] < 0) // 舵向角度差[90,270]反转的情况
        // {
        //     speed[i] *= -1;
        //     LIMIT(speed[i], DRIVE_WHEEL_MAX_SPEED);
        //     speed[i] *= -1;
        //     continue;
        // }
//        if (fabs(speed[i])>=DRIVE_WHEEL_MAX_SPEED)
//        {
//            log_w("vel > 4.0");
//        }
        __LIMIT(speed[i], DRIVE_WHEEL_MAX_SPEED);
    }
}
float SW_DriveMotor::SW_Speed2eRPM(float speed)
{
    return ((speed * 30) / (PI * DRIVE_WHEEL_RADIUS)) * DRIVE_MOTOR_POLE_PARIS * DRIVE_WHEEL_REDUCTION_RATIO;
}


#endif