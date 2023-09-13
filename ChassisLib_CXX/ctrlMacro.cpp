/*!
 * @author LiNY2 
 * @date 2023/03/31 0031
 */


#include "ctrlMacro.h"
MacroSpeedCtrl::MacroSpeedCtrl(BaseChassis *chassisPtr) : chassis_ptr(chassisPtr) {}
void MacroSpeedCtrl::Ctrl_exe(float target_speed, float target_omega)
{
    PostureStatus_s nowpos = chassis_ptr->Chassis_GetPostureStatus();
    float now_speed = sqrtf(nowpos.speed_x*nowpos.speed_x +
                            nowpos.speed_y*nowpos.speed_y);
    float now_omega = nowpos.omega;
    float ctrl_speed = PID_GetOutput(&SpeedctrlPID,target_speed,now_speed);
    float ctrl_omega = PID_GetOutput(&OmegactrlPID,target_omega,now_omega);

    if(fabs(target_omega)<omega_Ctrl_Th)
    {
        ctrl_omega = 0;
    }
    chassis_ptr->Chassis_Stop();
}

void MacroSpeedCtrl::SetspeedCtrlPID(float kp, float ki, float kd)
{
    SpeedctrlPID.Kp = kp;
    SpeedctrlPID.Kd = kd;
    SpeedctrlPID.Ki = ki;
}
void MacroSpeedCtrl::SetomegaCtrlPID(float kp, float ki, float kd)
{
    OmegactrlPID.Kp = kp;
    OmegactrlPID.Kd = kd;
    OmegactrlPID.Ki = ki;
}

void MacroSpeedCtrl::ResetStatus()
{
    PID_Reset(&OmegactrlPID);
    PID_Reset(&SpeedctrlPID);
    chassis_ptr->Chassis_SetTargetVel(0,0,0,false,true,false);

}

YawTurning::YawTurning(BaseChassis *chassisPtr) : chassis_ptr(chassisPtr) {}

void YawTurning::Ctrl_exe()
{
    PostureStatus_s nowpos = chassis_ptr->Chassis_GetPostureStatus();
    float delta_angle = target_yaw - nowpos.yaw;
    delta_angle = AngleLimitPI(delta_angle);

    target_yaw = nowpos.yaw + delta_angle;
    float omega = PID_GetOutput(&yawPID,target_yaw,nowpos.yaw);

    if(fabs(delta_angle) < yaw_Ctrl_Th)
    {
        omega = (keep_dir) ? 0.01 : 0;
    }
    chassis_ptr->Chassis_SetTargetVel(0,-1,omega);

}

void YawTurning::SetTarget(float target_yaw, uint8_t degree)
{
    if (degree)
    {
        YawTurning::target_yaw = __ANGLE2RAD(target_yaw);
        return ;
    }
    YawTurning::target_yaw = target_yaw;
}


void YawTurning::SetyawPID(float kp, float ki, float kd)
{
    yawPID.Kp = kp;
    yawPID.Kd = kd;
    yawPID.Ki = ki;
}
void YawTurning::ResetStatus()
{
    PID_Reset(&yawPID);
    target_yaw = 1.57;
    chassis_ptr->Chassis_Stop();
}
