/*!
 * @author LiNY2 
 * @date 2023/03/31 0031
 */


#include "ctrlGo2Point.h"
Go2Point::Go2Point(BaseChassis *chassisPtr) : chassis_ptr(chassisPtr) {}

void Go2Point::SetTarget(Point2D_s target_pos, float target_yaw, float start_spd, float end_spd)
{
//    enable = true;
//    arrived = false;
    status_flag = Running;
    PostureStatus_s nowpos = chassis_ptr->Chassis_GetPostureStatus();
    start_point.x = nowpos.x;
    start_point.y = nowpos.y;
    target_point = target_pos;
    start_speed = start_spd;
    final_speed = end_spd;
    total_distance = sqrtf(powf(target_point.x - start_point.x,2)+
                           powf(target_point.y - start_point.y,2));
}

void Go2Point::ResetStatus()
{
    PID_Reset(&yawPID);
    PID_Reset(&lockPID);
    chassis_ptr->Chassis_Stop();
//    enable = false;
    status_flag = Reset;
    start_speed = 0.01;
    final_speed = 0 ;
    total_distance = 0;
}

float Go2Point::Plan2PointSpeed(PostureStatus_s nowpos,float acc_ratio, float dec_ratio)
{
    if (final_speed > start_speed && acc_ratio < 0.05)
    {
#if defined(OSLIB_LOG_MODULE_ENABLED)
        log_e("final_speed>start_speed and acc_ratio is too low!");
#elif defined(OSLIB_UART_MODULE_ENABLED)
        uprintf("## Error! final_speed>start_speed and acc_ratio is too low! ##\r\n");
#endif
        return 0;
    }
    if (acc_ratio + dec_ratio > 1 )
    {
#if defined(OSLIB_LOG_MODULE_ENABLED)
        log_e("acc_ratio + dec_ratio > 1!");
#elif defined(OSLIB_UART_MODULE_ENABLED)
        uprintf("## Error! acc_ratio + dec_ratio > 1! ##\r\n");
#endif
    }
    float distance_to_target = sqrtf(powf((target_point.x - nowpos.x), 2) +
                                     powf((target_point.y - nowpos.y), 2));
    float distance_traveled = fabs(total_distance - distance_to_target);
    float ret_speed = 0;
    float uniform_speed = Max(Min(1.1 * total_distance,max_speed),min_speed);///< 匀速段速度
    if(acc_ratio < 1e-4)// 纯减速过程
    {
        uniform_speed = start_speed;
    }
    else if (dec_ratio < 1e-4)// 纯加速过程
    {
        uniform_speed = final_speed;
    }
    // 加速过程
    if (distance_traveled <= total_distance * acc_ratio && acc_ratio > 1e-4)
    {
        float k = (uniform_speed - start_speed) / (acc_ratio * total_distance);
        ret_speed = k * distance_traveled + start_speed;
    }
    else if (distance_traveled <= total_distance * ( 1 - dec_ratio ))
    {
        ret_speed = uniform_speed;
    }
    else if (distance_traveled <= total_distance)
    {
        float k = -((uniform_speed - final_speed)/(dec_ratio * total_distance));
        float x = dec_ratio * total_distance - ( total_distance - distance_traveled );
        float b = final_speed - k * dec_ratio * total_distance;
        ret_speed = k * x + b;
    }
    return ret_speed;
}

void Go2Point::Ctrl_exe()
{
    if (!status_flag) ///status_flag != Reset进入跑点
    {
        return ;
    }
    PostureStatus_s nowpos = chassis_ptr->Chassis_GetPostureStatus();

    float distance = (sqrtf(powf(nowpos.x - target_point.x,2) +
                            powf(nowpos.y - target_point.y,2)  ));
    /// 换算偏差量限制到[0,pi]
    float delta_angle = target_yaw - nowpos.yaw;
    delta_angle = AngleLimitPI(delta_angle);
    target_yaw = nowpos.yaw + delta_angle;
    float ctrl_speed = 0 , ctrl_dir = 0 ,ctrl_omega = 0;
    uint8_t keep_dir = false;
    if(distance >= lock_Circie_Th) /// 锁止区之外，使用速度规划跑
    {
        status_flag = Running;
        if (enable_always_yaw_ctrl){
            ctrl_omega = PID_GetOutput(&yawPID,target_yaw,nowpos.yaw);
        }
        ctrl_dir = AngleBetweenPoints(nowpos.x,nowpos.y,
                                      target_point.x,target_point.y);
        ctrl_speed = Plan2PointSpeed(nowpos,acc_ratio,dec_ratio);
        __LIMIT_FROM_TO(ctrl_speed,min_speed,max_speed);

    }
    else
    {
        /// 在锁止区内，但不在到达区内，用锁止pid控制
        if(distance >= arrived_Circie_Th || fabs(delta_angle) >= yaw_Ctrl_Th)
        {
            status_flag = Running;
            ctrl_omega = PID_GetOutput(&yawPID , target_yaw , nowpos.yaw);

            ctrl_speed = fabs(PID_GetOutput(&lockPID,0,distance));
            __LIMIT_FROM_TO(ctrl_speed,min_speed,max_speed);

            ctrl_dir = AngleBetweenPoints(nowpos.x,nowpos.y,
                                          target_point.x,target_point.y);
        }
        else
        {
            status_flag = Arrived;
            ctrl_speed = 0;
            ctrl_omega = 0;
            keep_dir = true;
        }
    }
    if (fabs(delta_angle) < yaw_Ctrl_Th || disable_yaw_ctrl)
    {
        ctrl_omega = 0;
    }
    chassis_ptr->Chassis_SetTargetVel(ctrl_speed,ctrl_dir,ctrl_omega,false,keep_dir);

}
void Go2Point::SetyawPID(float kp, float ki, float kd)
{
    yawPID.Kp = kp;
    yawPID.Ki = ki;
    yawPID.Kd = kd;
}
void Go2Point::SetlockPID(float kp, float ki, float kd)
{
    lockPID.Kp = kp;
    lockPID.Ki = ki;
    lockPID.Kd = kd;
}
