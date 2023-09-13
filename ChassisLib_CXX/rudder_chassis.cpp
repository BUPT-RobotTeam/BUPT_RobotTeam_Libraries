/*!
 * @author LiNY2 
 * @date 2023/03/26 0026
 */


#include "rudder_chassis.h"

#ifdef __cplusplus
extern "C"
{
#endif

// CStyle-Link inc,func and var begin
#include "main.h"
#include "BUPTLib_port.h"
#include "../MotorLib/motor_driver.h"
#include "../MotorLib/vesc_can.h"
#include "../MotorLib/dji_boardv2_can.h"
//#include "point.h"

// CStyle-Link inc,func and var end

#ifdef __cplusplus
}
#endif

#ifdef USE_CHASSIS_RUDDER

/**
* 构造函数，同时初始化舵轮舵向电机和驱动电机指针。
*/
RudderChassis::RudderChassis()
{
    driveMotors = new SW_DriveMotor();
    steerMotors = new SW_SteerMotor();
}
/**
 * 析构，释放舵轮舵向电机和驱动电机指针；
 */
RudderChassis::~RudderChassis()
{
    delete driveMotors;
    delete steerMotors;
}

void RudderChassis::ChassisMove(float target_speed, float target_dir, float target_omega)
{
    float absolute_angle_offset = 0; //使用绝对坐标时，根据全场定位测得的偏航角进行补偿
    if (pos_mode == BaseChassis::PosModeAbsolute)
    {
        absolute_angle_offset = postureStatus.yaw - (PI/2.0) ;
        if (absolute_angle_offset > PI)
        {
            absolute_angle_offset = - ( 2.0 * PI - absolute_angle_offset );
        }
    }
    target_dir -= absolute_angle_offset;
    float vx = target_speed * cos(target_dir);
    float vy = target_speed * sin(target_dir);
    __LIMIT(target_omega , MAX_ROTATE_VEL);
    float steerWheelPos[4];
    float driveWheelSpeed[4];

    if( fabs(target_speed) > 1e-4 || fabs(target_omega) > 1e-4 )
    {
        for (int i = 0; i < 4; ++i)
        {
            float self_turn_vel_x = target_omega * RudderChassis_Front2Back / 2; // 自转切向速度的分量
            float self_turn_vel_y = target_omega * RudderChassis_Left2Right / 2;
            float vel_sum_x = vx + omega_ratio[i][0] * self_turn_vel_x; // 速度合成，考虑底盘形状
            float vel_sum_y = vy + omega_ratio[i][1] * self_turn_vel_y;
            float real_vel = sqrt(pow(vel_sum_x, 2) + pow(vel_sum_y, 2));
            float real_dir = atan2f(vel_sum_y, vel_sum_x); //

            // 附加角速度后的真实速度方向
            RudderChassis_TurnMinorArc(RM_MotorStatus[i].pos / STEER_WHEEL_REDUCTION_RATIO, real_dir, real_vel);

            //保存当前位置，否则：当舵向从90度到-90时，舵向反转
            RM_MotorStatus[i].pos = real_dir * STEER_WHEEL_REDUCTION_RATIO;

            steerWheelPos[i] = real_dir * STEER_WHEEL_REDUCTION_RATIO;                                        // 乘舵轮减速比
//            real_vel = real_vel > 0 ? real_vel + DRIVE_WHEEL_MIN_SPEED : real_vel - DRIVE_WHEEL_MIN_SPEED; // 补偿启动速度
            driveWheelSpeed[i] = real_vel;
        }
    }
    else // 速度和角速度都为0时要保证舵向有效
    {
        for (int i = 0; i < 4; i++)
        {
            RudderChassis_TurnMinorArc(RM_MotorStatus[i].pos / STEER_WHEEL_REDUCTION_RATIO, target_dir, steerWheelPos[i]);
            RM_MotorStatus[i].pos = target_dir * STEER_WHEEL_REDUCTION_RATIO;
            steerWheelPos[i] = target_dir * STEER_WHEEL_REDUCTION_RATIO;
            driveWheelSpeed[i] = 0;
        }
    }

    // 数值限制在舵轮类实现

    //向驱动板发送命令
    steerMotors->SetPos(steerWheelPos);
    if(handbrake_flag)
    {
        driveMotors->Handbrake(10.0);
        log_i("RudderChassis|sended handbrake command!\r\n");
    }
    else
    {
        driveMotors->SetSpeed(driveWheelSpeed);
    }
}

void RudderChassis::ChassisInit()
{
    steerMotors->motorInit();
    driveMotors->motorInit();
}

void RudderChassis::mpc_move(float tar_speed[], float tar_dir[])
{
    float steerWheelPos[4];
    float driveWheelSpeed[4];
    for (int i = 0; i < 4; ++i)
    {
//        float self_turn_vel_x = target_omega * RudderChassis_Front2Back / 2; // 自转切向速度的分量
//        float self_turn_vel_y = target_omega * RudderChassis_Left2Right / 2;
//        float vel_sum_x = vx + omega_ratio[i][0] * self_turn_vel_x; // 速度合成，考虑底盘形状
//        float vel_sum_y = vy + omega_ratio[i][1] * self_turn_vel_y;
        float real_vel = tar_speed[i];//sqrt(pow(vel_sum_x, 2) + pow(vel_sum_y, 2));
        float real_dir = tar_dir[i];//atan2f(vel_sum_y, vel_sum_x); //

//         __LIMIT(real_vel, 3.0);
        // 附加角速度后的真实速度方向
        RudderChassis_TurnMinorArc(RM_MotorStatus[i].pos / STEER_WHEEL_REDUCTION_RATIO, real_dir, real_vel);

        //保存当前位置，否则：当舵向从90度到-90时，舵向反转
        RM_MotorStatus[i].pos = real_dir * STEER_WHEEL_REDUCTION_RATIO;

        steerWheelPos[i] = real_dir * STEER_WHEEL_REDUCTION_RATIO;                                        // 乘舵轮减速比
//        real_vel = real_vel > 0 ? real_vel + DRIVE_WHEEL_MIN_SPEED : real_vel - DRIVE_WHEEL_MIN_SPEED; // 补偿启动速度
        driveWheelSpeed[i] = real_vel;
    }
    //向驱动板发送命令
//    if ( fabs(driveWheelSpeed[0]) > 4.0 )
//    {
//                    return;
//        log_w("vel > 4.0");
//    }
    steerMotors->SetPos(steerWheelPos);
    if(handbrake_flag)
    {
        driveMotors->Handbrake(10.0);
        log_i("RudderChassis|sended handbrake command!\r\n");
    }
    else
    {
        driveMotors->SetSpeed(driveWheelSpeed);
    }
}

void RudderChassis::RudderChassis_TurnMinorArc(float now_dir, float &target_dir, float &target_speed)
{
    float delta_dir = target_dir - now_dir; // 偏差量

    if (fabs(delta_dir) > __ANGLE2RAD(90) && fabs(delta_dir) < __ANGLE2RAD(270))
    {
        if (target_dir < 0)
        {
            target_dir += PI;
        }
        else
        {
            target_dir -= PI;
        }
        target_speed = -target_speed;
    }
    else
    {
        //  uprintf(">270,delta_dir:%.2f\r\n",delta_dir);
        if (delta_dir > __ANGLE2RAD(270))
        {
            delta_dir -= 6.28;
        }
        else if (delta_dir < -__ANGLE2RAD(270))
        {
            delta_dir += 6.28;
        }

        target_dir = now_dir + delta_dir;
    }
}
void RudderChassis::RudderChassis_SingleWheelCtrl(int index, float speed, float dir)
{
    // 数值限位
    __LIMIT(speed, DRIVE_WHEEL_MAX_SPEED);
    __LIMIT(dir, STEER_WHEEL_MAX_POS);
    //向驱动板发送命令
    dji_PosCtrl(STEER_WHEEL_DJIV2_BOARDID,index,__RAD2ANGLE(dir * STEER_WHEEL_REDUCTION_RATIO));
    //    DJI_PosCtrl(CAN1, index, ABSOLUTE_MODE, (int32_t)__RAD2ANGLE(dir * STEER_WHEEL_REDUCTION_RATIO));
    comm_can_set_rpm(driveMotors->can_id[index], speed);
}
void RudderChassis::RudderChassis_SteerWhellDirCtrl(float *dir)
{
    steerMotors->SetPos(dir);
}



#endif