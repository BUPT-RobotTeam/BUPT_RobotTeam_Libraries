/*!
 * @author LiNY2 
 * @date 2023/03/26 0026
 */


#ifndef STEER_WHEEL_H
#define STEER_WHEEL_H

#include "chassis_common_config.h"

#ifdef USE_CHASSIS_RUDDER

#include "base_chassis.h"

#ifdef __cplusplus
extern "C"
{
#endif
///CStyle-Link inc,func and var begin
#include "../MotorLib/motor_driver.h"
#include "../SimpleLib/utils/utils.h"
#include "BUPTLib_port.h"


///CStyle-Link func and var end
#ifdef __cplusplus
}
#endif


/* 驱动轮使用什么驱动器（都用cpp了，继承吧）---------------------------------*/
//#define DRIVE_MTR_USE_VESC
// #define DRIVE_MTR_USE_ODRIVE
/**
 * 舵向电机，默认使用大疆驱动板实现
 */
class SW_SteerMotor
{
public:
    int16_t now_pos[4]{0};
    uint8_t can_send_flag{0};


    SW_SteerMotor();
    /**
     * @brief 实例初始化时，不能保证oslib初始化完成，应当手动调用初始化函数
     */
    void motorInit();
    /**
     * @brief 发送CAN报文，同时设置4个舵轮的舵向；舵轮舵向电机角度环控制的虚函数
     * @param pos 目标角度/rad 假定已经乘过减速比。需要转为degree发给电调
     */
    virtual void SetPos(float pos[4]);
    /**
     * @brief 限制电机旋转范围
     * @param pos
     */
    void SW_SteerMotors_LimitPos(float pos[4]);
    /**
     * @brief  查询舵向电机角度
     * @note 驱动板发回0x281的反馈CAN消息，数据读到全局结构体Robomaster_Motor中
     */
    void SW_SteerMotors_GetMTRInfo();

    virtual ~SW_SteerMotor() = default;
};
/**
 * 驱动轮电机，默认使用VESC实现
 */
class SW_DriveMotor
{
public:
    float target_duty[4]{0};/// 占空比
    float target_curr[4]{0};/// 电流
    float target_rpm[4]{0}; /// 转速
    uint8_t motor_mode{MTR_CTRL_RPM};
    uint8_t can_send_flag{0};
    uint32_t can_id[4];
    /**
     * 舵轮驱动轮速度环控制的虚函数
     * @param speed 四个电机速度
     */
    virtual void SetSpeed(float speed[4]);///
    /**
     * 舵轮驱动轮手刹控制的虚函数
     * @param hand_brake_current 手刹电流
     */
    virtual void Handbrake(float hand_brake_current);///

    /**
     * @brief 实例初始化时，不能保证oslib初始化完成，应当手动调用初始化函数
     */
    void motorInit();
    /**
     * @brief 限制速度绝对值大小
     *
     * @param speed
     * @note 注意最小速度不能太大，否则底盘不会静止
     */
    void SW_DriveMotors_LimitSpeed(float speed[4]);
    /**
     * @brief 将底盘速度m/s转为电机电转速
     *
     * @param speed m/s
     * @return float eRPM=RPM*Pole Pairs
     */
    float SW_Speed2eRPM(float speed);
    /**
     * 虚析构函数，避免内存泄漏
     */
    virtual ~SW_DriveMotor() = default;
};

#endif//#ifdef USE_CHASSIS_RUDDER

#endif//STEER_WHEEL_H
