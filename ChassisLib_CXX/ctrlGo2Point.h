/*!
 * @file
 * @author LiNY2 
 * @date 2023/03/31 0031
 */


#ifndef CTRLGO2POINT_H
#define CTRLGO2POINT_H

/**
 * Log default configuration for EasyLogger.
 * NOTE: Must defined before including the <elog.h>
 */
#if !defined(LOG_TAG)
#define LOG_TAG "Go2PT"
#endif
#undef LOG_LVL
#define CHASSIS_LOG_LVL ELOG_LVL_INFO
#if defined(CHASSIS_LOG_LVL)
#define LOG_LVL CHASSIS_LOG_LVL
#endif

#include "base_chassis.h"


#ifdef __cplusplus
extern "C"
{
#endif
//CStyle-Link inc,func and var begin
#include "../SimpleLib/utils/utils.h"
#include "../SimpleLib/utils/vec.h"
#include "BUPTLib_port.h"


    //CStyle-Link func and var end
#ifdef __cplusplus
}
#endif


class Go2Point
{
public:
    /**
     * 构造函数。
     * @param chassisPtr 被控底盘基类指针
     */
    explicit Go2Point(BaseChassis *chassisPtr);
    /**
     * 三种状态量
     */
    enum Go2PointFlag{
        Reset = 0, ///<未设置目标，等待控制开始
        Running, ///< 正在跑点
        Arrived ///< 到达
    };
    ///< 关闭yaw控制
    uint8_t disable_yaw_ctrl = {false};
    ///< 使能持续yaw控制
    uint8_t enable_always_yaw_ctrl = {true};
    /**
     * 初始化跑点目标
     * @param target_pos 二维坐标点
     * @param target_yaw 目标偏航角
     * @param start_spd 启动速度
     * @param end_spd 结束速度
     */
    void SetTarget(Point2D_s target_pos,float target_yaw,float start_spd,float end_spd);
    /**
     * 运行跑点控制，应当在控制app中按频率调用。
     * @note 注意只调用这个函数不会让底盘运动。需要调用BaseChassis::ChassisMove();
     */
    void Ctrl_exe();
    /**
     * 重置跑点状态
     */
    void ResetStatus();
    void SetyawPID(float kp,float ki,float kd);
    void SetlockPID(float kp,float ki,float kd);
    Go2PointFlag getStatusFlag() const
    {
        return status_flag;
    }
    void setMinSpeed(float minSpeed)
    {
        min_speed = minSpeed;
    }
    void setMaxSpeed(float maxSpeed)
    {
        max_speed = maxSpeed;
    }
    void setAccRatio(float accRatio)
    {
        acc_ratio = accRatio;
    }
    void setDecRatio(float decRatio)
    {
        dec_ratio = decRatio;
    }

protected:
    BaseChassis *chassis_ptr{nullptr};
    float arrived_Circie_Th = {0.005};///<< m 到达半径阈值
    float lock_Circie_Th = {0.15};     ///<< m 启用原地锁定PID的距离半径阈值
    float yaw_Ctrl_Th = {0.01745  / 2};
    PID_s yawPID = {
            .Kp = 3.50,
            .Kd = 3.0,
            .Ki = 0.00,
            .int_max = 10.0,
            .int_duty = 10.0,
            .ctrl_max = 2.0};///<< 跑点模式角度环PID
    PID_s lockPID = {
            .Kp = 3.0,
            .Kd = 2.0,
            .Ki = 0.00,
            .int_max = 1.0,
            .int_duty = 5.0,
            .ctrl_max = 0.3
    };///<< 跑点模式锁止PID
    Point2D_s start_point{0}; ///<<起始点，应当为小车坐标
    Point2D_s target_point{0}; ///< 终点
    Go2PointFlag status_flag{Reset};
//    uint8_t arrived {false};///< 到达
//    uint8_t enable {false};///< 运行中
    float target_yaw{1.5708}; ///< 目标偏航角
    float start_speed{0}; ///< 起始速度
    float final_speed{0}; ///< 目标速度
    float min_speed{DRIVE_WHEEL_MIN_SPEED}; ///< 最小速度
    float max_speed{DRIVE_WHEEL_MAX_SPEED}; ///< 最大速度
    float acc_ratio{0.1};
    float dec_ratio{0.4};
    float total_distance{0.00};
    float Plan2PointSpeed(PostureStatus_s nowpos,float acc_ratio, float dec_ratio);

};

#endif//CTRLGO2POINT_H
