/*!
 * @author LiNY2 
 * @date 2023/03/27 0027
 */


#ifndef CTRLPATHTRACKER_H
#define CTRLPATHTRACKER_H

/**
 * Log default configuration for EasyLogger.
 * NOTE: Must defined before including the <elog.h>
 */
#if !defined(LOG_TAG)
#define LOG_TAG "Track"
#endif
#undef LOG_LVL
#define CHASSIS_LOG_LVL ELOG_LVL_INFO
#if defined(CHASSIS_LOG_LVL)
#define LOG_LVL CHASSIS_LOG_LVL
#endif

#include "base_chassis.h"
#include "ctrlGo2Point.h"

#ifdef __cplusplus
extern "C"
{
#endif
///CStyle-Link inc,func and var begin
#include "../SimpleLib/utils/utils.h"
#include "../SimpleLib/utils/vec.h"
#include "BUPTLib_port.h"
#include "point.h"


    ///CStyle-Link func and var end
#ifdef __cplusplus
}
#endif


class PathTracker
{
public:
    explicit PathTracker(BaseChassis *chassisPtr);
    virtual ~PathTracker();
    enum TrackerFlag{
        Reset = 0 ,
        Running ,
        InitGo2LastPt,
        Go2LastPT,
        Arrived
    };

    uint8_t disable_yaw_ctrl = {false};
    uint8_t enable_always_yaw_ctrl = {false};
    void SetPathPoints(const PlanPoint *points, uint16_t point_num);
    void Ctrl_exe();
    void ResetStatus();
    void SetyawPID(float kp,float ki,float kd);
    void SetnormalCorrPID_x(float kp,float ki,float kd);
    void SetnormalCorrPID_y(float kp,float ki,float kd);
    TrackerFlag getStatusFlag() const
    {
        return status_flag;
    }


protected:


    BaseChassis *chassis_ptr{nullptr};
    Go2Point *go2Point;

    PID_s yawPID = {
            .Kp = 3.00,
            .Kd = 2.0,
            .Ki = 0.00,
            .int_max = 10.0,
            .int_duty = 10.0,
            .ctrl_max = 5.0};///< 角度环PID
    PID_s normalCorrPID_x = {
            .Kp = 0.2,
            .Kd = 0.0,
            .Ki = 0.00,
            .int_max = 10.0,
            .int_duty = 1.0,
            .ctrl_max = 10.0
    };///< x方向法向偏差PID
    PID_s normalCorrPID_y = {
            .Kp = 0.2,
            .Kd = 0.0,
            .Ki = 0.00,
            .int_max = 10.0,
            .int_duty = 0.0,
            .ctrl_max = 10.0
    };///< y方向法向偏差PID

    TrackerFlag status_flag {Reset}; ///< 状态标志位

    const PlanPoint * path_point{nullptr};///< 要跟踪的轨迹点
    const float passedWaypointCircleTh {0.05}; //m
    uint16_t point_num{0};///< 轨迹点的数量
    uint16_t point_index{0};///< 目前跟踪的点

//    vec now_speed_vec{0};          // 底盘当前的速度向量
//    vec target_speed_vec{0};       // 下一个目标点的速度向量（速度在x、y方向的分量）
//    vec now_pos2now_target{0};     // 底盘当前坐标到当前目标点的位移向量
//    vec now_pos2next_target{0};    // 底盘到下一个目标点的位移向量
//    vec now_target2last_target{0}; // 当前目标点到上一个目标点的向量

    uint8_t UpdatePointIndex(PostureStatus_s nowpos,CMD_Vel_t nowcmd);
    void TrackVector(vec now_speed_vec, vec target_speed_vec,
    vec now_target2last_target, vec now_pos2now_target,float now_yaw, float target_yaw);

};



#endif//CTRLPATHTRACKER_H
