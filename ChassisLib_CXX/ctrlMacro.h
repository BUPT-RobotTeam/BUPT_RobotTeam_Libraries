/*!
 * @author LiNY2 
 * @date 2023/03/31 0031
 */


#ifndef CTRLMACRO_H
#define CTRLMACRO_H


/**
 * Log default configuration for EasyLogger.
 * NOTE: Must defined before including the <elog.h>
 */
#if !defined(LOG_TAG)
#define LOG_TAG "Macro"
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


///CStyle-Link func and var end
#ifdef __cplusplus
}
#endif

class MacroSpeedCtrl
{
public:
    explicit MacroSpeedCtrl(BaseChassis *chassisPtr);
    void Ctrl_exe(float target_speed,float target_omega);
    void ResetStatus();
    void SetspeedCtrlPID(float kp,float ki,float kd);
    void SetomegaCtrlPID(float kp,float ki,float kd);
    float omega_Ctrl_Th;
protected:
    BaseChassis *chassis_ptr{nullptr};
    PID_s SpeedctrlPID = {
            .Kp = 1.5,
            .Kd = 0.00,
            .Ki = 0.01,
            .int_max = 10.0,
            .int_duty = 1.0,
            .ctrl_max = 5.0
    };///< 宏观线速度环PID
    PID_s OmegactrlPID = {
            .Kp = 10,
            .Kd = 0.5,
            .Ki = 0,
            .int_max = 10.0,
            .int_duty = 1.0,
            .ctrl_max = 10.0
    };///< 宏观角速度环PID
};

class YawTurning
{
public:
    explicit YawTurning(BaseChassis *chassisPtr);
    void Ctrl_exe();
    void ResetStatus();
    void SetyawPID(float kp,float ki,float kd);
    void SetTarget(float target_yaw,uint8_t degree = false);
    float yaw_Ctrl_Th;
    uint8_t keep_dir = false;///< 在舵轮场景下，有时要让舵向锁住
protected:
    float target_yaw {1.57};
    BaseChassis *chassis_ptr{nullptr};
    PID_s yawPID = {
            .Kp = 1.5,
            .Kd = 0.00,
            .Ki = 0.01,
            .int_max = 10.0,
            .int_duty = 1.0,
            .ctrl_max = 5.0
    };///< 角度环PID

};



#endif//CTRLMACRO_H
