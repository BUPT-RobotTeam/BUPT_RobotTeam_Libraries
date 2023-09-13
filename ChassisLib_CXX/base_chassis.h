/*!
 * @file
 * @author LiNY2 
 * @date 2023/03/25 0025
 */


#ifndef BASE_CHASSIS_H
#define BASE_CHASSIS_H

/**
 * Log default configuration for EasyLogger.
 * NOTE: Must defined before including the <elog.h>
 */
#if !defined(LOG_TAG)
#define LOG_TAG                    "Chassis"
#endif
#undef LOG_LVL
#define  CHASSIS_LOG_LVL ELOG_LVL_INFO
#if defined(CHASSIS_LOG_LVL)
#define LOG_LVL                    CHASSIS_LOG_LVL
#endif
#include "chassis_common_config.h"
#ifdef __cplusplus
extern "C"
{
#endif
//CStyle-Link inc,func and var begin
#include "BUPTLib_port.h"
#include "../SimpleLib/utils/vec.h"
#include "../SimpleLib/utils/utils.h"

/**
 * @brief 控制模式，用户侧写状态机使用。
 *
 */
typedef enum CHASSIS_CTRL_MODE
{
    CTRL_MODE_NONE = 0,
    CTRL_MODE_HANDLE = 1,
    CTRL_MODE_CMD,
    CTRL_MODE_TRACK,///< 推荐用来调试
    CTRL_MODE_GO_TO_POINT,///< 推荐用来调试
    CTRL_MODE_TUNING, ///< 推荐用来调试
    CTRL_MODE_EXTERNAL, // 外界直接置位速度和方向，更加适用于比赛状态机。
} CHASSIS_CTRL_MODE;

void CAN_Callback_Location_ReadPos_X(CAN_ConnMessage *data);
void CAN_Callback_Location_ReadPos_Y(CAN_ConnMessage *data);
void CAN_Callback_Location_ReadPos_Yaw(CAN_ConnMessage *data);

//CStyle-Link func and var end
#ifdef __cplusplus
}
#endif

struct CMD_Vel_t
{
    float speed; ///< 速度大小
    float dir; ///< 速度方向
    float omega; ///< 角速度
}; // 类似ROS中的/cmd_vel
struct PostureStatus_s
{
    float x; // 单位m
    float y; // 单位m
    float last_x;
    float last_y;
    float yaw; // 偏航角/rad
    float last_yaw;
    float speed;         // 线速度m/s
    float speed_x;       // x方向分速度
    float speed_y;       // y方向分速度
    float omega;         // 角速度rad/s
    float acc;           // 加速度 = Δv2-Δv1 = v2-2v1+v0
    float speed_err2;    // Δv2
    float speed_err1;    // Δv1
    float laser_pos_yaw; // 由激光计算出的偏航角
    float pos_corr_x;    // x方向坐标修正量(激光等设备计算得到)
    float pos_corr_y;
};
class BaseChassis
{
public:
    /**
     * 位置模式
     */
    enum Chassis_Pos_Mode
    {
        PosModeRelative = 0,///<相对车体自身
        PosModeAbsolute = 1///<绝对于世界坐标系
    };
    /**
     * 虚函数，运动解算接口
     * @param target_speed 速度大小，尽量传入非负数，单位 m/s(整个程序中统一就好，也可以是cm)
     * @param target_dir 速度方向/rad
     * @param target_omega 自转角速度(rad/s)，逆时针为正方向
     */
    virtual void ChassisMove(float target_speed, float target_dir, float target_omega) = 0;
    /**
     * @brief 初始化电机等需要延迟初始化的对象。
     */
    virtual void ChassisInit() = 0;
    /**
     * 保持当前运动状态
     */
    void ChassisMove(){
        ChassisMove(targetVel.speed,targetVel.dir,targetVel.omega);
    };

    void Chassis_PrintPostureStatus();
    /**
      * @brief 由用户实现 | 更新底盘位姿状态
      * @note  此函数调用频率不应低于控制频率
      */
    void Chassis_UpdatePostureStatus();
    /**
     * 获取位姿
     * @return 位姿结构体，常引用
     */
    const PostureStatus_s &Chassis_GetPostureStatus() const
    {
        return postureStatus;
    };
    /**
     * 获取期望速度
     * @return 位姿结构体，常引用
     */
    const CMD_Vel_t &Chassis_GettargetVel() const
    {
        return targetVel;
    };
    /**
     * 设置期望运动状态
     * @param speed 速度大小，尽量传入非负数，单位 m/s(整个程序中统一就好，也可以是cm)
     * @param dir 速度方向/rad
     * @param omega 自转角速度(rad/s)，逆时针为正方向
     * @param keep_dir
     * @param keep_omega
     * @param keep_speed
     */
    void Chassis_SetTargetVel(float speed,float dir,float omega,
    uint8_t keep_speed = false,uint8_t keep_dir = false,uint8_t keep_omega = false);
    /**
     * 停下底盘，舵轮会保持舵向
     */
    void Chassis_Stop(){
        targetVel.speed = 0.00;
        targetVel.omega = 0.00;
    };
    /**
     * 设置位姿模式，有绝对坐标和相对坐标模式
     * @param posMode
     * @refitem PosModeAbsolute
     * @refitem PosModeRelative
     */
    void Chassis_SetPosMode(Chassis_Pos_Mode posMode);
    /**
     * 刹车标志位
     */
    bool handbrake_flag{false};
    /**
     * 方向锁定标志位
     */
    bool lock_yaw_flag{false};
    bool Chassis_PrintPostureStatus_Flag{true};
    /**
     * 析构函数，虚函数。
     */
    virtual ~BaseChassis() = default;

protected:
    /**
     * 存储控制量
     */
    CMD_Vel_t targetVel{0,0,0};
    Chassis_Pos_Mode pos_mode{PosModeAbsolute};
    PostureStatus_s postureStatus{0};

//    float th_yawCtrl{0.01745};  ///<1°的死区
    ///< 死区设置移动到控制器中。



};

#endif//BASE_CHASSIS_H
