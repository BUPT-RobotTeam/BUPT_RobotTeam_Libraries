//
// Created by liny2 on 23-4-26.
//

#ifndef OMNI_CHASSIS_H
#define OMNI_CHASSIS_H



#include "base_chassis.h"
#include "steer_wheel.h"
#include "chassis_common_config.h"

#ifdef __cplusplus
extern "C"
{
#endif
///CStyle-Link inc,func and var begin
#include "main.h"
#include "BUPTLib_port.h"
#include "../SimpleLib/utils/utils.h"
#include "../SimpleLib/utils/vec.h"
//#include "point.h"
///CStyle-Link func and var end
#ifdef __cplusplus
}
#endif

class OmniChassis : public  BaseChassis
{
public:
    OmniChassis();
    ~OmniChassis() override;
    /**
     * @brief 横辊子全向轮运动学解算
     * @param speed 速度大小/rpm 尽量传入非负数
     * @param dir 速度方向/rad
     * @param omega 自转角速度(rad/s)，逆时针为正方向
     * @note 控制方法为四个轮子同时转向，始终保持一个方向，并根据速度方向偏差转动每个单独轮以进行补偿
     *       规定dir的范围是[0,pi],[-pi,0]
     **/
    void ChassisMove(float target_speed, float target_dir, float target_omega) override final;
    /**
     * @brief 电机要延迟初始化，不能和对象一起初始化
     */
    void ChassisInit() override final;

protected:

    ///< 车轮正方形与底盘坐标系x轴的夹角/rad
    float InstallAngle[4] = {
            __ANGLE2RAD(-45),
            __ANGLE2RAD(-135),
            __ANGLE2RAD(135),
            __ANGLE2RAD(45)
    };

    CAN_Message_u MotorRpmCANMsg[4]{};

};


#endif//OMNI_CHASSIS_H
