/*!
 * @author LiNY2 
 * @date 2023/03/26 0026
 */


#ifndef RUDDER_CHASSIS_H
#define RUDDER_CHASSIS_H


#include "base_chassis.h"
#include "steer_wheel.h"
#include "chassis_common_config.h"


#ifdef __cplusplus
extern "C"
{
#endif
// CStyle-Link inc,func and var begin

#include "main.h"
#include "BUPTLib_port.h"
#include "../SimpleLib/utils/utils.h"
#include "../SimpleLib/utils/vec.h"
//#include "point.h"




#ifdef __cplusplus
}
#endif
// CStyle-Link func and var end


class RudderChassis : public BaseChassis
{
public:
    /**
     * 构造函数，同时初始化舵轮舵向电机和驱动电机指针。
     */
    RudderChassis();
    /**
     * 析构，释放舵轮舵向电机和驱动电机指针；
     */
    ~RudderChassis() override;
    /**
     * @brief 底盘速度矢量映射为四个轮子的线速度与舵向角
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
    /**
     * @brief 两帧角度差太大时通过反转驱动轮方向减小转动量，使舵向电机走劣弧
     * @param now_dir [in]当前朝向
     * @param target_dir [in,out]目标朝向，若角度差超过180*，会被修改为走劣弧
     * @param target_speed [in,out]目标速度，若角度差超过180*，会被修改为反向
     */
    static void RudderChassis_TurnMinorArc(float now_dir, float &target_dir, float &target_speed);
    /**
     * @brief 单轮控制
     * @param index 电机编号
     * @param speed 驱动电机转速/rpm
     * @param dir 速度方向/rad
     * @note 测试函数，一般不用
     */
    void RudderChassis_SingleWheelCtrl(int index, float speed, float dir);
    /**
     * @brief 单独控制舵向
     * @param yaw 目标舵向/rad
     * @param max_omega 最大角速度/rpm
     * @note 测试函数，一般不用
     */
    void RudderChassis_SteerWhellDirCtrl(float dir[4]);

    void mpc_move(float tar_speed[],float tar_dir[]);


protected:
    SW_DriveMotor * driveMotors{nullptr};
    SW_SteerMotor * steerMotors{nullptr};
    uint8_t send_ctrl_msg_flag = {true};
    int8_t omega_ratio[4][2] = {
            {-1, -1},
            {-1,  1},
            { 1,  1},
            { 1, -1}
    };


};

#endif //RUDDER_CHASSIS_H
