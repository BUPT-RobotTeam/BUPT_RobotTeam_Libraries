//
// Created by liny2 on 23-4-26.
//

#include "omni_chassis.h"


#ifdef __cplusplus
extern "C"
{
#endif
///< CStyle-Link inc,func and var begin
#include "BUPTLib_port.h"
#include "../SimpleLib/utils/utils.h"
#include "../SimpleLib/utils/vec.h"
#include "MotorLib/dji_boardv2_can.h"

//#include "point.h"

#ifdef __cplusplus
}
#endif
///< CStyle-Link func and var end


OmniChassis::OmniChassis() {

}
OmniChassis::~OmniChassis()
{

}
void OmniChassis::ChassisMove(float target_speed, float target_dir, float target_omega)
{
    __LIMIT(target_speed,DRIVE_WHEEL_MAX_SPEED);

    float absolute_angle_offset = 0; // 使用绝对坐标时，根据全场定位测得的偏航角进行补偿
    if (pos_mode == BaseChassis::PosModeAbsolute)
    {
        absolute_angle_offset = postureStatus.yaw - 1.5708f; // 车头初始方向90°
    }
    target_dir -= absolute_angle_offset;

    float driveWheelSpeed[4];

    for(uint8_t i = 0 ; i < 4 ; i++)
    {
        ///<将目标速度向量投影到每个驱动轮上
        driveWheelSpeed[i] = target_speed * cos( InstallAngle[i] - target_dir );
        ///< 与角速度在每个轮上的线速度合成
        driveWheelSpeed[i] += target_omega * WHEEL2CHASSISCENTER;

        ///< 使用大疆3508
        driveWheelSpeed[i]=(int16_t)(driveWheelSpeed[i] / DRIVE_WHEEL_RADIUS * 30.0f / PI * 2.7); //m/s转为rpm

        dji_VelCtrl(OMNI_WHEEL_DJIV2_BOARDID,i+1,(int32_t)driveWheelSpeed[i]);

        // 设置速度模式
        // MotorRpmCANMsg[i].in[0] = 1;
        // MotorRpmCANMsg[i].in[1] = (int)(motor_speed[i] / DRIVE_WHEEL_RADIUS * 30.0f / PI); //m/s转为rpm
        // CAN_SendMsg(OmniChassis.drive_motors[i].can_id, &MotorRpmCANMsg[i]);

        // 使用自制电机
        // OmniChassis.drive_motors[i].target_rpm=(float)(motor_speed[i] / DRIVE_WHEEL_RADIUS * 30.0f / PI); //m/s转为rpm
        // DrivePro_SetRPM(OmniChassis.drive_motors[i]);

        Delay_msec(1);
    }


}
void OmniChassis::ChassisInit()
{
    for (int i = 0; i < 4; i++)
    {
        // VESC_Init(&OmniChassis.drive_motors[i],i + OMNI_DRIVE_MOTOR_BASE_CANID,(VESCMode)VESC_RPM); // 若使用本杰明驱动器
//        MotorDriver_Init(&OmniChassis.drive_motors[i], i + OMNI_DRIVE_MOTOR_BASE_CANID, MTR_CTRL_RPM); // 若使用BUPT自制驱动卡
        ///< DJI
        dji_VelCfg(OMNI_WHEEL_DJIV2_BOARDID,i);
    }
}
