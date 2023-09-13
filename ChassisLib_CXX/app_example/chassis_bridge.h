/*!
 * @author LiNY2 
 * @date 2023/04/14 0014
 */


#ifndef CHASSIS_BRIDGE_H
#define CHASSIS_BRIDGE_H

#include "../base_chassis.h"
#include "../rudder_chassis.h"
#ifdef __cplusplus
extern "C"
{
#endif
//CStyle-Link inc,func and var begin
#include "BUPTLib_port.h"
#include "../../SimpleLib/utils/vec.h"
#include "../../SimpleLib/utils/utils.h"
#ifdef __cplusplus
}
#endif

typedef enum Msg_Mode
{
    ///底盘运动
    Mode_velSet = 0,      //对应仅传入target_speed,target_dir,target_omega
    ///单个电机位置（大疆）
    Mode_PosCtrl,
    ///<单个电机速度
    Mode_VelCtrl,
    ///电流控制模式
    CurCfg,
    ///MPC直控舵轮驱动
    MPC_Move

}Msg_Mode;

typedef enum Current_Mode
{
    //本杰明电调控制电机电流
    VEC = 0,
    //dji控制电机电流
    DJI,
}Current_Mode;

typedef struct Current_ControllMsg_s
{
    //信息类型--对应Current_Mode
    uint8_t current_mode;
    float target_current;
    //dji控制所需
    int boardId;
    int motorId;
    //vec控制所需
    int vecId;

} Current_ControllMsg_s;


typedef struct Rudder_ControllMsg_s
{
    //信息类型--对应Msg_Mode
    uint8_t msg_mode;
    uint8_t pos_mode;

    //用于Only_Move
    float target_speed;
    float target_dir;
    float target_omega;
    //用于控制单个电机
    int boardId;
    int motorId;
    int pos;
    int vel;
    //MPC直控舵轮信息
    float target_pos[4];
    float target_spd[4];
    Current_ControllMsg_s CurrentMsg;


} Rudder_ControllMsg_s;



typedef union FeedBackMsg_s
{
    uint8_t data[64];
    PostureStatus_s chassis_pos;
}FeedBackMsg_s;

void bridge_exe();
void Bridge_PosUpdate();
void Received_Control_Data_Dealer(OSLIB_UART_Handle_t *uart_handle);

#endif//CHASSIS_BRIDGE_H
