/*!
 * @author LiNY2 
 * @date 2023/03/25 0025
 * @brief 便于底盘库和电机库在simplelib和oslib间切换。添加了一些兼容性的定义。
 */


#ifndef BUPTLIB_PORT_H
#define BUPTLIB_PORT_H

///<这里的宏定义可以写在CMakeList里面，也可以在此定义
//#define USE_SIMPLELIB
//#define USE_OSLIB

#if defined(USE_SIMPLElIB)
#include "../SimpleLib/utils/utils.h"
#include "simplelib.h"
#define Delay_msec(msec) HAL_Delay(msec)

#elif defined(USE_OSLIB)
#include "oslib.h"
///< 兼容性定义
typedef CAN_Message CAN_Message_u;
typedef CAN_ConnMessage CAN_ConnMessage_s;
///< 以下宏定义只被vesc_can.h使用，注意在此处切换需要的hcan,类型是CAN_HandleTypeDef *
#define VESC_CAN_HANDLEP (&hcan1)
#define CAN_SendExtMsg(ext_id,msg) OSLIB_CAN_SendMessage(VESC_CAN_HANDLEP,CAN_ID_EXT,ext_id,msg)
#define CAN_SendStdMsg(id,msg) OSLIB_CAN_SendMessage(VESC_CAN_HANDLEP,CAN_ID_STD,id,msg)
#define Delay_msec(msec) osDelay(msec)
#endif


#endif//BUPTLIB_PORT_H
