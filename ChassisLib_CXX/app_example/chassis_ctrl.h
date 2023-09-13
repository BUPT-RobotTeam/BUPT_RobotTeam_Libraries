/*!
 * @author LiNY2 
 * @date 2023/04/07 0007
 */


#ifndef CHASSIS_APP_H
#define CHASSIS_APP_H

/**
 * Log default configuration for EasyLogger.
 * NOTE: Must defined before including the <elog.h>
 */
#if !defined(LOG_TAG)
#define LOG_TAG                    "ChassisCtrl"
#endif
#undef LOG_LVL
#define  CHASSIS_LOG_LVL ELOG_LVL_INFO
#if defined(CHASSIS_LOG_LVL)
#define LOG_LVL                    CHASSIS_LOG_LVL
#endif

#include "ChassisLib_CXX/base_chassis.h"
#include "ChassisLib_CXX/rudder_chassis.h"

extern RudderChassis chassis;
void ctrl_exe();
void ctrl_mode_transfer(CHASSIS_CTRL_MODE target_mode);

#endif//CHASSIS_APP_H
