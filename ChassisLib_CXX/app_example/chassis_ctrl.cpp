/*!
 * @author LiNY2 
 * @date 2023/04/07 0007
 */


#include "chassis_ctrl.h"
#include "chassis_bridge.h"
#include "ChassisLib_CXX/ctrlGo2Point.h"
#include "ChassisLib_CXX/ctrlMacro.h"
#include "ChassisLib_CXX/ctrlPathTracker.h"
#include "chassis_cmd.h"
#ifdef __cplusplus
extern "C"
{
#endif
//CStyle-Link inc,func and var begin
#include "BUPTLib_port.h"
#include "oslib_uart_cmd.h"
//CStyle-Link func and var end
#ifdef __cplusplus
}
#endif


RudderChassis chassis;
Go2Point ctrl_Go2Point{&chassis};
YawTurning ctrl_YawTurn{&chassis};
PathTracker ctrl_Track{&chassis};

CHASSIS_CTRL_MODE chassis_ctrl_mode = CTRL_MODE_CMD;
void ctrl_exe()
{
    chassis.Chassis_UpdatePostureStatus();
    switch (chassis_ctrl_mode)
    {
        case CTRL_MODE_NONE:{
            chassis.Chassis_SetTargetVel(0,0,0,false,false,false);
            chassis.BaseChassis::ChassisMove();
            break ;
        }
        case CTRL_MODE_CMD:
        {
            Bridge_PosUpdate();

            chassis.BaseChassis::ChassisMove();
            break;
        }
        case CTRL_MODE_HANDLE:
            //Received_Control_Data_Dealer(OSLIB_UART_Handle_Get(&huart3));
            Bridge_PosUpdate();
            break;
        case CTRL_MODE_TRACK:
            ctrl_Track.Ctrl_exe();
            chassis.BaseChassis::ChassisMove();
            break;
        case CTRL_MODE_GO_TO_POINT:
            ctrl_Go2Point.Ctrl_exe();
            chassis.BaseChassis::ChassisMove();
            break;
        case CTRL_MODE_TUNING:
            ctrl_YawTurn.Ctrl_exe();
            chassis.BaseChassis::ChassisMove();
            break;
        case CTRL_MODE_EXTERNAL:
            bridge_exe();
//            chassis.BaseChassis::ChassisMove();
            break;
        default:
            break;
    }
}
void ctrl_mode_transfer(CHASSIS_CTRL_MODE target_mode)
{
    ///< 当前状态复位
    switch (chassis_ctrl_mode)
    {
        case CTRL_MODE_NONE:
            break;
        case CTRL_MODE_HANDLE:
            chassis.Chassis_Stop();
            break;
        case CTRL_MODE_CMD:
            chassis.Chassis_Stop();
            break;
        case CTRL_MODE_TRACK:
            ctrl_Track.ResetStatus();
            break;
        case CTRL_MODE_GO_TO_POINT:
            ctrl_Go2Point.ResetStatus();
            break;
        case CTRL_MODE_TUNING:
            ctrl_YawTurn.ResetStatus();
            break;
        case CTRL_MODE_EXTERNAL:
            break;
    }

    ///< 转移到目标状态
    chassis_ctrl_mode = target_mode;

}
void command_chassis_SetCtrlMode(OSLIB_UART_Handle_t *uart_handle,int argc, char *argv[])
{
    uint8_t in  = atoi(argv[1]);
    if(in > 6)
    {
        log_e("Param Error!\r\n");
        return;
    }
    ctrl_mode_transfer(CHASSIS_CTRL_MODE(in));
}

void command_chassis_SetCMDVel(OSLIB_UART_Handle_t *uart_handle,int argc, char *argv[])
{
    if (argc < 3) {
        log_e("Param Error!\r\n");
        return;
    }
    float speed = atof(argv[1]);
    float dir = atof(argv[2]);
    float omega = atof(argv[3]);
    log_i("ctrl cmd:\r\nspeed:%.3f,dir:%.3f,omega:%.3f\r\n",speed,dir,omega);
    chassis.Chassis_SetTargetVel(speed,dir,omega);

}

void command_Chassis_Stop(OSLIB_UART_Handle_t *uart_handle,int argc, char *argv[])
{
    ctrl_mode_transfer(CTRL_MODE_NONE);
    chassis.Chassis_Stop();
}

void command_Chassis_getPos(OSLIB_UART_Handle_t *uart_handle,int argc, char *argv[])
{
    auto nowpos = chassis.Chassis_GetPostureStatus();
    log_i("car pos:\r\nx:%2.4f,y:%2.4f,yaw:%.4f\r\n",nowpos.x,nowpos.y,nowpos.yaw);

}
void command_Chassis_setPrintPosFlag(OSLIB_UART_Handle_t *uart_handle,int argc, char *argv[])
{
    chassis.Chassis_PrintPostureStatus_Flag = atoi(argv[1]);
}

void command_YawTurning_SetYaw(OSLIB_UART_Handle_t *uart_handle,int argc, char *argv[])
{
//    BaseChassis.ctrl_mode = CTRL_MODE_TUNING;
//    CMD_TargetYaw= __ANGLE2RAD( atof(argv[1]));
    ctrl_YawTurn.SetTarget(atof(argv[1]),true);
    ctrl_mode_transfer(CTRL_MODE_TUNING);
}

void command_YawTurning_setYawPid(OSLIB_UART_Handle_t *uart_handle,int argc, char *argv[])
{
    ctrl_YawTurn.SetyawPID(atof(argv[1]),atof(argv[2]),atof(argv[3]));

}
void command_Go2Point_setYawPid(OSLIB_UART_Handle_t *uart_handle,int argc, char *argv[])
{
    ctrl_YawTurn.SetyawPID(atof(argv[1]),atof(argv[2]),atof(argv[3]));

}
void command_Go2Point_setLockPid(OSLIB_UART_Handle_t *uart_handle,int argc, char *argv[])
{
    ctrl_Go2Point.SetlockPID(atof(argv[1]),atof(argv[2]),atof(argv[3]));
//    LockPID.Kp = atof(argv[1]);
//    LockPID.Ki = atof(argv[2]);
//    LockPID.Kd = atof(argv[3]);
}

void command_Go2Point_setPt(OSLIB_UART_Handle_t *uart_handle,int argc, char *argv[])
{
    if (argc == 2 && atoi(argv[1])==-1)
    {
        auto nowpos = chassis.Chassis_GetPostureStatus();
        ctrl_Go2Point.SetTarget({nowpos.x,nowpos.y},nowpos.yaw,0.3,0.2);
        log_i("go2point: stand\r\n");
    }
    else
    {
        float tar_x = atoff(argv[1]);
        float tar_y = atoff(argv[2]);
        float tar_yaw = __ANGLE2RAD(atoff(argv[3]));
        ctrl_Go2Point.SetTarget({tar_x,tar_y},tar_yaw,0.3,0.2);
//        BaseChassis.Go2PointStatus.target_point.x = atof(argv[1]);
//        BaseChassis.Go2PointStatus.target_point.y = atof(argv[2]);
//        BaseChassis.Go2PointStatus.target_yaw = __ANGLE2RAD(atof(argv[3]));
        log_i("go2point:x:%.4f,y:%.4f,yaw:%.3f\r\n",tar_x,tar_y,tar_yaw);

    }
//    BaseChassis.Go2PointStatus.start_speed = 0.12;
//    BaseChassis.Go2PointStatus.final_speed = 0;
//    BaseChassis.Go2PointStatus.enable = 1;
//    BaseChassis.Go2PointStatus.start = 1;
    ctrl_mode_transfer( CTRL_MODE_GO_TO_POINT);
}

void chassis_cmd_reg(UART_HandleTypeDef * uart)
{
    OSLIB_UART_Handle_t * uart_handle = OSLIB_UART_Handle_Get(uart);
    OSLIB_UART_CLI_AddCommand(uart_handle,
                              "csetmode","usage: csetmode mode;"
                                          "mode: 0:None 1:Handle 2:CMD 3:TRACK 4:GO2PT 5:Turn 6:EXT",command_chassis_SetCtrlMode);
    OSLIB_UART_CLI_AddCommand(uart_handle,
                              "csetvel","usage: csetvel speed dir omega",command_chassis_SetCMDVel);
    OSLIB_UART_CLI_AddCommand(uart_handle,
                              "cstop","usage: cstop",command_Chassis_Stop);
    OSLIB_UART_CLI_AddCommand(uart_handle,
                              "cgetpos","usage: cgetpos ",command_Chassis_getPos);
    OSLIB_UART_CLI_AddCommand(uart_handle,
                              "csetposout","enable pos print;usage: cgetpos 0/1",command_Chassis_setPrintPosFlag);
    OSLIB_UART_CLI_AddCommand(uart_handle,
                              "ysettar","set target yaw and start turn;usage: ysetyaw angle(deg)",command_YawTurning_SetYaw);
    OSLIB_UART_CLI_AddCommand(uart_handle,
                              "ysetyawpid","set yawPID;usage: ysetyawpid kp ki kd",command_YawTurning_setYawPid);
    OSLIB_UART_CLI_AddCommand(uart_handle,
                              "gsettar","set go2pt target;usage: gsettar (-1) x y yaw",command_Go2Point_setPt);
    OSLIB_UART_CLI_AddCommand(uart_handle,
                              "gsetlockpid","set lockPID;usage: gsetlockpid kp ki kd",command_Go2Point_setLockPid);
    OSLIB_UART_CLI_AddCommand(uart_handle,
                              "gsetyawpid","set yawPID;usage: ysetyawpid kp ki kd",command_Go2Point_setYawPid);


}
