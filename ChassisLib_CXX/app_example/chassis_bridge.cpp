/*!
 * @author LiNY2 
 * @date 2023/04/14 0014
 */


#include "chassis_bridge.h"
#include "chassis_ctrl.h"
#include <cstring>
#ifdef __cplusplus
extern "C"
{
#endif
//CStyle-Link inc,func and var begin
#include "BUPTLib_port.h"
#include "SimpleLib/utils/vec.h"
#include "SimpleLib/utils/utils.h"
#include "MotorLib/vesc_can.h"
#include "MotorLib/dji_boardv2_can.h"

#ifdef __cplusplus
}
#endif


static Rudder_ControllMsg_s ControllMsg;

static uint8_t mpc_flag;

void bridge_exe()
{
//    chassis.Chassis_SetPosMode(BaseChassis::Chassis_Pos_Mode(ControllMsg.pos_mode));
    Received_Control_Data_Dealer(OSLIB_UART_Handle_Get(&huart6));
    switch (Msg_Mode(ControllMsg.msg_mode))
    {
        case Mode_velSet:
            mpc_flag = 0;
            chassis.mpc_move(ControllMsg.target_spd,ControllMsg.target_pos);
            log_i("mpc0:speed:%.4f,dir:%.4f",ControllMsg.target_spd[0],ControllMsg.target_pos[0]);

//            chassis.Chassis_SetTargetVel(ControllMsg.target_speed,ControllMsg.target_dir,ControllMsg.target_omega);
//            chassis.BaseChassis::ChassisMove();
//            log_i("BridgeGot:speed:%.4f,dir:%.4f,omega:%.4f",ControllMsg.target_speed,ControllMsg.target_dir,ControllMsg.target_omega);
//          log_i("mpc0:speed:%.4f,dir:%.4f",ControllMsg.target_spd[0],ControllMsg.target_pos[0]);
//            log_i("mpc1:speed:%.4f,dir:%.4f",ControllMsg.target_spd[1],ControllMsg.target_pos[1]);
//            log_i("mpc2:speed:%.4f,dir:%.4f",ControllMsg.target_spd[2],ControllMsg.target_pos[2]);
//            log_i("mpc3:speed:%.4f,dir:%.4f",ControllMsg.target_spd[3],ControllMsg.target_pos[3]);
            break;
        case Mode_PosCtrl:
            dji_PosCtrl(ControllMsg.boardId,ControllMsg.motorId,ControllMsg.pos);
            break;
        case Mode_VelCtrl:
            dji_VelCtrl(ControllMsg.boardId,ControllMsg.motorId,ControllMsg.vel);
            break;
        case CurCfg:
            break;
        case MPC_Move:

            break;
    }
    Bridge_PosUpdate();
}
void Bridge_PosUpdate()
{
    const uint8_t data_to_send_size = 66;
    static uint8_t data_to_send[data_to_send_size+2];
    static FeedBackMsg_s feedback;
    static uint8_t cnt = 0;
    data_to_send[0] = 0xAA;
    data_to_send[1] = 0x55;
    feedback.chassis_pos = chassis.Chassis_GetPostureStatus();
    memcpy(data_to_send+2,feedback.data,64);
    OSLIB_UART_SendData(&huart5,data_to_send,data_to_send_size*sizeof(uint8_t));
    if (cnt == 10)
    {
        HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
        cnt = 0;
    }
    cnt++;
}

void Received_Control_Data_Dealer(OSLIB_UART_Handle_t *uart_handle)
{
    union USART_Receive
    {
        uint8_t data[84];
        Rudder_ControllMsg_s ctrlmsg;
    } RX_Data_u{};
    if(uart_handle->rx.dma.rx_task_buffer[0]==0xaa && uart_handle->rx.dma.rx_task_buffer[1]==0x55)
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);

        memcpy(RX_Data_u.data,uart_handle->rx.dma.rx_task_buffer+2,84*sizeof(uint8_t));
        //		for (int i = 0; i <84 ; i++)
        //		{
        //			RX_Data_u.data[i] = uart_handle->rx.dma.rx_buffer[i+2];
        //			// if(i>3 && i<15) OSLIB_UART_Printf(&huart_major,"RX_Data_u.data:%d,value:%x\r\n",i,RX_Data_u.data[i]);
        //		}
        ControllMsg = RX_Data_u.ctrlmsg;
        // OSLIB_UART_Printf(&huart_major,"controlMsg-spd:%f,%f,dir:%f,%f,omega:%f,%f\r\n",\
		// ControllMsg.target_speed,RX_Data_u.temp.target_speed,\
		// ControllMsg.target_dir,RX_Data_u.temp.target_dir,\
		// ControllMsg.target_omega,RX_Data_u.temp.target_omega);
    }
    else{
        HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
    }
}