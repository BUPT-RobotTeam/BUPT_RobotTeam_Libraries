/*!
 * @author LiNY2 
 * @date 2023/04/07 0007
 * @note 文件应当被纯c语言包含，用于连接串口命令
 */


#ifndef CHASSIS_CMD_H
#define CHASSIS_CMD_H
#ifdef __cplusplus
extern "C"
{
#endif
    void chassis_cmd_reg(UART_HandleTypeDef * uart);

#ifdef __cplusplus
}
#endif
#endif//CHASSIS_CMD_H
