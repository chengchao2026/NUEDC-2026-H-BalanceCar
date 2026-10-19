#ifndef DEBUG UART_H
#define DEBUG UART_H

#include "main.h"

// 调试UART函数
void Debug_ProcessCommand(uint8_t* cmd, uint16_t len);
void Debug_Uart_Process(void);
void Debug_SendData(void);

#endif