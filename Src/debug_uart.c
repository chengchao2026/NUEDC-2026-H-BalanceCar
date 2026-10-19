#include "debug_uart.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

/* 视觉接入后恢复（依赖 yaw_pid / pitch_pid）
void Debug_ProcessCommand(uint8_t* cmd, uint16_t len)
{
    if(len < 3) return;

    char cmd_type = cmd[0];
    char axis = cmd[1];
    float value = 0;

    if(sscanf((char*)&cmd[2], "%f", &value) != 1) return;

    switch(cmd_type){
        case 'K':
            if(len < 4) return;
            char param = cmd[2];
            if(axis == 'X'){
                switch(param){
                    case 'P': yaw_pid.kp = value; break;
                    case 'I': yaw_pid.ki = value; break;
                    case 'D': yaw_pid.kd = value; break;
                }
            } else if(axis == 'Y'){
                switch(param){
                    case 'P': pitch_pid.kp = value; break;
                    case 'I': pitch_pid.ki = value; break;
                    case 'D': pitch_pid.kd = value; break;
                }
            }
            break;

        case 'M':
            if(axis == 'X'){
                yaw_pid.OUT_MAX = value;
            } else if(axis == 'Y'){
                pitch_pid.OUT_MAX = value;
            }
            break;

        case 'I':
            if(axis == 'X'){
                yaw_pid.KI_MAX = value;
            } else if(axis == 'Y'){
                pitch_pid.KI_MAX = value;
            }
            break;

        case 'R':
            yaw_pid.Errorint = 0;
            pitch_pid.Errorint = 0;
            break;
    }

    char ack[32];
    int ack_len = snprintf(ack, sizeof(ack), "OK\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)ack, ack_len, 10);
}
*/

#define CMD_MAX_LEN 32
uint8_t rx_buffer[CMD_MAX_LEN];
uint16_t rx_index = 0;

void Debug_Uart_Receive(void)
{
    uint8_t byte;
    if(HAL_UART_Receive(&huart1, &byte, 1, 0) == HAL_OK){
        if(byte == '\r' || byte == '\n'){
            if(rx_index > 0){
                // Debug_ProcessCommand(rx_buffer, rx_index);  // 视觉接入后恢复
                rx_index = 0;
            }
        } else if(rx_index < CMD_MAX_LEN - 1){
            rx_buffer[rx_index++] = byte;
        }
    }
}

void Debug_Uart_Process(void)
{
    static uint32_t last_process_time = 0;
    if(HAL_GetTick() - last_process_time > 10){
        last_process_time = HAL_GetTick();
        Debug_Uart_Receive();
    }
}