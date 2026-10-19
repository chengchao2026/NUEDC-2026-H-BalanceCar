

#include "Tmc2209.h"
#include <stdio.h>
// #include "Oled.h"   // 暂无 OLED，注释屏蔽
#include "delay.h"
#include <string.h>
#include "main.h"

extern UART_HandleTypeDef huart1;

/*
 *
 * 这份工程当前实际只写 4 个核心寄存器：
 * 1. GCONF       (0x00)  全局功能开关
 * 2. IHOLD_IRUN  (0x10)  保持/运行电流
 * 3. VACTUAL     (0x22)  内部速度模式，这里固定为 0，不使用
 * 4. CHOPCONF    (0x6C)  细分/斩波核心配置
 */
static const uint32_t TMC2209_GCONF_VALUE = 0x00000180;
static const uint32_t TMC2209_CHOPCONF_VALUE = 0x12000053;
static const uint32_t TMC2209_IHOLD_IRUN_VALUE = 0x00060C04;
static const uint32_t TMC2209_VACTUAL_VALUE = 0x00000000;

void tmc_calc_crc(uint8_t *dat, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < (len - 1); i++)
    {
        uint8_t currentByte = dat[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (currentByte & 0x01))
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc = (crc << 1);
            }
            currentByte >>= 1;
        }
    }
    dat[len - 1] = crc;
}

void TMC2209_WriteReg(uint8_t address, uint8_t reg, uint32_t val)
{
    uint8_t txBuffer[8];
    txBuffer[0] = 0x05;
    txBuffer[1] = address;
    txBuffer[2] = reg | 0x80;
    txBuffer[3] = (val >> 24) & 0xFF;
    txBuffer[4] = (val >> 16) & 0xFF;
    txBuffer[5] = (val >> 8) & 0xFF;
    txBuffer[6] = val & 0xFF;
    tmc_calc_crc(txBuffer, 8);

    HAL_UART_Transmit(&huart1, txBuffer, 8, 10);
    uint8_t trash[8];
    HAL_UART_Receive(&huart1, trash, 8, 5);
}

uint8_t TMC2209_ReadReg_Robust(uint8_t tmc_addr, uint8_t reg, uint32_t *val_out)
{
    uint8_t txBuffer[4];
    uint8_t rxBuffer[24];

    txBuffer[0] = 0x05;
    txBuffer[1] = tmc_addr;
    txBuffer[2] = reg;
    tmc_calc_crc(txBuffer, 4);

    __HAL_UART_FLUSH_DRREGISTER(&huart1);
    memset(rxBuffer, 0, sizeof(rxBuffer));

    HAL_UART_Transmit(&huart1, txBuffer, 4, 10);

    HAL_UART_Receive(&huart1, rxBuffer, 20, 50);

    // 滑动窗口搜索特征头 (0x05 0xFF)
    for (uint8_t i = 0; i < 12; i++)
    {
        // 特征：当前是 0x05，下一个是 0xFF (主机地址)
        if (rxBuffer[i] == 0x05 && rxBuffer[i + 1] == 0xFF)
        {
            if (rxBuffer[i + 2] != reg)
            {
                continue;
            }

            // 5.2 提取这 8 个字节进行 CRC 校验
            uint8_t potential_packet[8];
            memcpy(potential_packet, &rxBuffer[i], 8);

            uint8_t received_crc = potential_packet[7];
            tmc_calc_crc(potential_packet, 8);

            if (received_crc == potential_packet[7])
            {
                *val_out = 0;
                *val_out |= (uint32_t)potential_packet[3] << 24;
                *val_out |= (uint32_t)potential_packet[4] << 16;
                *val_out |= (uint32_t)potential_packet[5] << 8;
                *val_out |= (uint32_t)potential_packet[6];
                return 1; // 成功
            }
        }
    }

    return 0;
}

void Tmc2209_Init(uint8_t tmc_addr)
{

    // OLED_String(0, 0, "Tmc", 16);    // 暂无 OLED，注释屏蔽
    // OLED_Num(25, 0, tmc_addr, 1, 16);
    delay_ms(100);
    TMC2209_WriteReg(tmc_addr, TMC2209_GCONF, TMC2209_GCONF_VALUE);
    TMC2209_WriteReg(tmc_addr, TMC2209_CHOPCONF, TMC2209_CHOPCONF_VALUE);
    TMC2209_WriteReg(tmc_addr, TMC2209_IHOLD_IRUN, TMC2209_IHOLD_IRUN_VALUE);
    TMC2209_WriteReg(tmc_addr, TMC2209_VACTUAL, TMC2209_VACTUAL_VALUE);

    delay_ms(50);

    uint32_t read_val = 0;
    uint8_t retry = 0;
    uint8_t read_ok = 0;

    const uint32_t expected_gconf_mask = 0x00000180;
    while (retry < 10)
    {
        if (TMC2209_ReadReg_Robust(tmc_addr, TMC2209_GCONF, &read_val))
        {
            // OLED_String(0, 17, "read sucess", 12);  // 暂无 OLED
            if ((read_val & expected_gconf_mask) == expected_gconf_mask)
            {
                read_ok = 1;
                break;
            }
        }
        else
        {
            // OLED_String(0, 17, "read false  ", 12);  // 暂无 OLED
        }
        // OLED_String(0, 30, "Attempts:", 12);         // 暂无 OLED
        // OLED_Num(60, 30, retry + 1, 4, 12);
        retry++;
        delay_ms(200);
    }
    if (!read_ok)
    {
        // OLED_String(0, 17, "GCONF warn ", 12);       // 暂无 OLED
        delay_ms(1000);
    }
    // OLED_String(41, 0, " Init OK  ", 16);            // 暂无 OLED
    // OLED_Clear();
}
