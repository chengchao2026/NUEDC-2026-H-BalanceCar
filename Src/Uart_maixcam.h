#ifndef VISION_UART_H
#define VISION_UART_H

#include "stm32f4xx_hal.h"

/* MaixCam Pro -> STM32 UART5 frame: $B,<valid>,<x>\n (pixel protocol, run_maixcam2.py)
   MaixCam H_demo -> STM32 frame: $C,<valid>,<pos_cm_x100>,<vel_cms_x100>\n
   (cm+velocity protocol; values are real_value*100 fixed-point so the MCU side
   never needs float parsing). Both frame types are parsed independently and
   can coexist in the same firmware image. */
void VisionUart_Init(void);
void VisionUart_Process(void);
uint8_t VisionUart_ConsumeFrame(uint8_t *valid, int16_t *x);
uint8_t VisionUart_ConsumeCmFrame(uint8_t *valid, int32_t *pos_cm_x100, int32_t *vel_cms_x100);
/* 统一输出经过一阶低通后的厘米坐标；$B 像素帧和 $C 厘米帧均可使用。 */
uint8_t VisionUart_ConsumeFilteredPosition(uint8_t *valid,
                                           float *pos_cm,
                                           float *vel_cms);

uint32_t VisionUart_GetRxByteCount(void);
uint8_t VisionUart_GetFrameReadyFlag(void);
uint8_t VisionUart_GetParserState(void);

#endif
