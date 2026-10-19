#ifndef LASER_H
#define LASER_H

#include "main.h"

#define   Laser_On     HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_SET)
#define   Laser_Off    HAL_GPIO_WritePin(Laser_GPIO_Port,Laser_Pin,GPIO_PIN_RESET)

void Laser_Init(void);

#endif

