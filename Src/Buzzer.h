#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"

#define   Buzzer_On     HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_RESET)
#define   Buzzer_Off    HAL_GPIO_WritePin(Buzzer_GPIO_Port,Buzzer_Pin,GPIO_PIN_SET)

void Buzzer_Init(void);

void Buzzer_On_1s(void);

#endif

