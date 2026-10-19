#include "Buzzer.h"
#include "delay.h"

void Buzzer_Init(void){
		Buzzer_Off;
}

void Buzzer_On_1s(void){
		Buzzer_On;
		delay_ms(1000);
		Buzzer_Off;
}

