#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "stm32f10x_tim.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

void RCC_Configure(void)
{
    /* BUZZER */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // PB0
}

void GPIO_Configure(void)
{
    /* BUZZER */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void delay(void) {
	  int i;
	  for (i = 0; i < 1000000; i++) {}
}

int main(void)
{
    SystemInit();
    RCC_Configure();
    GPIO_Configure();

		GPIO_SetBits(GPIOB, GPIO_Pin_0);
		delay();
		GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}
