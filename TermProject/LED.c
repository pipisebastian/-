#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

// PE2, PE3, PE4를 이용함

void LED_RCC_Configure(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
}

void LED_GPIO_Configure(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  // LED Output Mode
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // output mode로
  GPIO_InitStructure.GPIO_Pin=(GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);// PD2,3,4
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void LED_Init(void) {
  LED_RCC_Configure();
  LED_GPIO_Configure();
}

int main(void)
{
		SystemInit();
    LED_Init();
		 
		//흰색 키기
		GPIO_SetBits(GPIOE, GPIO_Pin_2); //파랑색 on
		GPIO_SetBits(GPIOE, GPIO_Pin_3); //초록색 on
		GPIO_SetBits(GPIOE, GPIO_Pin_4); // 빨간색 on
    return 0;
}
