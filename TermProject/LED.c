#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

void LED_RCC_Configure(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
}

void LED_GPIO_Configure(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  // LED Output Mode
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // output mode로
  GPIO_InitStructure.GPIO_Pin=(GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4);// PD2,3,4
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void LED_Init(void) {
  LED_RCC_Configure();
  LED_GPIO_Configure();
}

//안써욤
boolean LeftLight = false;
boolean RightLight = false;

void LeftLight_OnOff(){//main에서 해당 함수 호출시
}

void RightLight_OnOff(){
}

void LightOnWhenDark(){
}
//안써욤

int main(void)
{
		SystemInit();
    LED_Init();
		 
		//흰색 키기
		GPIO_SetBits(GPIOD, GPIO_Pin_2); //파랑색 on
		GPIO_SetBits(GPIOD, GPIO_Pin_3); //초록색 on
		GPIO_SetBits(GPIOD, GPIO_Pin_4); // 빨간색 on
    return 0;
}
