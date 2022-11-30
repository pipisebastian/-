
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "touch.h"
#include "misc.h"

/* function prototype */
void LED_RCC_Configure(void);
void LED_GPIO_Configure(void);
void LED_Init(void);

// PE2, PE3, PE4를 이용함

void LED_RCC_Configure(void) {
/* Alternate Function IO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
}

void LED_GPIO_Configure(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  // LED Output Mode
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void LED_Init(void) {
  LED_RCC_Configure();
  LED_GPIO_Configure();
}

int main(void)
{
  SystemInit();

  LED_RCC_Configure();
  LED_GPIO_Configure();
		 
  //흰색 키기
  GPIO_SetBits(GPIOE, GPIO_Pin_0); //파랑색 on
  GPIO_SetBits(GPIOE, GPIO_Pin_1); //초록색 on
  GPIO_SetBits(GPIOE, GPIO_Pin_2); // 빨간색 on
  return 0;
}
