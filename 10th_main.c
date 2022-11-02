#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "lcd.h"
#include "touch.h"
#include "misc.h"


uint16_t value = 100;
int flag = 1;
int ledCount = 0;

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef       TIM_OCInitStructure;
uint16_t prescale;

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void NVIC_Configure(void);

//---------------------------------------------------------------------------------------------------

void RCC_Configure(void)
{
      /* LCD */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // port C RCC ENABLE
      /* PWM */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // TIM2
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // Port B
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // TIM3
      /* LED */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); // Port D
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

}

void GPIO_Configure(void)
{

    GPIO_InitTypeDef GPIO_InitStructure3;
    GPIO_InitStructure3.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure3.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure3); // led 활성화

   /* PWM */
    GPIO_InitTypeDef GPIO_InitStructure2;
    GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure2);

}

void NVIC_Configure(void) {
    
     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
     NVIC_InitTypeDef NVIC_InitStructure;
     /* Enable TIM2 Global Interrupt */
     NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);

}


void TIM2_IRQHandler(void) {
      if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
      {
          if (ledCount % 2 == 0) {
          GPIO_SetBits(GPIOD, GPIO_Pin_2);
          } else{
          GPIO_ResetBits(GPIOD, GPIO_Pin_2); 
          }

          if(ledCount == 0) {
          GPIO_ResetBits(GPIOD, GPIO_Pin_3);
          } else if(ledCount == 5) {
          GPIO_SetBits(GPIOD, GPIO_Pin_3);
          }

          ledCount++;
          ledCount %= 10;

          TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
      }
}

void PWM_Configure1()
{
     prescale = (uint16_t) (SystemCoreClock / 10000);

     TIM_TimeBaseStructure.TIM_Period = 10000;
     TIM_TimeBaseStructure.TIM_Prescaler = prescale;
     TIM_TimeBaseStructure.TIM_ClockDivision = 0;
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;

     TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
     TIM_ARRPreloadConfig(TIM2, ENABLE);
     TIM_Cmd(TIM2, ENABLE);
}

void PWM_Configure2()
{
     prescale = (uint16_t) (SystemCoreClock / 100000);

     TIM_TimeBaseStructure.TIM_Period = 20000;
     TIM_TimeBaseStructure.TIM_Prescaler = prescale;
     TIM_TimeBaseStructure.TIM_ClockDivision = 0;
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;

     TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
     TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
     TIM_OCInitStructure.TIM_Pulse = 1500; // us
     TIM_OC3Init(TIM3, &TIM_OCInitStructure);

     TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
     TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);
     TIM_ARRPreloadConfig(TIM3, ENABLE);
     TIM_Cmd(TIM3, ENABLE);
}

void delay()
{
     for(int j = 0; j < 5000000; j++) {
        continue;
     }
}


int main(void)
{

      SystemInit();
      RCC_Configure();
      GPIO_Configure();
      PWM_Configure1();
      PWM_Configure2();

      NVIC_Configure();

      //-----------------

      LCD_Init(); // LCD 초기화
      Touch_Configuration(); // 터치 설정
      Touch_Adjust(); // 화면 터치 초점 맞추기
      LCD_Clear(WHITE); // LCD 배경 초기화

      //-----------------
      uint16_t rotation[2] = {700, 2300};
      int i = 0;

      uint16_t x;
      uint16_t y;
      uint16_t convert_x;
      uint16_t convert_y;

       LCD_ShowString(40, 10, "WED_TEAM03", BLUE, WHITE);
       LCD_ShowString(90, 50, "OFF", RED, WHITE);
       LCD_DrawRectangle(40, 80, 80, 120);
       LCD_ShowString(50,90, "BUT", BLUE, WHITE);
      
      while (1) {
           Touch_GetXY(&x, &y, 1); // 터치 좌표 받아서 배열에 입력
           Convert_Pos(x, y,&convert_x, &convert_y); // 받은 좌표를 LCD 크기에 맞게 변환

           if ((uint16_t)40 < convert_x && convert_x < (uint16_t)80 && \
               (uint16_t)80 < convert_y && convert_y < (uint16_t)120) {
              if (flag) {
                 LCD_ShowString(90, 50, "    ", RED, WHITE); 
                 LCD_ShowString(40, 50, "ON", RED, WHITE); 
                 TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // interrupt enable
              } else {
                 LCD_ShowString(90, 50, "OFF", RED, WHITE); 
                 LCD_ShowString(40, 50, "    ", RED, WHITE); 
                 TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE); // interrupt enable
              }
              flag++;
              flag %= 2;
           }

           TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
           TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
           TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
           TIM_OCInitStructure.TIM_Pulse = rotation[i]; // us
           TIM_OC3Init(TIM3, &TIM_OCInitStructure);
            i++;
            i %= 2;
            delay();

      }
      return 0;
}
