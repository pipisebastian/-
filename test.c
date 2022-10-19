#include <stdio.h>
#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "font.h"
#include "lcd.h"
#include "lcd.c"
#include "touch.h"
#include "touch.c"

int color[12] = {WHITE, CYAN, BLUE, RED, MAGENTA, LGRAY, GREEN, YELLOW, BROWN, BRRED, GRAY};
uint16_t value = 100;

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void NVIC_Configure(void);

//---------------------------------------------------------------------------------------------------

void RCC_Configure(void)
{
   /* Alternate Function IO clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE); // ADC1, port C RCC ENABLE
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
   // 나머지 LCD에 필요한 포트들은 lcd.c에서 활성화 된다.
}

void GPIO_Configure(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // 아날로그 입력 설정
   GPIO_Init(GPIOC, &GPIO_InitStructure);        // C0포트 활성화, 조도센서를 PC0에 연결할 계획
}

void ADC_Configure(void)
{
   // Analog to Digital Converter
   ADC_InitTypeDef ADC_InitStruct;

   // ADC_StructInit(&ADC_InitStruct);
   ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;                  // slave-master가 없는 독립 ADC
   ADC_InitStruct.ADC_ScanConvMode = DISABLE;                       // 단일채널이므로 비활성화
   ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;                  // 한 번의 트리거로 한 채널의 샘플링 시행
   ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 외부 입력핀에 의한 트리거 비활성화
   ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;              // Default
   ADC_InitStruct.ADC_NbrOfChannel = 1;                             // 채널은 하나

   ADC_Init(ADC1, &ADC_InitStruct); // 위 설정을 ADC1에 적용

   ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_28Cycles5); // 채널 우선순위 설정, 10채널 단독사용

   // ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE); // 무분별한 핸들러 호출 방지를 위해 주석처리

   ADC_Cmd(ADC1, ENABLE); // ADC1 활성화

   // Calibration reset & start
   ADC_ResetCalibration(ADC1);

   while (ADC_GetResetCalibrationStatus(ADC1))
      ;

   ADC_StartCalibration(ADC1);

   while (ADC_GetCalibrationStatus(ADC1))
      ;

   ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void NVIC_Configure(void)
{

   NVIC_InitTypeDef NVIC_InitStructure;

   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

   NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn; // ADC IRQ 인터럽트 활성화
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
}

void ADC1_2_IRQHandler(void)
{
   // printf("Handle\n");
   if (ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
   {                                        // End Of Conversion, ADC변환이 끝났을때,
      value = ADC_GetConversionValue(ADC1); // value에 조도센서 값 입력
      ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
   }
}

int main(void)
{

   SystemInit();
   RCC_Configure();
   GPIO_Configure();
   ADC_Configure();
   NVIC_Configure();

   //-----------------

   LCD_Init();            // LCD 초기화
   Touch_Configuration(); // 터치 설정
   Touch_Adjust();        // 화면 터치 초점 맞추기
   LCD_Clear(WHITE);      // LCD 배경 초기화

   //-----------------

   uint16_t pos_temp[2]; // x, y좌표를 담을 배열

   while (1)
   {
      LCD_ShowString(40, 40, "MON_Team02", MAGENTA, WHITE); // 팀명 출력

      Touch_GetXY(&pos_temp[0], &pos_temp[1], 1);                        // 터치 좌표 받아서 배열에 입력
      Convert_Pos(pos_temp[0], pos_temp[1], &pos_temp[0], &pos_temp[1]); // 받은 좌표를 LCD 크기에 맞게 변환
      Draw_Big_Point(pos_temp[0], pos_temp[1]);                          // 받은 좌표에 큰 원을 출력한다. 이때 touch.c의 Draw_Big_Point함수를 수정하여 이용했다.

      LCD_ShowNum(40, 60, (u32)pos_temp[0], 3, BLUE, WHITE); // x좌표 출력
      LCD_ShowNum(40, 80, (u32)pos_temp[1], 3, BLUE, WHITE); // y좌표 출력

      ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // ADC 변환이 끝났을때 인터럽트 발생
      LCD_ShowNum(40, 100, value, 4, BLUE, WHITE);
      ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE); // 무분별한 인터럽트 방지를 위해 비활성화
   }
   return 0;
}
