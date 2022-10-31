#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"


/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void NVIC_Configure(void);
void ADC_Configure(void);
void Delay(void);


uint16_t ADC1_CONVERTED_VALUE;

void RCC_Configure(void)
{
    // TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'
    
    /* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    /* ADC */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

void ADC_Configure(void) {
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // TODO: Initialize the GPIO pins using the structure 'GPIO_InitTypeDef' and the function 'GPIO_Init'
    
    // ADC
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}


void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    
    // TODO: fill the arg you want
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    // TODO: Initialize the NVIC using the structure 'NVIC_InitTypeDef' and the function 'NVIC_Init'
    
    // 'NVIC_EnableIRQ' is only required for USART setting
    // NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Delay(void) {
    int i;

    for (i = 0; i < 2000000; i++) {}
}


void ADC1_2_IRQHandler(void) {
    uint16_t diff = ADC_GetConversionValue(ADC1) - ADC1_CONVERTED_VALUE;
    ADC1_CONVERTED_VALUE = ADC_GetConversionValue(ADC1);
}

int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};

int main(void)
{

    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    NVIC_Configure();
    
    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);
    ADC_Configure();
    
    uint16_t x;
    uint16_t y;
    uint16_t convert_x;
    uint16_t convert_y;
    while(1) {
        Touch_GetXY(&x, &y, 0);
        Convert_Pos(x, y, &convert_x, &convert_y);
        if (T_INT != 1) {
          LCD_ShowNum(0x10, 0x60, convert_x, 16, BLACK, WHITE);
          LCD_ShowNum(0x10, 0x50, convert_y, 16, BLACK, WHITE);
          //LCD_DrawRectangle(convert_x, convert_y, convert_x + 10, convert_y + 10);
          LCD_DrawCircle(convert_x,convert_y,4);
          Delay();
        }
        LCD_ShowString(0x45, 0x30, "WED_TEAM03", BLACK, WHITE );
        LCD_ShowNum(0x10, 0x80, ADC1_CONVERTED_VALUE, 16, BLACK, WHITE);
    }
    return 0;
}
