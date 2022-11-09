#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "lcd.h"
#include "touch.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void ADC_Configure(void);
void DMA_Configure(void); // DMA
void Delay(void);

 // ADC값 저장
volatile uint32_t ADC_Value[1];

void RCC_Configure(void)
{
    /* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    /* ADC */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* DMA */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
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
    // ADC_ITConfig -> ADC_DMACmd (ppt)
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void DMA_Configure(void) {

    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;  //ADC 어디서 불러올지
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &ADC_Value; // 저장할 주소(버퍼)
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // Read from peripheral
    DMA_InitStructure.DMA_BufferSize = 1; // 조도센서 값 한개
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 주변 레지스터의 추가 안 됨

    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//여기를 Enable해야 메모리 주소르 증가시키면서 다음메모리에 정보 씀
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; // 32bit
    DMA_InitStructure.DMA_MemoryDataSize =  DMA_MemoryDataSize_Word; // 32bit
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//정해진 크기의 메모리에 데이터를 쓰면 처음으로 돌아와서 씀
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //NVIC에서 우선순위 결정한거랑 비슷한건가봄!
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//전송할 data의 수가 0이 되면 stream이 disable
    
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel1, ENABLE);
   
}

void GPIO_Configure(void) // 조도센서 C0 input mode 설정
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Delay(void) {
    int i;
    for (i = 0; i < 2000000; i++) {}
}

int main(void)
{

    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    
    LCD_Init();
    Touch_Configuration();
    Touch_Adjust();
    LCD_Clear(WHITE);
    ADC_Configure();
    DMA_Configure(); // DMA
    

    while(1) {
        if(  ADC_Value[0] < 200 ){ // 플래시 비출때
            // 화면 회색
             LCD_Clear(WHITE); //배경 바꾸면 글자색도 사라짐 -> 업데이트
             LCD_ShowNum(0x10, 0x80, ADC_Value[0], 16, BLACK, WHITE);
        }
        else { //플래시 안 비출때
            // 화면 흰색
             LCD_Clear(GRAY);
             LCD_ShowNum(0x10, 0x80, ADC_Value[0], 16, BLACK, GRAY);
        }
        
        
    }
    return 0;
}
