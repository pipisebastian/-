#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"

#define MUSIC_REST 0
#define MUSIC_DO 191
#define MUSIC_C_SHARP 180
#define MUSIC_RE 170
#define MUSIC_D_SHARP 161
#define MUSIC_MI 152
#define MUSIC_PA 143
#define MUSIC_F_SHARP 135
#define MUSIC_SOL 128
#define MUSIC_G_SHARP 120
#define MUSIC_RA 114
#define MUSIC_A_SHARP 107
#define MUSIC_SI 101
#define MUSIC_HDO 96

uint32_t Sound = 0;
uint32_t Music = 0;

void RCC_Configure(void)
{
    /* PWM */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // TIM3
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

void NVIC_Configure(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM3 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM3_Configure(void)
{
    uint16_t prescale = (uint16_t)(SystemCoreClock / 10);

    /* Time base configuration */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 10;          // Overflow Interrupt On 10 usec 타이머주기
    TIM_TimeBaseStructure.TIM_Prescaler = prescale; // Timer/Count2 Clock = 36Mhz / (35 + 1) = 1Mhz = 1 usec
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 카운터모드동작
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* TIM3 counter enable */
    TIM_Cmd(TIM3, ENABLE);

    /* TIM IT(인터럽트) enable */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
}

void TIM3_IRQHandler(void) // 1mS Timer
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

        Sound++;

        if (Sound >= Music)
        {
            GPIOB->ODR ^= GPIO_Pin_0;
            Sound = 0;
        }
    }
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
    TIM3_Configure();
    NVIC_Configure();

    Music = MUSIC_SOL;
    delay();
    Music = MUSIC_REST;
    delay();
    Music = MUSIC_SOL;
    delay();
    Music = MUSIC_RA;
    delay();
    Music = MUSIC_REST;
    delay();
    Music = MUSIC_RA;
    delay();
    Music = MUSIC_SOL;
    delay();
    Music = MUSIC_REST;
    delay();
    Music = MUSIC_SOL;
    delay();
    Music = MUSIC_MI;
    delay();
    delay();

    TIM_Cmd(TIM3, DISABLE);
    GPIOB->BRR = GPIO_Pin_0;
} 
