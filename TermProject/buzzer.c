#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "lcd.h"
#include "touch.h"

// Buzzor : B0

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
    TIM_TimeBaseStructure.TIM_Period = 10;    // Overflow Interrupt On 10 msec 타이머주기
    TIM_TimeBaseStructure.TIM_Prescaler = 36; // Timer/Count2 Clock = 36Mhz / (35 + 1) = 1Mhz = 1 usec
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

void delay(void)
{
    int i;
    for (i = 0; i < 1000000; i++)
    {
    }
}

int main(void)
{
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    TIM3_Configure();
    NVIC_Configure();

    enum notes
    {   
        C6 = 1046502,  // 도
        D6 = 1174659, // 래
        DS6 = 1244508, // 레 샾
        E6 = 1318510, // 미
        F6 = 1396913, // 파
        G6 = 1567982, // 솔
        A6 = 1760000, // 라
        B6 = 1975533, // 시

        C7 = 2093,  // 도
        D7 = 2349, // 래
        DS7 = 2489, // 레 샾
        E7 = 2637, // 미
        F7 = 2794, // 파
        G7 = 3136, // 솔
        A7 = 3520, // 라
        B7 = 3951, // 시
    };

    enum notes back[] = {E7, DS7, E7, DS7, E7, B6, D7, C7, A6, A6};

    while (1)
    {
        delay();
        delay();

        for (int i = 0; i < sizeof(back) / sizeof(enum notes); i++)
        {
            Music = 100000000 / back[i];
            delay();
        }
        Music = 0;
        delay();
        delay();
    }

    TIM_Cmd(TIM3, DISABLE);
    GPIOB->BRR = GPIO_Pin_0;
}
