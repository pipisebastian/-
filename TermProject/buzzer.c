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
    TIM_TimeBaseStructure.TIM_Period = 10;    // Overflow Interrupt On 10 usec 타이머주기
    TIM_TimeBaseStructure.TIM_Prescaler = 32; // Timer/Count2 Clock = 36Mhz / (35 + 1) = 1Mhz = 1 usec
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
    {   // 음 약간 이상함.
        C4 = 26163, // 도(261.63Hz)
        D4 = 29366, // 래(293.66Hz)
        E4 = 32963, // 미(329.63Hz)
        F4 = 34923, // 파(349.23Hz)
        G4 = 39200, // 솔(392.00Hz)
        A4 = 44000, // 라(440.00Hz)
        B4 = 49388, // 시(493.88Hz)
        C5 = 52325  // 도(523.25Hz)

//         C4 = 956, // 도(261.63Hz)
//         D4 = 851, // 래(293.66Hz)
//         E4 = 758, // 미(329.63Hz)
//         F4 = 716, // 파(349.23Hz)
//         G4 = 638, // 솔(392.00Hz)
//         A4 = 568, // 라(440.00Hz)
//         B4 = 506, // 시(493.88Hz)
//         C5 = 523  // 도(523.25Hz)
    };

    enum notes A[] = {G4, G4, A4, A4, G4, G4, E4, G4, G4, E4, E4, D4,
                      G4, G4, A4, A4, G4, G4, E4, G4, E4, D4, E4, C4};

    while (1)
    {
        for (int i = 0; i < sizeof(A) / sizeof(enum notes); i++)
        {
            Music = A[i];
            delay();
            Music = 0;
            delay();
        }
    }

    TIM_Cmd(TIM3, DISABLE);
    GPIOB->BRR = GPIO_Pin_0;
}
