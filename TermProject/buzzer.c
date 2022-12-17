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
        C1 = 33,  // 도(523.25Hz)
        D1 = 37, // 래
        DS1 = 39, // 레 샾
        E1 = 41, // 미
        F1 = 44, // 파
        G1 = 49, // 솔
        A1 = 55, // 라
        B1 = 62, // 시

        C2 = 65,  // 도(523.25Hz)
        D2 = 73, // 래
        DS2 = 78, // 레 샾
        E2 = 82, // 미
        F2 = 87, // 파
        G2 = 98, // 솔
        A2 = 110, // 라
        B2 = 123, // 시

        C3 = 131,  // 도(523.25Hz)
        D3 = 147, // 래
        DS3 = 156, // 레 샾
        E3 = 165, // 미
        F3 = 175, // 파
        G3 = 196, // 솔
        A3 = 220, // 라
        B3 = 247, // 시

        C4 = 261, // 도(261.63Hz)
        D4 = 293, // 레(293.66Hz)
        E4 = 329, // 미(329.63Hz)
        F4 = 349, // 파(349.23Hz)
        G4 = 392, // 솔(392.00Hz)
        A4 = 440, // 라(440.00Hz)
        B4 = 493, // 시(493.88Hz)

        C5 = 523,  // 도(523.25Hz)
        D5 = 587, // 래
        DS5 = 622, // 레 샾
        E5 = 659, // 미
        F5 = 698, // 파
        G5 = 784, // 솔
        A5 = 880, // 라
        B5 = 988, // 시

        C6 = 1047,  // 도(523.25Hz)
        D6 = 1175, // 래
        DS6 = 1245, // 레 샾
        E6 = 1319, // 미
        F6 = 1397, // 파
        G6 = 1568, // 솔
        A6 = 1760, // 라
        B6 = 1976, // 시

        C7 = 2093,  // 도(523.25Hz)
        D7 = 2349, // 래
        DS7 = 2489, // 레 샾
        E7 = 2637, // 미
        F7 = 2794, // 파
        G7 = 3136, // 솔
        A7 = 3520, // 라
        B7 = 3951, // 시

        C8 = 4186,  // 도(523.25Hz)
        D8 = 4699, // 래
        DS8 = 4978, // 레 샾
        E8 = 5274, // 미
        F8 = 5588, // 파
        G8 = 6272, // 솔
        A8 = 7040, // 라
        B8 = 7902, // 시
    };

    enum notes A[] = {G4, G4, A4, A4, G4, G4, E4, G4, G4, E4, E4, D4,
                      G4, G4, A4, A4, G4, G4, E4, G4, E4, D4, E4, C4};

    enum notes back[] = {E5, DS5, E5, DS5, E5, B4, DS5, C5, A4, A4};

    enum notes all[] = {C1, D1, DS1, E1, F1, G1, A1, B1, C2, D2, DS2, E2, F2, G2, A2, B2, C3, D3, DS3, E3, F3, G3, A3, B3, C4, D4, DS4, E4, F4, G4, A4, B4, C5, D5, DS5, E5, F5, G5, A5, B5, C6, D6, DS6, E6, F6, G6, A6, B6, C7, D7, DS7, E7, F7, G7, A7, B7, C8, D8, DS8, E8, F8, G8, A8, B8};
    while (1)
    {
        for (int i = 0; i < sizeof(all) / sizeof(enum notes); i++)
        {
            Music = all[i];
            delay();
            delay();
        }
    }

    TIM_Cmd(TIM3, DISABLE);
    GPIOB->BRR = GPIO_Pin_0;
}
