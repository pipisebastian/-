#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void Start(void);
void Back(void);
void TurnLeft(void);
void TurnRight(void);
void Stop(void);

int16_t MotorPin[4] = {GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3, GPIO_Pin_4};

void RCC_Configure(void)
{
    /* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    /* PWM */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // Port B
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
}

void GPIO_Configure(void) //
{
    // PE1,2,3,4 -> IN1,IN2,IN3,IN4
    // PE5,6 -> ENA, ENB
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = MotorPin[0] | MotorPin[1] | MotorPin[2] | MotorPin[3] | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // output mode로
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    /* PWM */
    GPIO_InitTypeDef GPIO_InitStructure2;
    GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure2);
}

void PWM_Configure()
{
    // printf("%d\n", SystemCoreClock);
    prescale = (uint16_t)(SystemCoreClock / 1000000);
    // printf("%d\n", prescale);
    TIM_TimeBaseStructure.TIM_Period = 20000;
    // printf("%d\n", TIM_TimeBaseStructure.TIM_Period);
    TIM_TimeBaseStructure.TIM_Prescaler = prescale;
    // printf("%d\n", SystemCoreClock / prescale / TIM_TimeBaseStructure.TIM_Period);
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

void Start(void)
{
    GPIO_WriteBit(GPIOE, MotorPin[0], Bit_SET);
    GPIO_WriteBit(GPIOE, MotorPin[1], Bit_RESET);
    GPIO_WriteBit(GPIOE, MotorPin[2], Bit_SET);
    GPIO_WriteBit(GPIOE, MotorPin[3], Bit_RESET);
}

void Back(void)
{
    GPIO_WriteBit(GPIOE, MotorPin[0], Bit_RESET);
    GPIO_WriteBit(GPIOE, MotorPin[1], Bit_SET);
    GPIO_WriteBit(GPIOE, MotorPin[2], Bit_RESET);
    GPIO_WriteBit(GPIOE, MotorPin[3], Bit_SET);
}

void TurnLeft(void)
{
    GPIO_WriteBit(GPIOE, MotorPin[0], Bit_RESET);
    GPIO_WriteBit(GPIOE, MotorPin[1], Bit_SET);
    GPIO_WriteBit(GPIOE, MotorPin[2], Bit_SET);
    GPIO_WriteBit(GPIOE, MotorPin[3], Bit_RESET);
}

void TurnRight(void)
{
    GPIO_WriteBit(GPIOE, MotorPin[0], Bit_SET);
    GPIO_WriteBit(GPIOE, MotorPin[1], Bit_RESET);
    GPIO_WriteBit(GPIOE, MotorPin[2], Bit_RESET);
    GPIO_WriteBit(GPIOE, MotorPin[3], Bit_SET);
}

void Stop(void)
{
    GPIO_WriteBit(GPIOE, MotorPin[0], Bit_RESET);
    GPIO_WriteBit(GPIOE, MotorPin[1], Bit_RESET);
    GPIO_WriteBit(GPIOE, MotorPin[2], Bit_RESET);
    GPIO_WriteBit(GPIOE, MotorPin[3], Bit_RESET);
}

void ControlMotorSpeed(int speed)
{

    // pwm조정 음냐링
}

void delay()
{
  printf("delay\n");
   for(int j = 0; j < 5000000; j++) {
      continue;
   }
}

int main(void)
{

    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    PWM_Configure();

    uint16_t rotation[2] = {700, 2300};
    int i = 0;

    /* GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_SET);
    GPIO_WriteBit(GPIOE, GPIO_Pin_6, Bit_SET); */

    while (1)
    {
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = rotation[i]; // us
        TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	i++;
	i %= 2;
	delay();
	    
        //속도 느리게
        // ControlMotorSpeed(50);
        // TurnLeft();
    }
    return 0;
}
