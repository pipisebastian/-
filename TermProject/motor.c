#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include <stdio.h>

/* function prototype */
void Motor_RCC_Configure(void);
void Motor_GPIO_Configure(void);
void Motor_Start(void);
void Motor_Back(void);
void Motor_TurnLeft(void);
void Motor_TurnRight(void);
void Motor_Stop(void);
void Motor_Init(void);

int16_t Motor_Pin[4] = {GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13};

void Motor_RCC_Configure(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
}

void Motor_GPIO_Configure(void) //
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}
void Motor_Start(void)
{
    GPIO_SetBits(GPIOE, MotorPin[0]);
    GPIO_ResetBits(GPIOE, MotorPin[1]);
    GPIO_ResetBits(GPIOE, MotorPin[2]);
    GPIO_SetBits(GPIOE, MotorPin[3]);
}

void Motor_Back(void)
{
    GPIO_ResetBits(GPIOE, MotorPin[0]);
    GPIO_SetBits(GPIOE, MotorPin[1]);
    GPIO_SetBits(GPIOE, MotorPin[2]);
    GPIO_ResetBits(GPIOE, MotorPin[3]);
}

void Motor_TurnLeft(void)
{
    GPIO_ResetBits(GPIOE, MotorPin[0]);
    GPIO_SetBits(GPIOE, MotorPin[1]);
    GPIO_ResetBits(GPIOE, MotorPin[2]);
    GPIO_SetBits(GPIOE, MotorPin[3]);
}

void Motor_TurnRight(void)
{
    GPIO_SetBits(GPIOE, MotorPin[0]);
    GPIO_ResetBits(GPIOE, MotorPin[1]);
    GPIO_SetBits(GPIOE, MotorPin[2]);
    GPIO_ResetBits(GPIOE, MotorPin[3]);
}

void Motor_Stop(void)
{
    GPIO_ResetBits(GPIOE, MotorPin[0]);
    GPIO_ResetBits(GPIOE, MotorPin[1]);
    GPIO_ResetBits(GPIOE, MotorPin[2]);
    GPIO_ResetBits(GPIOE, MotorPin[3]);
}

void Motor_Init(void)
{
    Motor_RCC_Configure();
    Motor_GPIO_Configure();
}

int main(void)
{

    SystemInit();
    Motor_Init();

    while (1)
    {
        Motor_Start();
    }
    return 0;
}
