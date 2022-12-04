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
}

void GPIO_Configure(void) //
{
    // PE1,2,3,4 -> IN1,IN2,IN3,IN4
    // PE5,6 -> ENA, ENB
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // output mode로
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}
void Start(void)
{
    GPIO_SetBits(GPIOE, MotorPin[0]);
    GPIO_ResetBits(GPIOE, MotorPin[1]);
    GPIO_SetBits(GPIOE, MotorPin[2]);
    GPIO_ResetBits(GPIOE, MotorPin[3]);

    // GPIO_WriteBit(GPIOE, MotorPin[0], Bit_SET);
    // GPIO_WriteBit(GPIOE, MotorPin[1], Bit_RESET);
    // GPIO_WriteBit(GPIOE, MotorPin[2], Bit_SET);
    // GPIO_WriteBit(GPIOE, MotorPin[3], Bit_RESET);
}

void Back(void)
{
    GPIO_ResetBits(GPIOE, MotorPin[0]);
    GPIO_SetBits(GPIOE, MotorPin[1]);
    GPIO_ResetBits(GPIOE, MotorPin[2]);
    GPIO_SetBits(GPIOE, MotorPin[3]);

    //     GPIO_WriteBit(GPIOE, MotorPin[0], Bit_RESET);
    //     GPIO_WriteBit(GPIOE, MotorPin[1], Bit_SET);
    //     GPIO_WriteBit(GPIOE, MotorPin[2], Bit_RESET);
    //     GPIO_WriteBit(GPIOE, MotorPin[3], Bit_SET);
    //
}

void TurnLeft(void)
{
    printf("1");
    GPIO_ResetBits(GPIOE, MotorPin[0]);
    GPIO_SetBits(GPIOE, MotorPin[1]);
    GPIO_SetBits(GPIOE, MotorPin[2]);
    GPIO_ResetBits(GPIOE, MotorPin[3]);
    printf("2");
    // GPIO_WriteBit(GPIOE, MotorPin[0], Bit_RESET);
    // GPIO_WriteBit(GPIOE, MotorPin[1], Bit_SET);
    // GPIO_WriteBit(GPIOE, MotorPin[2], Bit_SET);
    // GPIO_WriteBit(GPIOE, MotorPin[3], Bit_RESET);
}

void TurnRight(void)
{
    GPIO_SetBits(GPIOE, MotorPin[0]);
    GPIO_ResetBits(GPIOE, MotorPin[1]);
    GPIO_ResetBits(GPIOE, MotorPin[2]);
    GPIO_SetBits(GPIOE, MotorPin[3]);
    // GPIO_WriteBit(GPIOE, MotorPin[0], Bit_SET);
    // GPIO_WriteBit(GPIOE, MotorPin[1], Bit_RESET);
    // GPIO_WriteBit(GPIOE, MotorPin[2], Bit_RESET);
    // GPIO_WriteBit(GPIOE, MotorPin[3], Bit_SET);
}

void Stop(void)
{
    GPIO_ResetBits(GPIOE, MotorPin[0]);
    GPIO_ResetBits(GPIOE, MotorPin[1]);
    GPIO_ResetBits(GPIOE, MotorPin[2]);
    GPIO_ResetBits(GPIOE, MotorPin[3]);
    // GPIO_WriteBit(GPIOE, MotorPin[0], Bit_RESET);
    // GPIO_WriteBit(GPIOE, MotorPin[1], Bit_RESET);
    // GPIO_WriteBit(GPIOE, MotorPin[2], Bit_RESET);
    // GPIO_WriteBit(GPIOE, MotorPin[3], Bit_RESET);
}

void ControlMotorSpeed(int speed)
{

    // pwm조정 음냐링
}

int main(void)
{

    SystemInit();
    RCC_Configure();
    GPIO_Configure();

    // GPIO_WriteBit(GPIOE, GPIO_Pin_5, Bit_SET);
    // GPIO_WriteBit(GPIOE, GPIO_Pin_6, Bit_SET);

    while (1)
    {
        //속도 느리게
        // ControlMotorSpeed(50);
        printf("0");
        TurnLeft();
        printf("4");
    }
    return 0;
}
