
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "misc.h"

/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void USART1_Init(void);
void NVIC_Configure(void);

void EXTI15_10_IRQHandler(void);

void Delay(void);

void sendDataUART1(uint16_t data);

int flag = 0;
//---------------------------------------------------------------------------------------------------

void RCC_Configure(void)
{
    // TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'

    /* UART TX/RX port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* JoyStick Up/Down port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    /* 필요없음 JoyStick Selection port clock enable */

    /* LED port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    /* USART1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // TODO: Initialize the GPIO pins using the structure 'GPIO_InitTypeDef' and the function 'GPIO_Init'

    /* JoyStick up, down pin setting */
    // PC 2,5 -> input with pull-up/pull down mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU | GPIO_Mode_IPD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 필요없음 JoyStick selection pin setting */

    /* button pin setting */
    // PD 11 -> input with pull-up/pull down mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU | GPIO_Mode_IPD;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* LED pin setting*/
    // PD 2,3,4,5 -> output mode, 50MHz
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* UART pin setting */
    // TX -> PA 9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate function output Push-pull로 설정
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // RX -> PA 10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU | GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void EXTI_Configure(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;

    // TODO: Select the GPIO pin (Joystick, button) used as EXTI Line using function 'GPIO_EXTILineConfig'
    // TODO: Initialize the EXTI using the structure 'EXTI_InitTypeDef' and the function 'EXTI_Init'
    // EXTI Line으로 쓸 pin 선택
    // PIN 2,5,11 -> LINE 2,5,11

    /* Joystick Down PC 2 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //제공코드
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Joystick Up PC 5*/
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource5);
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* 필요없음 Joystick Selection */

    /* Button  PD11 */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource11);
    EXTI_InitStructure.EXTI_Line = EXTI_Line11;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    // NOTE: do not select the UART GPIO pin used as EXTI Line here
}

void USART1_Init(void)
{
    USART_InitTypeDef USART1_InitStructure;

    // Enable the USART1 peripheral
    USART_Cmd(USART1, ENABLE);

    // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    // USART 초기설정

    USART1_InitStructure.USART_BaudRate = 28800;
    USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART1_InitStructure.USART_Parity = USART_Parity_No;
    USART1_InitStructure.USART_StopBits = USART_StopBits_1;
    USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // Tx, Rx enable
    USART_Init(USART1, &USART1_InitStructure);

    // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
    // USART Interrupts를 enable!
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void NVIC_Configure(void)
{

    NVIC_InitTypeDef NVIC_InitStructure;

    // TODO: fill the arg you want
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // TODO: Initialize the NVIC using the structure 'NVIC_InitTypeDef' and the function 'NVIC_Init'
    //우선순위 지정 -> 1. 조이스틱 UP // 2. 조이스틱 DOWN // 3. BTN // 4. USART입력

    // Joystick Down PC2
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Joystick Up PC5
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // User S1 Button PA11
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // UART1
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
//////////////////////////여기까지 했습니다!

void USART1_IRQHandler()
{
    uint16_t word;
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART1);

        // TODO implement
        /* USART -> 입력 "a" -> 동작 A
                 -> 입력 "b" -> 동작 B
        */
        if (word == 'a')
        {
            flag = 0;
        }
        else if (word == "b")
        {
            flag = 1;
        }

        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void EXTI15_10_IRQHandler(void)
{ // when the button is pressed => putty에 "team~ 출력"

    if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_11) == Bit_RESET)
        {
            // TODO implement
        }
        EXTI_ClearITPendingBit(EXTI_Line11);
    }
}

void EXTI2_IRQHandler(void)
{ // 조이스틱 down (PC2) -> b 동작 , 역방향, flag = 1

    if (EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == Bit_RESET)
        {
            flag = 1;
        }
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

void EXTI9_5_IRQHandler(void)
{ // 조이스틱 up (PC5) -> a 동작 , 순방향, flag = 0

    if (EXTI_GetITStatus(EXTI_Line5) != RESET)
    {
        if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_5) == Bit_RESET)
        {
            flag = 0;
        }
        EXTI_ClearITPendingBit(EXTI_Line5);
    }
}
void Delay(void)
{
    int i;

    for (i = 0; i < 2000000; i++)
    {
    }
}

void sendDataUART1(uint16_t data)
{
    /* Wait till TC is set */
    while ((USART1->SR & USART_SR_TC) == 0)
        ;
    USART_SendData(USART1, data);
}

int main(void)
{

    SystemInit();

    RCC_Configure();

    GPIO_Configure();

    EXTI_Configure();

    USART1_Init();

    NVIC_Configure();

    while (1)
    {
        // TODO: implement

        // Delay
        Delay();
    }
    return 0;
}
