#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "core_cm3.h"
#include "touch.h"
#include "misc.h"
#include "stdio.h"

#define U3_BUFFER_SIZE 1000

// 사용하는 포트 정리
// Buzzer
// PB0 : Buzzer
// 조도센서
// PC0  : 조도센서 ADC
// LED
// PE0 : 양 쪽 전조등
// PE1 : 왼쪽 방향 지시등
// PE2 : 오른쪽 방향지시등
// Bluetooth
// PA2 : TX
// PA3 : RX
// ultrasound
// PE3 : ultrasound Echo (수신부 - INPUT)
// PE4 : ultrasound Trig (송신부 - OUTPUT)
// 모터
// PE10 : 왼쪽 앞바퀴
// PE11 : 오른쪽 앞바퀴
// PE12 : 왼쪽 뒷바퀴
// PE13 : 오른쪽 바퀴

uint16_t u3_rx_buffer[U3_BUFFER_SIZE];

uint32_t u3_rx_point_head = 0;
uint32_t u3_rx_point_tail = 0;
uint16_t data = 1000;

int RightLED = 0;
int LeftLED = 0;

uint32_t usTime = 0;

uint32_t Sound = 0;
uint32_t Music = 0;
uint32_t Music_index = 0;

// 조도센서 전역변수
uint16_t ADC1_CONVERTED_VALUE;

// 모터 전역변수
int16_t Motor_Pin[4] = {GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13};
int16_t Motor_front[4] = {GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13};

/* function prototype */
void LED_RCC_Configure(void);
void LED_GPIO_Configure(void);
void LED_Init(void);
void LEDTurnOnOff(void);

int Bluetooth_Uart3_Is_Empty(void);
void Bluetooth_RCC_Configure(void);
void Bluetooth_GPIO_Configure(void);
void Bluetooth_NVIC_Configure(void);
void Bluetooth_u3_increase_point_value(uint32_t *data_p);
void Bluetooth_USART2_Init(void);
void Bluetooth_Init(void);
uint16_t Bluetooth_Uart3_DeQueue(void);
// void Bluetooth_EXTI_Configure(void);
// void Bluetooth_EXTI15_10_IRQHandler(void);
// void Bluetooth_SendDataUART1(uint16_t data);

void UltraSound_RCC_Configure(void);
void UltraSound_GPIO_Configure(void);
void UltraSound_TIM_Configure(void);
void UltraSound_NVIC_Configure(void);
void TIM2_IRQHandler(void);
void UltraSound_Init(void);
int Read_Distance(void);

// 모터
void Motor_RCC_Configure(void);
void Motor_GPIO_Configure(void);
void Motor_Start(void);
void Motor_Back(void);
void Motor_TurnLeft(void);
void Motor_TurnRight(void);
void Motor_Stop(void);
void Motor_Init(void);

// 조도센서
void Light_RCC_Configure(void);
void Light_GPIO_Configure(void);
void Light_NVIC_Configure(void);
void Light_ADC_Configure(void);
void Light_Init(void);

// buzzer
void Buzzer_RCC_Configure(void);
void Buzzer_GPIO_Configure(void);
void Buzzer_NVIC_Configure(void);
void Buzzer_TIM3_Configure(void);
void Buzzer_playBackMelody(void);
void Buzzer_playBeepMelody(void);
void Buzzer_Init(void);
void TIM3_IRQHandler(void);

// 딜레이
void Delay(void);
void Buzzer_daley(void);

void LED_RCC_Configure(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
}

void LED_GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  // LED Output Mode
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void LED_Init(void)
{
  LED_RCC_Configure();
  LED_GPIO_Configure();
}

void LEDTurnOnOff(void)
{
  if (ADC1_CONVERTED_VALUE < 2200)
  {
    GPIO_SetBits(GPIOE, GPIO_Pin_0); // 전조등 끄기
  }
  else
  {
    GPIO_ResetBits(GPIOE, GPIO_Pin_0); // 전조등 켜기
  }

  if (LeftLED % 2 == 0)
  {
    GPIO_SetBits(GPIOE, GPIO_Pin_1); // 왼쪽 방향지시등 끄기
  }
  else
  {
    GPIO_ResetBits(GPIOE, GPIO_Pin_1); // 왼쪽 방향지시등 켜기
  }

  if (RightLED % 2 == 0)
  {
    GPIO_SetBits(GPIOE, GPIO_Pin_2); // 오른족 방향지시등 끄기
  }
  else
  {
    GPIO_ResetBits(GPIOE, GPIO_Pin_2); // 오른쪽 방향지시등 켜기
  }
}

void Bluetooth_Uart3_EnQueue(uint16_t data)
{
  u3_rx_buffer[u3_rx_point_head] = data;
  Bluetooth_u3_increase_point_value(&u3_rx_point_head);
}

void Bluetooth_u3_increase_point_value(uint32_t *data_p)
{
  (*data_p)++;
  if (U3_BUFFER_SIZE == (*data_p))
  {
    (*data_p) = 0;
  }
}

uint16_t Bluetooth_Uart3_DeQueue(void)
{
  uint16_t retVal = u3_rx_buffer[u3_rx_point_tail];
  Bluetooth_u3_increase_point_value(&u3_rx_point_tail);
  return retVal;
}

int Bluetooth_Uart3_Is_Empty(void)
{
  if (u3_rx_point_head == u3_rx_point_tail)
  {
    return 1;
  }
  return 0;
}

void Bluetooth_RCC_Configure(void)
{
  // Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'

  /* UART TX/RX port clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* USART2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* Alternate Function IO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void Bluetooth_GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* UART2 pin setting */
  // TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* UART1 pin setting */
  // TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  // RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Bluetooth_USART2_Init(void)
{
  USART_InitTypeDef USART2_InitStructure;

  // Enable the USART2 peripheral
  USART_Cmd(USART2, ENABLE);

  // Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
  USART2_InitStructure.USART_BaudRate = 9600;
  USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART2_InitStructure.USART_Parity = USART_Parity_No;
  USART2_InitStructure.USART_StopBits = USART_StopBits_1;
  USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART2, &USART2_InitStructure);

  // Enable the USART2 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void Bluetooth_NVIC_Configure(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  // UART1
  // 'NVIC_EnableIRQ' is only required for USART setting
  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // UART2
  // 'NVIC_EnableIRQ' is only required for USART setting
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Bluetooth_Init(void)
{
  Bluetooth_RCC_Configure();
  Bluetooth_GPIO_Configure();
  Bluetooth_USART2_Init();
  Bluetooth_NVIC_Configure();
}

void UltraSound_RCC_Configure(void)
{
  // Alternate Function IO clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  // TIM2 clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  // port E RCC ENABLE
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

  /* ADC1 Enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}

void UltraSound_GPIO_Configure(void)
{
  // UltraSound
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // PE4 : Trig (송신부 - OUTPUT)
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // PE3 : Echo (수신부 - INPUT)
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void UltraSound_TIM_Configure(void)
{
  // set 1us
  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  TIM_InitStructure.TIM_Prescaler = 72;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_Period = 1;
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TIM_InitStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);
}

void UltraSound_NVIC_Configure(void)
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable TIM2 Global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    usTime++;
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

void UltraSound_Init(void)
{
  UltraSound_RCC_Configure();
  UltraSound_GPIO_Configure();
  UltraSound_TIM_Configure();
  UltraSound_NVIC_Configure();
}

int Read_Distance(void)
{
  uint32_t prev = 0;
  GPIO_SetBits(GPIOE, GPIO_Pin_4);
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);
  Delay();
  GPIO_ResetBits(GPIOE, GPIO_Pin_4);
  uint8_t val = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3);
  prev = usTime;
  while (val == RESET)
  {
    if (usTime - prev >= 10000)
      break; // 10ms
    else
    {
      val = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3);
    }
  }
  if (val == SET)
  {
    prev = usTime;
    while (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) != RESET)
    {
    }
    return (usTime - prev) * 34 / 1000;
  }
  else
  {
    return 150;
  }
}

void USART2_IRQHandler(void)
{
  uint16_t word;
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
    // the most recent received data by the USART1 peripheral
    word = USART_ReceiveData(USART2);
    Bluetooth_Uart3_EnQueue(word);

    // clear 'Read data register not empty' flag
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}

// 모터
void Motor_RCC_Configure(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
}

void Motor_GPIO_Configure(void) //
{
  GPIO_InitTypeDef GPIO_InitStructure;
  // 뒷 바퀴
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  // 앞 바퀴
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}
void Motor_Start(void)
{
  /*
    GPIO_SetBits(GPIOE, Motor_Pin[0]);
    GPIO_ResetBits(GPIOE, Motor_Pin[1]);
    GPIO_ResetBits(GPIOE, Motor_Pin[2]);
    GPIO_SetBits(GPIOE, Motor_Pin[3]);
*/

  GPIO_SetBits(GPIOD, Motor_Pin[0]);
  GPIO_ResetBits(GPIOD, Motor_Pin[1]);
  GPIO_ResetBits(GPIOD, Motor_Pin[2]);
  GPIO_SetBits(GPIOD, Motor_Pin[3]);
}

void Motor_Back(void)
{
  /*
    GPIO_ResetBits(GPIOE, Motor_Pin[0]);
    GPIO_SetBits(GPIOE, Motor_Pin[1]);
    GPIO_SetBits(GPIOE, Motor_Pin[2]);
    GPIO_ResetBits(GPIOE, Motor_Pin[3]);
*/

  GPIO_ResetBits(GPIOD, Motor_Pin[0]);
  GPIO_SetBits(GPIOD, Motor_Pin[1]);
  GPIO_SetBits(GPIOD, Motor_Pin[2]);
  GPIO_ResetBits(GPIOD, Motor_Pin[3]);
}

void Motor_TurnLeft(void)
{ /*
     GPIO_ResetBits(GPIOE, Motor_Pin[0]);
     GPIO_SetBits(GPIOE, Motor_Pin[1]);
     GPIO_ResetBits(GPIOE, Motor_Pin[2]);
     GPIO_SetBits(GPIOE, Motor_Pin[3]);
 */

  GPIO_ResetBits(GPIOD, Motor_Pin[0]);
  GPIO_SetBits(GPIOD, Motor_Pin[1]);
  GPIO_ResetBits(GPIOD, Motor_Pin[2]);
  GPIO_SetBits(GPIOD, Motor_Pin[3]);
}

void Motor_TurnRight(void)
{ /*
    GPIO_SetBits(GPIOE, Motor_Pin[0]);
    GPIO_ResetBits(GPIOE, Motor_Pin[1]);
    GPIO_SetBits(GPIOE, Motor_Pin[2]);
    GPIO_ResetBits(GPIOE, Motor_Pin[3]);
    */

  GPIO_SetBits(GPIOD, Motor_Pin[0]);
  GPIO_ResetBits(GPIOD, Motor_Pin[1]);
  GPIO_SetBits(GPIOD, Motor_Pin[2]);
  GPIO_ResetBits(GPIOD, Motor_Pin[3]);
}

void Motor_Stop(void)
{
  GPIO_ResetBits(GPIOE, Motor_Pin[0]);
  GPIO_ResetBits(GPIOE, Motor_Pin[1]);
  GPIO_ResetBits(GPIOE, Motor_Pin[2]);
  GPIO_ResetBits(GPIOE, Motor_Pin[3]);

  GPIO_ResetBits(GPIOD, Motor_Pin[0]);
  GPIO_ResetBits(GPIOD, Motor_Pin[1]);
  GPIO_ResetBits(GPIOD, Motor_Pin[2]);
  GPIO_ResetBits(GPIOD, Motor_Pin[3]);
}

void Motor_Init(void)
{
  Motor_RCC_Configure();
  Motor_GPIO_Configure();
}

// 조도센서

void Light_RCC_Configure(void)
{
  // TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'

  /* Alternate Function IO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* ADC */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

void Light_ADC_Configure(void)
{
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
  while (ADC_GetResetCalibrationStatus(ADC1))
    ;
  ADC_StartCalibration(ADC1);
  while (ADC_GetCalibrationStatus(ADC1))
    ;
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void Light_GPIO_Configure(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // TODO: Initialize the GPIO pins using the structure 'GPIO_InitTypeDef' and the function 'GPIO_Init'

  // ADC
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Light_NVIC_Configure(void)
{

  NVIC_InitTypeDef NVIC_InitStructure;

  // TODO: fill the arg you want
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

  // TODO: Initialize the NVIC using the structure 'NVIC_InitTypeDef' and the function 'NVIC_Init'

  // 'NVIC_EnableIRQ' is only required for USART setting
  // NVIC_EnableIRQ(USART1_IRQn);
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // TODO
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void ADC1_2_IRQHandler(void)
{
  uint16_t diff = ADC_GetConversionValue(ADC1) - ADC1_CONVERTED_VALUE;
  ADC1_CONVERTED_VALUE = ADC_GetConversionValue(ADC1);
}

void Light_Init(void)
{
  Light_RCC_Configure();
  Light_GPIO_Configure();
  Light_NVIC_Configure();
  Light_ADC_Configure();
}

// Buzzer

void Buzzer_RCC_Configure(void)
{
  /* PWM */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // TIM3
  /* BUZZER */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // PB0
}

void Buzzer_GPIO_Configure(void)
{
  /* BUZZER */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Buzzer_NVIC_Configure(void)
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

void Buzzer_TIM3_Configure(void)
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
  TIM_Cmd(TIM3, DISABLE);

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

void Buzzer_playBackMelody(void)
{
  TIM_Cmd(TIM3, ENABLE);

  enum notes
  {
    C6 = 1046502,  // 도
    D6 = 1174659,  // 래
    DS6 = 1244508, // 레 샾
    E6 = 1318510,  // 미
    F6 = 1396913,  // 파
    G6 = 1567982,  // 솔
    A6 = 1760000,  // 라
    B6 = 1975533,  // 시

    C7 = 2093005,  // 도
    D7 = 2349318,  // 래
    DS7 = 2489016, // 레 샾
    E7 = 2637020,  // 미
    F7 = 2793826,  // 파
    G7 = 3135963,  // 솔
    A7 = 3520000,  // 라
    B7 = 3951066,  // 시
  };

  enum notes back[] = {E7, DS7, E7, DS7, E7, B6, D7, C7, A6, A6};

  for (int i = 0; i < sizeof(back) / sizeof(enum notes); i++)
  {
    Music = 100000000 / back[i];
    Buzzer_daley();
  }
  Music_index++;

  Music_index %= 10;

  TIM_Cmd(TIM3, DISABLE);
  GPIOB->BRR = GPIO_Pin_0;
}

void Buzzer_playBeepMelody(void)
{
  TIM_Cmd(TIM3, ENABLE);

  Music = 100000 / 523;
  Buzzer_daley();
  Buzzer_daley();
  Music = 100000 / 523;
  Buzzer_daley();

  TIM_Cmd(TIM3, DISABLE);
  GPIOB->BRR = GPIO_Pin_0;
}

void Buzzer_Init(void)
{
  Buzzer_RCC_Configure();
  Buzzer_GPIO_Configure();
  Buzzer_NVIC_Configure();
  Buzzer_TIM3_Configure();
}

void Delay(void)
{
  int i;

  for (i = 0; i < 1000000; i++)
  {
  }
}

void Buzzer_daley(void)
{
  int i;

  for (i = 0; i < 100000; i++)
  {
  }
}

int main(void)
{

  SystemInit();
  LED_Init();
  Bluetooth_Init();
  UltraSound_Init();
  Motor_Init();  // 모터
  Light_Init();  // 조도센서
  Buzzer_Init(); // Buzzer

  TIM_Cmd(TIM3, DISABLE);
  GPIOB->BRR = GPIO_Pin_0;
  int count = 1;
  while (1)
  {
    Motor_Start();
      /*
    LEDTurnOnOff();

    if (0 == Bluetooth_Uart3_Is_Empty())
      data = Bluetooth_Uart3_DeQueue() - '0';

    if (data == 0)
    { // 후진
      Motor_Back();

      Buzzer_playBackMelody();
      Music_index++;
    }
    else if (data == 4)
    { // 왼쪽 방향 지시등
      LeftLED++;
      LeftLED %= 2;
      Music_index = 0;
    }
    else if (data == 5)
    { // 오른쪽 방향 지시등
      RightLED++;
      RightLED %= 2;
      Music_index = 0;
    }
    else if (data == 6)
    { // 크락션
      Buzzer_playBeepMelody();
      Music_index = 0;
    }
    else if (data == 9)
    { // 정지
      Motor_Stop();
    }
    else if (Read_Distance() < 15)
    {
      // 정지!
      Motor_Stop();
      Music_index = 0;
    }
    else if (data == 1)
    { // 전진
      Motor_Start();
      Music_index = 0;
    }
    else if (data == 2)
    { // 좌회전
      Motor_TurnLeft();
      Music_index = 0;
    }
    else if (data == 3)
    { // 우회전
      Motor_TurnRight();
      Music_index = 0;
    }
    */
  }
  return 0;
}
