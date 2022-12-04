#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "touch.h"
#include "misc.h"

#define U3_BUFFER_SIZE 100

// 사용하는 포트 정리
// PE0 : 양 쪽 전조등
// PE1 : 왼쪽 방향 지시등
// PE2 : 오른쪽 방향지시등
// PE3 : ultrasound Echo (수신부 - INPUT)
// PE4 : ultrasound Trig (송신부 - OUTPUT)

uint16_t u3_rx_buffer[U3_BUFFER_SIZE];

uint32_t u3_rx_point_head = 0;
uint32_t u3_rx_point_tail = 0;
uint16_t data;

int RightLED = 0;
int LeftLED = 0;

uint32_t usTime=0;

/* function prototype */
void LED_RCC_Configure(void);
void LED_GPIO_Configure(void);
void LED_Init(void);
void LEDTurnOnOff(void);

int Bluetooth_Uart3_Is_Empty(void);
void Bluetooth_RCC_Configure(void);
void Bluetooth_GPIO_Configure(void);
void Bluetooth_USART1_Init(void);
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

void Delay(void);

void LED_RCC_Configure(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
}

void LED_GPIO_Configure(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  // LED Output Mode
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void LED_Init(void) {
  LED_RCC_Configure();
  LED_GPIO_Configure();
}

void LEDTurnOnOff(void) {
  if (/* TODO 조도센서 일정 값 이상 */) {
    GPIO_SetBits(GPIOE, GPIO_Pin_0); // 전조등 끄기
  } else {
    GPIO_ResetBits(GPIOE, GPIO_Pin_0); // 전조등 켜기
  }

  if (LeftLED % 2 == 0) {
    GPIO_SetBits(GPIOE, GPIO_Pin_1); // 왼쪽 방향지시등 끄기
  } else {
    GPIO_ResetBits(GPIOE, GPIO_Pin_1); // 왼쪽 방향지시등 켜기
  }

  if (RightLED % 2 == 0) {
    GPIO_SetBits(GPIOE, GPIO_Pin_1); // 왼쪽 방향지시등 끄기
  } else {
    GPIO_ResetBits(GPIOE, GPIO_Pin_1); // 왼쪽 방향지시등 켜기
  }
}

void Bluetooth_Uart3_EnQueue(uint16_t data)
{
    u3_rx_buffer[u3_rx_point_head] = data;
    Bluetooth_u3_increase_point_value(&u3_rx_point_head);
}

void Bluetooth_u3_increase_point_value(uint32_t *data_p) {
  (*data_p)++;
  if (U3_BUFFER_SIZE == (*data_p)) {
    (*data_p) = 0;
  }
}

uint16_t Bluetooth_Uart3_DeQueue(void) {
  uint16_t retVal = u3_rx_buffer[u3_rx_point_tail];
  Bluetooth_u3_increase_point_value(&u3_rx_point_tail);
  return retVal;
}

int Bluetooth_Uart3_Is_Empty(void) {
  if (u3_rx_point_head == u3_rx_point_tail) {
    return 1;
  }
  return 0;
}

void Bluetooth_RCC_Configure(void) {
  // Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'

  /* UART TX/RX port clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* USART1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  /* USART2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /* Alternate Function IO clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void Bluetooth_GPIO_Configure(void) {
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

void Bluetooth_USART1_Init(void) {
  USART_InitTypeDef USART1_InitStructure;

  // Enable the USART1 peripheral
  USART_Cmd(USART1, ENABLE);

  // Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
  USART1_InitStructure.USART_BaudRate = 9600;
  USART1_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART1_InitStructure.USART_Parity = USART_Parity_No;
  USART1_InitStructure.USART_StopBits = USART_StopBits_1;
  USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART1, &USART1_InitStructure);

  // Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void Bluetooth_USART2_Init(void) {
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

void Bluetooth_NVIC_Configure(void) {
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

void Bluetooth_Init(void) {
  Bluetooth_RCC_Configure();
  Bluetooth_GPIO_Configure();
  Bluetooth_USART1_Init();
  Bluetooth_USART2_Init();
  Bluetooth_NVIC_Configure();
}

void UltraSound_RCC_Configure(void) {
  // Alternate Function IO clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  // TIM2 clock enable
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  // port E RCC ENABLE
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

  /* ADC1 Enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
}

void UltraSound_GPIO_Configure(void) {
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

void UltraSound_TIM_Configure(void) {
  //set 1us
  TIM_TimeBaseInitTypeDef TIM_InitStructure;
  TIM_InitStructure.TIM_Prescaler = 72;
  TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_InitStructure.TIM_Period = 1;
  TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TIM_InitStructure);
  
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
  TIM_Cmd(TIM2, ENABLE);
}

void UltraSound_NVIC_Configure(void) {
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitTypeDef NVIC_InitStructure;
  /* Enable TIM2 Global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void TIM2_IRQHandler(void) {
  if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    usTime++;
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}

void UltraSound_Init(void) {
  UltraSound_RCC_Configure();
  UltraSound_GPIO_Configure();
  UltraSound_TIM_Configure();
  UltraSound_NVIC_Configure();
}

int Read_Distance(void){
  uint32_t prev=0;
  GPIO_SetBits(GPIOE,GPIO_Pin_4);
  GPIO_ResetBits(GPIOE, GPIO_Pin_3);
  Delay();
  GPIO_ResetBits(GPIOE,GPIO_Pin_4);
  uint8_t val = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3);
  prev = usTime;
  while(val == RESET){
    if(usTime - prev >= 10000) break; // 10ms
    else{
      val = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3);
    }
  }
  if(val == SET) {
    prev = usTime;
    while(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) != RESET)
    {
    }
    return (usTime - prev) * 34 / 1000;
  }else{
      return 150;
  }
}

void USART1_IRQHandler(void) {
  uint16_t word;
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    // the most recent received data by the USART1 peripheral
    word = USART_ReceiveData(USART1);
    USART_SendData(USART2, word);

    // clear 'Read data register not empty' flag
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  }
}

void USART2_IRQHandler(void) {
  uint16_t word;
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
    // the most recent received data by the USART1 peripheral
    word = USART_ReceiveData(USART2);
    Uart3_EnQueue(word);
    USART_SendData(USART1, word);

    // clear 'Read data register not empty' flag
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}

void Delay(void) {
  int i;

  for (i = 0; i < 2000000; i++){
  }
}

int main(void) {

  SystemInit();
  LED_Init();
  Bluetooth_Init();
  UltraSound_Init();

  while (1) {
    LEDTurnOnOff();

    if(Read_Distance() < 15){
      // TODO 정지!
    }

    if (0 == Bluetooth_Uart3_Is_Empty()) {
      data = Bluetooth_Uart3_DeQueue();
      printf("data: %d \n", data);
      USART_SendData(USART1, data);

      if (data == 0) { // TODO 후진
      } else if (data == 1) { // TODO 전진
      } else if (data == 2) { // TODO 좌회전
      } else if (data == 3) { // TODO 우회전
      } else if (data == 4) { // 왼쪽 방향 지시등
        LeftLED++;
        LeftLED %= 2;
      } else if (data == 5) { // 오른쪽 방향 지시등
        RightLED++;
        RightLED %= 2;
      } else if (data == 6) { // TODO 크락션
      }
    }
  }
  return 0;
}
