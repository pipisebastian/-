#include "stm32f10x.h"
#include "stdio.h"


#define  U3_BUFFER_SIZE  100

uint16_t  u3_rx_buffer[U3_BUFFER_SIZE];

uint32_t  u3_rx_point_head = 0;
uint32_t  u3_rx_point_tail = 0;

void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void USART1_Init(void);
void NVIC_Configure(void);

void EXTI15_10_IRQHandler(void);

void Delay(void);

void sendDataUART1(uint16_t data);
void u3_increase_point_value(uint32_t * data_p);

void Uart3_EnQueue(uint16_t data)
{
    u3_rx_buffer[u3_rx_point_head] = data;
    u3_increase_point_value(&u3_rx_point_head);
}

void u3_increase_point_value(uint32_t * data_p)
{
    (* data_p) ++;
    if(U3_BUFFER_SIZE == (* data_p))
    {
        (* data_p) = 0;
    }
}

uint16_t Uart3_DeQueue(void)
{
    uint16_t retVal = u3_rx_buffer[u3_rx_point_tail];
    u3_increase_point_value(&u3_rx_point_tail);
    return retVal;
}

int Uart3_Is_Empty(void)
{
    if(u3_rx_point_head == u3_rx_point_tail)
    {
        return 1;
    }
    return 0;
}

void RCC_Configure(void)
{
    /* UART TX/RX port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* USART1 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

    /* Alternate Function IO clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* UART2 pin setting -> bluetooth */
    //TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;

    // Enable the USART2 peripheral
    USART_Cmd(USART2, ENABLE);
    
    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx ;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART2_InitStructure);
    
    USART_ITConfig(USART2, USART_IT_RXNE,ENABLE);
}

void NVIC_Configure(void) {

    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    
    // UART2
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void USART2_IRQHandler(void) {
    uint16_t sensor_data[3] = {0}; //가속도 센서 값 3개 input
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
        // the most recent received data by the USART1 peripheral
        Uart3_EnQueue(USART_ReceiveData(USART2)); //<-sensor_data의 데이터 형식 선택        
        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

void Delay(void) {
    int i;
    for (i = 0; i < 2000000; i++) {
        
    }
}

int main(void)
{
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    USART2_Init();
    NVIC_Configure();
    
    while(1){
        if(0 == Uart3_Is_Empty()){
            uint16_t data = Uart3_DeQueue();
            printf("data: %d", data);
        }
    }
    return 0;
}
