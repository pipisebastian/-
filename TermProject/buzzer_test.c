#include <stdbool.h>
#include "stm32f4xx.h"

#define BUZZER_RCC_AHB1Periph_GPIO          RCC_AHB1Periph_GPIOF
#define BUZZER_GPIO                         GPIOB
#define BUZZER_GPIO_Pin                     GPIO_Pin_0

#define BUZZER_GPIO_PinSource               GPIO_PinSource8
#define BUZZER_GPIO_AF_TIM                  GPIO_AF_TIM13

#define BUZZER_RCC_APB1Periph_TIM           RCC_APB1Periph_TIM13
#define BUZZER_TIM                          TIM13

#define BUZZER_TIM_Prescaler                (8399)
#define BUZZER_TIM_Period                   (9999)

#define BUZZER_TIM_OCPolarity               TIM_OCPolarity_High

#define BUZZER_TIM_OCInit                   TIM_OC1Init
#define BUZZER_TIM_OCPreloadConfig          TIM_OC1PreloadConfig

#define BUZZER_TIM_SetCompare               TIM_SetCompare1

#define BUZZER_TIM_IRQn                     TIM8_UP_TIM13_IRQn
#define BUZZER_TIM_IRQHandler               TIM8_UP_TIM13_IRQHandler
#define BUZZER_TIM_IRQ_PreemptionPriority   (0)
#define BUZZER_TIM_IRQ_SubPriority          (0)

typedef enum
{
  BuzzerStateOff     = 0,
  BuzzerStateRing    = 1,
  BuzzerStateDrip    = 2,
  BuzzerStateDidi    = 3,
  BuzzerStateDidiDi  = 4,
  BuzzerStateWarning = 5,
  BuzzerStateDanger  = 6
}BuzzerState;


static __IO uint8_t     buzzerCount = 0;
static __IO BuzzerState buzzerState = BuzzerStateOff;

static void Buzzer_Init(void);
static void Buzzer_PwmInit(void);
static void Buzzer_NvicInit(void);
static void Buzzer_Off(void);
static void Buzzer_Ring(void);
static void Buzzer_Drip(void);
static void Buzzer_Didi(void);
static void Buzzer_DidiDi(void);
static void Buzzer_Warning(void);
static void Buzzer_Danger(void);

void Delay_us(uint64_t nus);
void Delay_ms(uint64_t nms);
void Delay_s(uint64_t ns);

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

static void Buzzer_Init(void)
{
  static bool init_flag = false;
  
  if(init_flag == false)
  {
    init_flag = true;
    
    Buzzer_PwmInit();
    Buzzer_NvicInit();
  }
}

static void Buzzer_PwmInit(void)
{
  GPIO_InitTypeDef        GPIO_InitStructure    = {0};
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
  TIM_OCInitTypeDef       TIM_OCInitStructure   = {0};
  
  RCC_AHB1PeriphClockCmd(BUZZER_RCC_AHB1Periph_GPIO, ENABLE);
  RCC_APB1PeriphClockCmd(BUZZER_RCC_APB1Periph_TIM, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin   = BUZZER_GPIO_Pin;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(BUZZER_GPIO, &GPIO_InitStructure);
  
  // GPIO_PinAFConfig(BUZZER_GPIO, BUZZER_GPIO_PinSource, BUZZER_GPIO_AF_TIM);
  
  TIM_TimeBaseStructure.TIM_Prescaler     = BUZZER_TIM_Prescaler;
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period        = BUZZER_TIM_Period;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(BUZZER_TIM, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse       = 0;
  TIM_OCInitStructure.TIM_OCPolarity  = BUZZER_TIM_OCPolarity;
  BUZZER_TIM_OCInit(BUZZER_TIM, &TIM_OCInitStructure);
  
  BUZZER_TIM_OCPreloadConfig(BUZZER_TIM, TIM_OCPreload_Enable);
  
  TIM_ARRPreloadConfig(BUZZER_TIM, ENABLE);
  
  TIM_Cmd(BUZZER_TIM, ENABLE);
}

static void Buzzer_NvicInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure = {0};
  
  NVIC_InitStructure.NVIC_IRQChannel                   = BUZZER_TIM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BUZZER_TIM_IRQ_PreemptionPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = BUZZER_TIM_IRQ_SubPriority;
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

static void Buzzer_Off(void)
{
  Buzzer_Init();
  
  BUZZER_TIM_SetCompare(BUZZER_TIM, 0);
  
  buzzerState = BuzzerStateOff;
}

static void Buzzer_Ring(void)
{
  Buzzer_Init();
  
  TIM_SetAutoreload(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 2 - 1);
  BUZZER_TIM_SetCompare(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 2);
  
  buzzerState = BuzzerStateRing;
}

static void Buzzer_Drip(void)
{
  Buzzer_Init();
  
  TIM_SetAutoreload(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 2 - 1);
  BUZZER_TIM_SetCompare(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 4);
  
  TIM_ClearFlag(BUZZER_TIM, TIM_FLAG_Update);
  TIM_ITConfig(BUZZER_TIM, TIM_IT_Update, ENABLE);
  
  buzzerCount = 0;
  buzzerState = BuzzerStateDrip;
}

static void Buzzer_Didi(void)
{
  Buzzer_Init();
  
  TIM_SetAutoreload(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 4 - 1);
  BUZZER_TIM_SetCompare(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 8);
  
  TIM_ClearFlag(BUZZER_TIM, TIM_FLAG_Update);
  TIM_ITConfig(BUZZER_TIM, TIM_IT_Update, ENABLE);
  
  buzzerCount = 1;
  buzzerState = BuzzerStateDidi;
}

static void Buzzer_DidiDi(void)
{
  Buzzer_Init();
  
  TIM_SetAutoreload(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 8 - 1);
  BUZZER_TIM_SetCompare(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 16);
  
  TIM_ClearFlag(BUZZER_TIM, TIM_FLAG_Update);
  TIM_ITConfig(BUZZER_TIM, TIM_IT_Update, ENABLE);
  
  buzzerCount = 2;
  buzzerState = BuzzerStateDidiDi;
}

static void Buzzer_Warning(void)
{
  Buzzer_Init();
  
  TIM_SetAutoreload(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 2 - 1);
  BUZZER_TIM_SetCompare(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 4);
  
  buzzerState = BuzzerStateWarning;
}

static void Buzzer_Danger(void)
{
  Buzzer_Init();
  
  TIM_SetAutoreload(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 4 - 1);
  BUZZER_TIM_SetCompare(BUZZER_TIM, (BUZZER_TIM_Period + 1) / 8);
  
  buzzerState = BuzzerStateDanger;
}

void Buzzer_SetState(BuzzerState state)
{
  if(state != buzzerState)
  {
    if(state == BuzzerStateOff)
    {
      Buzzer_Off();
    }
    else if(state == BuzzerStateRing)
    {
      Buzzer_Ring();
    }
    else if(state == BuzzerStateDrip)
    {
      Buzzer_Drip();
    }
    else if(state == BuzzerStateDidi)
    {
      Buzzer_Didi();
    }
    else if(state == BuzzerStateDidiDi)
    {
      Buzzer_DidiDi();
    }
    else if(state == BuzzerStateWarning)
    {
      Buzzer_Warning();
    }
    else if(state == BuzzerStateDanger)
    {
      Buzzer_Danger();
    }
  }
}

BuzzerState Buzzer_GetState(void)
{
  return buzzerState;
}

void BUZZER_TIM_IRQHandler(void)
{
  if(TIM_GetITStatus(BUZZER_TIM, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(BUZZER_TIM, TIM_IT_Update);
    
    if((buzzerState == BuzzerStateDrip) || (buzzerState == BuzzerStateDidi) || (buzzerState == BuzzerStateDidiDi))
    {
      if(buzzerCount > 0)
      {
        buzzerCount--;
      }
      else
      {
        TIM_ITConfig(BUZZER_TIM, TIM_IT_Update, DISABLE);
        Buzzer_Off();
      }
    }
    else
    {
      TIM_ITConfig(BUZZER_TIM, TIM_IT_Update, DISABLE);
    }
  }
}

void Delay_us(uint64_t nus)
{
  uint64_t nms = 0;
  
  if(nus == 0)
  {
    return;
  }
  
  nms = nus / 1000;
  nus = nus % 1000;
  
  if(nms > 0)
  {
    Delay_ms(nms);
  }
  
  if(nus > 0)
  {
    RCC_ClocksTypeDef RCC_ClockFreq;
    
    RCC_GetClocksFreq(&RCC_ClockFreq);                              /* Get the frequencies of different on chip clocks. */
    
    if(RCC_ClockFreq.HCLK_Frequency < 8000000)
    {
      SysTick->CTRL |= SysTick_CLKSource_HCLK;                      /* Configures the SysTick clock source. */
      SysTick->LOAD = RCC_ClockFreq.HCLK_Frequency / 1000000 * nus; /* Time load (SysTick-> LOAD is 24bit). */
    }
    else
    {
      SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;                 /* Configures the SysTick clock source. */
      SysTick->LOAD = RCC_ClockFreq.HCLK_Frequency / 8000000 * nus; /* Time load (SysTick-> LOAD is 24bit). */
    }
    
    SysTick->VAL = 0;                                               /* Empty counter. */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;                       /* Start the countdown. */
    
    while((SysTick->CTRL&(1UL<<16)) != (1UL<<16));                  /* Wait time is reached. */
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;                      /* Close counter. */
  }
}

void Delay_ms(uint64_t nms)
{
  if(nms == 0)
  {
    return;
  }
  
  while(nms > 500)
  {
    RCC_ClocksTypeDef RCC_ClockFreq;
    
    RCC_GetClocksFreq(&RCC_ClockFreq);                            /* Get the frequencies of different on chip clocks. */
    
    if(RCC_ClockFreq.HCLK_Frequency < 8000000)
    {
      SysTick->CTRL |= SysTick_CLKSource_HCLK;                    /* Configures the SysTick clock source. */
      SysTick->LOAD = RCC_ClockFreq.HCLK_Frequency / 1000 * 500;  /* Time load (SysTick-> LOAD is 24bit). */
    }
    else
    {
      SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;               /* Configures the SysTick clock source. */
      SysTick->LOAD = RCC_ClockFreq.HCLK_Frequency / 8000 * 500;  /* Time load (SysTick-> LOAD is 24bit). */
    }
    
    SysTick->VAL = 0;                                             /* Empty counter. */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;                     /* Start the countdown. */
    
    while((SysTick->CTRL&(1UL<<16)) != (1UL<<16));                /* Wait time is reached. */
    
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;                    /* Close counter. */
    
    nms -= 500;
  }
  
  RCC_ClocksTypeDef RCC_ClockFreq;
  
  RCC_GetClocksFreq(&RCC_ClockFreq);                              /* Get the frequencies of different on chip clocks. */
  
  if(RCC_ClockFreq.HCLK_Frequency < 8000000)
  {
    SysTick->CTRL |= SysTick_CLKSource_HCLK;                      /* Configures the SysTick clock source. */
    SysTick->LOAD = RCC_ClockFreq.HCLK_Frequency / 1000 * nms;    /* Time load (SysTick-> LOAD is 24bit). */
  }
  else
  {
    SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;                 /* Configures the SysTick clock source. */
    SysTick->LOAD = RCC_ClockFreq.HCLK_Frequency / 8000 * nms;    /* Time load (SysTick-> LOAD is 24bit). */
  }
  
  SysTick->VAL = 0;                                               /* Empty counter. */
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;                       /* Start the countdown. */
  
  while((SysTick->CTRL&(1UL<<16)) != (1UL<<16));                  /* Wait time is reached. */
  
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;                      /* Close counter. */
}

void Delay_s(uint64_t ns)
{
  while(ns > 0)
  {
    Delay_ms(1000);
    ns--;
  }
}

int main(void)
{
  for(;;)
  {
    Delay_s(5);
    Buzzer_SetState(BuzzerStateDrip);
    Delay_s(5);
    Buzzer_SetState(BuzzerStateDidi);
    Delay_s(5);
    Buzzer_SetState(BuzzerStateDidiDi);
    Delay_s(5);
    Buzzer_SetState(BuzzerStateWarning);
    Delay_s(5);
    Buzzer_SetState(BuzzerStateDanger);
    Delay_s(5);
    Buzzer_SetState(BuzzerStateRing);
    Delay_s(5);
    Buzzer_SetState(BuzzerStateOff);
  }
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}
