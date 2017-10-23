#include "stm32f4xx.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "sliding_median.h"
#include "engine.h"
#include "KPP.h"

#define LEFT     (ADCValue[0])
#define RIGHT    (ADCValue[1])
#define THROTTLE (ADCValue[2])
#define BRAKE    (ADCValue[3])
#define DECELER  (ADCValue[4])
#define TEMP     (ADCValue[5])

#define OT_LEFT  (ADCPWM[0])
#define OT_RIGHT (ADCPWM[1])
#define BF_LEFT  (ADCPWM[2])
#define BF_RIGHT (ADCPWM[3])
#define FIRST    (ADCPWM[4])
#define SECOND   (ADCPWM[5])
#define THIRD    (ADCPWM[6])
#define FORWARD  (ADCPWM[7])
#define REVERSE  (ADCPWM[8])

static uint16_t ADCValue[6] = {0};
static uint16_t ADCPWM[9]   = {0};
static uint32_t time_ms     = 0;

void MaxAllRccBusConfig(void);
void DMAforADCInit(void);
void ADCInputInit(void);
void CANInit(void);
void TIM_PWMInit(void);
void TimerInit(void);
void FlashInit(void);

Engine eng;
KPP    kpp;

void main()
{
  MaxAllRccBusConfig();
  FlashInit();
  
  GPIO_DeInit(GPIOA);//CAN1, ADC2ch1, ADC2ch4, ADC2ch5, ADC2ch7, TIM1-PWM
  GPIO_DeInit(GPIOB);//TIM4-PWM, CAN2, ADC2ch8
  GPIO_DeInit(GPIOC);//ADC3ch10, ADC3ch11, ADC2ch12-ADC2ch15, TIM3-PWM
  GPIO_DeInit(GPIOF);//ADC3ch4, ADC3ch6-ADC3ch8
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//CAN1, ADC2ch1, ADC2ch4, ADC2ch5, ADC2ch7, TIM1-PWM
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//TIM4-PWM, CAN2, ADC2ch8
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ADC3ch10, ADC3ch11, ADC2ch12-ADC2ch15, TIM3-PWM
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);//ADC3ch4, ADC3ch6-ADC3ch8
  
  CANInit();
  DMAforADCInit();
  ADCInputInit();
  TIM_PWMInit();
  TimerInit();
  
  while(true)
  {          
    __NOP();
  }
}

//Настраиваем тактирование ядра и переферии от HSE(8 МГц) на максимальные частоты.
void MaxAllRccBusConfig()
{
  SystemInit();
  SystemCoreClockUpdate();

  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  RCC_ClockSecuritySystemCmd(ENABLE);

  ErrorStatus State = RCC_WaitForHSEStartUp();
  
  if(State != SUCCESS)
    while(true) {}
  else
  {
    RCC_HCLKConfig(RCC_SYSCLK_Div1);//AHB clock = SYSCLK
    RCC_PCLK1Config(RCC_HCLK_Div4);//APB1 clock = HCLK/4
    RCC_PCLK2Config(RCC_HCLK_Div2);//APB2 clock = HCLK/2

    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);//8 MHz
    RCC_PLLCmd(ENABLE);

    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//SYSCLK clock = PLLCLK
    while(RCC_GetSYSCLKSource() != 8) {}
  }
}

void FlashInit()
{
  FLASH_PrefetchBufferCmd(ENABLE);
  FLASH_SetLatency(FLASH_Latency_5);
}

//Настраиваем модуль DMA2 для автоматической обработки всех каналов АЦП3 и АЦП2
void DMAforADCInit()
{
  DMA_DeInit(DMA2_Stream0);
  DMA_DeInit(DMA2_Stream2);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  DMA_InitTypeDef DMA_InitStruct;
  DMA_StructInit(&DMA_InitStruct);

  DMA_InitStruct.DMA_Channel            = DMA_Channel_2;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC3->DR);
  DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)ADCValue;
  DMA_InitStruct.DMA_BufferSize         = 6;
  DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_InitStruct.DMA_Mode               = DMA_Mode_Circular;
  DMA_InitStruct.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_Init(DMA2_Stream0, &DMA_InitStruct);
  
  DMA_InitStruct.DMA_Channel            = DMA_Channel_1;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC2->DR);
  DMA_InitStruct.DMA_Memory0BaseAddr    = (uint32_t)ADCPWM;
  DMA_InitStruct.DMA_BufferSize         = 9;
  DMA_Init(DMA2_Stream2, &DMA_InitStruct);
  
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);//если прерывание ненадо, можно ли убрать
  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);//если прерывание ненадо, можно ли убрать
  DMA_Cmd(DMA2_Stream0, ENABLE);
  DMA_Cmd(DMA2_Stream2, ENABLE);

  //Настройка прерывания, без него не работает!
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0x0;
  NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel                   = DMA2_Stream2_IRQn;
  NVIC_Init(&NVIC_InitStruct);
}

/******************************************************************************
Настраиваем АЦП3 каналы 4, 6 - 8, 10, 11 на преобразование по прерыванию таймера 2 ОС2. Чтобы прерывание срабатывало в 2 раза быстрее таймера 2, было настроенно прерывание по спаду и нарастанию сигнала(ADC_ExternalTrigConvEdge_RisingFalling). Это необходимо для улучшения отклика системы на управление потенциометром.
******************************************************************************/
void ADCInputInit()
{
  ADC_DeInit();
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);
  
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_Init(GPIOF, &GPIO_InitStruct);

  ADC_CommonInitTypeDef ADC_CommonInitStruct;
  ADC_CommonStructInit(&ADC_CommonInitStruct);
  ADC_CommonInit(&ADC_CommonInitStruct);

  ADC_InitTypeDef ADC_InitStruct;
  ADC_StructInit(&ADC_InitStruct);

  ADC_InitStruct.ADC_ScanConvMode         = ENABLE;
  ADC_InitStruct.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T2_CC2;
  ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_RisingFalling;
  ADC_InitStruct.ADC_NbrOfConversion      = 6;
  //ADC_InitStruct.ADC_ContinuousConvMode   = ENABLE;
  ADC_Init(ADC3, &ADC_InitStruct);
  
  ADC_InitStruct.ADC_NbrOfConversion      = 9;
  ADC_Init(ADC2, &ADC_InitStruct);

  ADC_RegularChannelConfig(ADC3, ADC_Channel_6,  1, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_7,  2, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_8,  3, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 4, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 5, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_4,  6, ADC_SampleTime_480Cycles);
  
  ADC_RegularChannelConfig(ADC2, ADC_Channel_1,  1, ADC_SampleTime_480Cycles);//OT left
  ADC_RegularChannelConfig(ADC2, ADC_Channel_4,  2, ADC_SampleTime_480Cycles);//OT right
  ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 3, ADC_SampleTime_480Cycles);//BF left
  ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 4, ADC_SampleTime_480Cycles);//BF right
  ADC_RegularChannelConfig(ADC2, ADC_Channel_8,  5, ADC_SampleTime_480Cycles);//1
  ADC_RegularChannelConfig(ADC2, ADC_Channel_7,  6, ADC_SampleTime_480Cycles);//2
  ADC_RegularChannelConfig(ADC2, ADC_Channel_5,  7, ADC_SampleTime_480Cycles);//3
  ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 8, ADC_SampleTime_480Cycles);//Forward
  ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 9, ADC_SampleTime_480Cycles);//Reverse

  //Запрос после последней передачи, без него не работает
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
  ADC_DMACmd(ADC3, ENABLE);
  ADC_DMACmd(ADC2, ENABLE);
  //ADC_ContinuousModeCmd(ADC3, ENABLE);
  ADC_Cmd(ADC3, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  ADC_SoftwareStartConv(ADC3);
  ADC_SoftwareStartConv(ADC2);
}

/******************************************************************************
Настраиваем таймер 2 и 7 для прерывания ADC и подсчета времени. Увеличена скорость прерывания канала по средствам установки реакции по спадающему и нарастающему фронту сигнала одновременно(ADC_ExternalTrigConvEdge_RisingFalling) для обработки с помощью DMA каналов АЦП и чтобы за время прерывания таймера 2, успел заполниться класс скользящей медианы.
******************************************************************************/
void TimerInit()
{
  TIM_DeInit(TIM2);
  TIM_DeInit(TIM7);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 840;//Mgz*10
  TIM_TimeBaseInitStruct.TIM_Period = 2000;// 20 мс
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
  
  TIM_TimeBaseInitStruct.TIM_Prescaler = 840;//Mgz*10
  TIM_TimeBaseInitStruct.TIM_Period = 100;// 1 мс
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruct);

  //Настройка канала ОС2 необходима чтобы сканирование АЦП происходило по прерыванию. Так же ненужна настройка самой ноги GPIOх
  TIM_SetCounter(TIM2, 0);
  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse = 1000;//в 2 раза быстрее чем таймер 2
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OC2Init(TIM2, &TIM_OCInitStruct);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);//разрешаем прерывание по переполнению
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);//разрешаем прерывание по переполнению
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM7_IRQn);
  
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM7, ENABLE);
}

void CANInit()
{
  CAN_DeInit(CAN1);
  CAN_DeInit(CAN2);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,  GPIO_AF_CAN2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

  //частота шины 42МГц / 24 = 1.75МГц / 7 = 250 кбит/с
  CAN_InitTypeDef CAN_InitStruct;
  CAN_StructInit(&CAN_InitStruct);

  CAN_InitStruct.CAN_ABOM = ENABLE;
  CAN_InitStruct.CAN_TXFP = ENABLE;
  CAN_InitStruct.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStruct.CAN_Prescaler = 24;
  CAN_Init(CAN1, &CAN_InitStruct);
  CAN_Init(CAN2, &CAN_InitStruct);

  CAN_FilterInitTypeDef CAN_FilterInitStruct;//без настройки фильтра не работает прием
  CAN_SlaveStartBank(0);//необходимо так же задать номер фильтра, без него не принимает
  CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x0000 << 5;
  CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0000 << 5;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x0000 << 5;
  CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0000 << 5;
  //CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdList;
  CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_16bit;
  //CAN_FilterInitStruct.CAN_FilterIdHigh         = 0x18FEEF00 >> 13;
  //CAN_FilterInitStruct.CAN_FilterIdLow          = 0x0FFFF & (0xEF000 >> 1);
  //CAN_FilterInitStruct.CAN_FilterMaskIdHigh     = 0x1FFFFF00 >> 13;
  //CAN_FilterInitStruct.CAN_FilterMaskIdLow      = 0x0FFFF & (0xFFFF0 >> 1);
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
  CAN_FilterInitStruct.CAN_FilterNumber         = 0;
  CAN_FilterInitStruct.CAN_FilterMode           = CAN_FilterMode_IdMask;
  //CAN_FilterInitStruct.CAN_FilterScale          = CAN_FilterScale_32bit;
  CAN_FilterInitStruct.CAN_FilterActivation     = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStruct);
  

  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
  NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}

void TIM_PWMInit()
{
  TIM_DeInit(TIM1);
  TIM_DeInit(TIM3);
  TIM_DeInit(TIM4);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
  
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;//Максимальная скорость для работы ШИМ
  GPIO_Init(GPIOA, &GPIO_InitStructure);//TIM1
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//TIM3
  GPIO_Init(GPIOB, &GPIO_InitStructure);//TIM4

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 840;
  TIM_TimeBaseInitStruct.TIM_Period    = 500;//200 Gz
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 1680;//200 Gz для TIM1
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCStructInit(&TIM_OCInitStruct);
  TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse       = 0;
  TIM_OC1Init(TIM1, &TIM_OCInitStruct);
  TIM_OC2Init(TIM1, &TIM_OCInitStruct);
  
  TIM_OC1Init(TIM3, &TIM_OCInitStruct);
  TIM_OC2Init(TIM3, &TIM_OCInitStruct);
  TIM_OC3Init(TIM3, &TIM_OCInitStruct);
  TIM_OC4Init(TIM3, &TIM_OCInitStruct);
  
  TIM_OC1Init(TIM4, &TIM_OCInitStruct);
  TIM_OC2Init(TIM4, &TIM_OCInitStruct);
  TIM_OC3Init(TIM4, &TIM_OCInitStruct);
  TIM_OC4Init(TIM4, &TIM_OCInitStruct);

  //TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);//разрешает загрузку в регистр CCR1
  //TIM_ARRPreloadConfig(TIM4, ENABLE);//разрешает предварительную загрузку в регистр ARR

  TIM_Cmd(TIM1, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
  
  TIM_CtrlPWMOutputs(TIM1, ENABLE);//Необходимо для таймера 1, т.к. у него есть регистр BDTR

  //TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);//разрешаем прерывание по переполнению
  //NVIC_EnableIRQ(TIM4_IRQn);
}

extern "C"
{
  /*Без обработчика прерывания не работает DMA. Обработчик прерывания привязан ко второму каналу таймера 2. Закидываем принятые значения(РУД, Тормоз, Диселератор, Влево, Вправо, Температура) с АЦП в фильтр скользящей медианы.*/
  void DMA2_Stream0_IRQHandler()
  {
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
      DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
      kpp.AnalogSet(ADCValue);
    }
  }
  void DMA2_Stream2_IRQHandler()
  {
    if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
    {
      DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
    }
  }
  
  void TIM2_IRQHandler()//Прерывание по таймеру 2 - 20 мс
  {
    if(TIM_GetITStatus(TIM2, TIM_IT_Update))
    {
      TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
  }
  
  //Прерывание по TIM7-1 мс. Ведем отсчет времени, шлем в CAN аналоговые и дискретные сигналы.
  void TIM7_IRQHandler()
  {
    if(TIM_GetITStatus(TIM7, TIM_IT_Update))
    {
      TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
      ++time_ms;
      
      if(!(time_ms % 10))
      {
        //управление клапанами в пропорциональном режиме
      }
      if(!(time_ms % 50))
      {
        CanTxMsg TxMessage;
        TxMessage.RTR   = CAN_RTR_DATA;
        TxMessage.IDE   = CAN_ID_STD;
        TxMessage.StdId   = 0x001;
        TxMessage.DLC     = 8;
        TxMessage.Data[0] = (uint8_t)(time_ms);
        TxMessage.Data[1] = (uint8_t)(time_ms >> 8);
        TxMessage.Data[2] = (uint8_t)(time_ms >> 16);
        TxMessage.Data[3] = (uint8_t)(time_ms >> 24);
        TxMessage.Data[4] = OT_LEFT  * 100 / 4095;
        TxMessage.Data[5] = OT_RIGHT * 100 / 4095;
        TxMessage.Data[6] = BF_LEFT  * 100 / 4095;
        TxMessage.Data[7] = BF_RIGHT * 100 / 4095;
        CAN_Transmit(CAN2, &TxMessage);
        kpp.SendMsg();
      }
      if(!(time_ms % 99))
      {
        kpp.Parking(eng.GetRpm());
        kpp.SetClutch(eng.GetRpm());
        kpp.SwitchDirection(eng);
        kpp.BrakeRotate();
      }
    }
  }
  
  //Принимает от контроллера АСУ2.0 дискретные сигналы с органов управления, также обрабатываем сообщение с ДВС об оборотах.
  void CAN2_RX0_IRQHandler()
  {
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0))
    {
      CanRxMsg RxMessage;
      CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
      CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);
      
      if(RxMessage.IDE == CAN_ID_STD)
        switch(RxMessage.StdId)
        {
        case 0x004:
          kpp.DigitalSet((RxMessage.Data[1] << 8) | RxMessage.Data[0]);
          break;
        case 0x111:
          kpp.Calibrate(RxMessage);
          break;
        }
      else
        switch(RxMessage.ExtId)
        {
        case 0x0CF00400:
          eng.SetRpm((RxMessage.Data[4] * 256 + RxMessage.Data[3]) / 8);
          break;
        }
    }
  }
}