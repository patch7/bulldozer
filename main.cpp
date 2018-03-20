#include "stm32f4xx.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "sliding_median.h"
#include "KPP.h"
#include "torque converter.h"

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
#define LOCK_V   (ADCPWM[9])
#define BRAKE_V  (ADCPWM[10])

const uint16_t DEFAULT = 0x010;

static uint16_t ADCValue[6] = {0};
static uint16_t ADCPWM[11]  = {0};
static uint32_t time_ms     = 0;
static uint32_t timeout     = 0;

void MaxAllRccBusConfig(void);
void DMAforADCInit(void);
void ADCInputInit(void);
void CANInit(void);
void TIM_PWMInit(void);
void TimerInit(void);
void FlashInit(void);

Pressure pres;
Calibrate::State state = Calibrate::Not;
Calibrate cal(9);
KPP       kpp;
TC        tc;

void main()
{
  MaxAllRccBusConfig();
  FlashInit();
  
  GPIO_DeInit(GPIOA);//CAN1, ADC2(ch1, ch3 - ch7), TIM1(ch1 - ch3)
  GPIO_DeInit(GPIOB);//CAN2, ADC2ch8, TIM2ch4, TIM4(ch1 - ch4)
  GPIO_DeInit(GPIOC);//ADC3(ch10, ch11), ADC2(ch12 - ch15), TIM3(ch1 - ch4)
  GPIO_DeInit(GPIOF);//ADC3(ch4, ch6 - ch8)
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  
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
  DMA_InitStruct.DMA_BufferSize         = 11;
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
  //                             ADC2ch1    ADC2ch3    ADC2ch4    ADC2ch5    ADC2ch6    ADC2ch7
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  //                             ADC2ch8
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  //                            ADC3ch10   ADC3ch11   ADC2ch12   ADC2ch13   ADC2ch14   ADC2ch15
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
  //                             ADC3ch4      ADC3ch6      ADC3ch7      ADC3ch8
  GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_Init(GPIOF, &GPIO_InitStruct);

  ADC_CommonInitTypeDef ADC_CommonInitStruct;
  ADC_CommonStructInit(&ADC_CommonInitStruct);
  ADC_CommonInit(&ADC_CommonInitStruct);

  ADC_InitTypeDef ADC_InitStruct;
  ADC_StructInit(&ADC_InitStruct);

  ADC_InitStruct.ADC_ScanConvMode         = ENABLE;
  ADC_InitStruct.ADC_ExternalTrigConv     = ADC_ExternalTrigConv_T5_CC1;
  ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_RisingFalling;
  ADC_InitStruct.ADC_NbrOfConversion      = 6;
  //ADC_InitStruct.ADC_ContinuousConvMode   = ENABLE;
  ADC_Init(ADC3, &ADC_InitStruct);
  
  ADC_InitStruct.ADC_NbrOfConversion      = 11;
  ADC_Init(ADC2, &ADC_InitStruct);

  ADC_RegularChannelConfig(ADC3, ADC_Channel_6,  1, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_7,  2, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_8,  3, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 4, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 5, ADC_SampleTime_480Cycles);
  ADC_RegularChannelConfig(ADC3, ADC_Channel_4,  6, ADC_SampleTime_480Cycles);
  
  ADC_RegularChannelConfig(ADC2, ADC_Channel_1,  1,  ADC_SampleTime_480Cycles);//OT left
  ADC_RegularChannelConfig(ADC2, ADC_Channel_4,  2,  ADC_SampleTime_480Cycles);//OT right
  ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 3,  ADC_SampleTime_480Cycles);//BF left
  ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 4,  ADC_SampleTime_480Cycles);//BF right
  ADC_RegularChannelConfig(ADC2, ADC_Channel_8,  5,  ADC_SampleTime_480Cycles);//1
  ADC_RegularChannelConfig(ADC2, ADC_Channel_7,  6,  ADC_SampleTime_480Cycles);//2
  ADC_RegularChannelConfig(ADC2, ADC_Channel_5,  7,  ADC_SampleTime_480Cycles);//3
  ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 8,  ADC_SampleTime_480Cycles);//Forward
  ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 9,  ADC_SampleTime_480Cycles);//Reverse
  ADC_RegularChannelConfig(ADC2, ADC_Channel_6,  10, ADC_SampleTime_480Cycles);//Lock valve
  ADC_RegularChannelConfig(ADC2, ADC_Channel_3,  11, ADC_SampleTime_480Cycles);//Reactor brake

  //Запрос после последней передачи, без него не работает
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
  ADC_DMACmd(ADC3, ENABLE);
  ADC_DMACmd(ADC2, ENABLE);
  //ADC_ContinuousModeCmd(ADC3, ENABLE);
  //ADC_ContinuousModeCmd(ADC2, ENABLE);
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
  TIM_DeInit(TIM7);
  TIM_DeInit(TIM5);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 839;//всегда +1, Mgz*10
  TIM_TimeBaseInitStruct.TIM_Period = 200;// 2 мс
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);

  TIM_TimeBaseInitStruct.TIM_Period = 100;// 1 мс
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStruct);

  //Настройка канала ОС1 необходима чтобы сканирование АЦП происходило по прерыванию. Так же ненужна настройка самой ноги GPIOх
  TIM_SetCounter(TIM5, 0);
  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCStructInit(&TIM_OCInitStruct);
  TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse = 100;//в 2 раза быстрее чем таймер 8
  TIM_OC1Init(TIM5, &TIM_OCInitStruct);

  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);//разрешаем прерывание по переполнению
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);//разрешаем прерывание по переполнению
  NVIC_EnableIRQ(TIM5_IRQn);
  NVIC_EnableIRQ(TIM7_IRQn);
  
  TIM_Cmd(TIM5, ENABLE);
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
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel                   = CAN2_RX1_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel                   = CAN2_TX_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 2;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel                   = CAN2_SCE_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 3;
  NVIC_Init(&NVIC_InitStruct);

  CAN_ITConfig(CAN2, CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_TME | CAN_IT_EWG | CAN_IT_EPV | CAN_IT_LEC | CAN_IT_ERR, ENABLE);
}
void TIM_PWMInit()
{
  TIM_DeInit(TIM1);
  TIM_DeInit(TIM2);
  TIM_DeInit(TIM3);
  TIM_DeInit(TIM4);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);
  
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
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;//Максимальная скорость для работы ШИМ
  GPIO_Init(GPIOA, &GPIO_InitStructure);//TIM1

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);//TIM2

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//TIM3
  GPIO_Init(GPIOB, &GPIO_InitStructure);//TIM4

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 1679;//всегда +1, TIM1 - 168GHz
  TIM_TimeBaseInitStruct.TIM_Period    = 500;//200 Hz
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

  TIM_TimeBaseInitStruct.TIM_Prescaler = 839;//всегда +1
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCStructInit(&TIM_OCInitStruct);
  TIM_OCInitStruct.TIM_OCMode      = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse       = 0;
  TIM_OC2Init(TIM1, &TIM_OCInitStruct);
  TIM_OC3Init(TIM1, &TIM_OCInitStruct);

  TIM_OC4Init(TIM2, &TIM_OCInitStruct);
  
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
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
  
  TIM_CtrlPWMOutputs(TIM1, ENABLE);//Необходимо для таймера 1, т.к. у него есть регистр BDTR
}

extern "C"
{
  /*Без обработчика прерывания не работает DMA. Обработчик прерывания привязан ко второму каналу таймера 2. Заполняем значения аналоговых органов управления с АЦП в фильтр скользящей медианы.*/
  void DMA2_Stream0_IRQHandler()
  {
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
      DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
      kpp.AnalogSet(ADCValue, cal);
    }
  }
  //Заполняем значения токов на клапанах с АЦП в фильтр скользящей медианы.
  void DMA2_Stream2_IRQHandler()
  {
    if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
    {
      DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
      kpp.CurrentSet(ADCPWM, cal);
    }
  }
  void TIM5_IRQHandler()
  {
    if(TIM_GetITStatus(TIM5, TIM_IT_Update))
      TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
  }
  //Прерывание по TIM7-1 мс. Ведем отсчет времени, шлем в CAN аналоговые и дискретные сигналы.
  void TIM7_IRQHandler()
  {
    if(TIM_GetITStatus(TIM7, TIM_IT_Update))
    {
      TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
      ++time_ms;

      kpp.GraphSetFR();//управление клапанами в пропорциональном режиме

      if(time_ms - timeout > 300)//при аварийной ситуации(обрыв кан АСУ2.0) все сигналы default
        kpp.DigitalSet(DEFAULT, cal);
      
      if(!(time_ms % 10))
        if(state != Calibrate::Not)
          cal.Valve(state, pres);//Калибровка клапана!
      if(!(time_ms % 24))
        kpp.Send(cal);
      if(!(time_ms % 99))
      {
        kpp.Parking(cal);
        kpp.SetClutch(cal);
        kpp.SwitchDirection(cal);
        kpp.BrakeRotate(cal);
        kpp.RequestRpm(cal);

        if(cal.Parking_Is_On())
          tc.reset();//очень часто вызывается
        else if(cal.Filter_Is_On())
          tc.lock();
        else
          tc.unlock();
      }
    }
  }
  //Принимает от контроллера АСУ2.0 дискретные сигналы с органов управления, также обрабатываем сообщение с ДВС об оборотах.
  void CAN2_RX0_IRQHandler()
  {
    if(CAN_GetITStatus(CAN2, CAN_IT_FMP0))
    {
      CanRxMsg RxMsg;
      CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
      CAN_Receive(CAN2, CAN_FIFO0, &RxMsg);
      CAN_FIFORelease(CAN2, CAN_FIFO0);
      
      if(RxMsg.IDE == CAN_ID_STD)
        switch(RxMsg.StdId)
        {
          case 0x005:
            kpp.DigitalSet(RxMsg.Data[4] << 8 | RxMsg.Data[0], cal);
            timeout = time_ms;                                                            break;
          case 0x010: cal.RemoteCtrlAndRPM(RxMsg.Data[0],RxMsg.Data[2]<<8|RxMsg.Data[1]); break;
          case 0x100: cal.OtLeftTime(RxMsg);                                              break;
          case 0x101: cal.OtLeftPres(RxMsg);                                              break;
          case 0x102: cal.OtRightTime(RxMsg);                                             break;
          case 0x103: cal.OtRightPres(RxMsg);                                             break;
          case 0x104: cal.BfLeftTime(RxMsg);                                              break;
          case 0x105: cal.BfLeftPres(RxMsg);                                              break;
          case 0x106: cal.BfRightTime(RxMsg);                                             break;
          case 0x107: cal.BfRightPres(RxMsg);                                             break;
          case 0x108: cal.ForwardTime(RxMsg);                                             break;
          case 0x109: cal.ForwardPres(RxMsg);                                             break;
          case 0x10A: cal.ReverseTime(RxMsg);                                             break;
          case 0x10B: cal.ReversePres(RxMsg);                                             break;
          case 0x10C: cal.OneTime(RxMsg);                                                 break;
          case 0x10D: cal.OnePres(RxMsg);                                                 break;
          case 0x10E: cal.TwoTime(RxMsg);                                                 break;
          case 0x10F: cal.TwoPres(RxMsg);                                                 break;
          case 0x110: cal.ThreeTime(RxMsg);                                               break;
          case 0x111: cal.ThreePres(RxMsg);                                               break;
          case 0x112: cal.Save();                                                         break;
          case 0x113: kpp.SendData(cal);                                                  break;
          case 0x114: kpp.SendDataValve(cal);                                             break;
          case 0x120: state = static_cast<Calibrate::State>(RxMsg.Data[0]);               break;
          case 0x181:
            pres.i = RxMsg.Data[3] << 24 | RxMsg.Data[2] << 16 | RxMsg.Data[1] << 8 |RxMsg.Data[0];
            if(pres.f < 0)
              pres.f = 0;
            break;
        }
      else
        switch(RxMsg.ExtId)
        {
          case 0x0CF00400: kpp.SetRpm(RxMsg.Data[4] * 256 + RxMsg.Data[3]); break;
        }
    }
  }
  void CAN2_RX1_IRQHandler()
  {
    if(CAN_GetITStatus(CAN2, CAN_IT_FMP1))
    {
      CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
      CAN_FIFORelease(CAN2, CAN_FIFO1);
    }
  }
  void CAN2_TX_IRQHandler()
  {
    if(CAN_GetITStatus(CAN2, CAN_IT_TME))
      CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
  }
  void CAN2_SCE_IRQHandler()
  {
    if(CAN_GetITStatus(CAN2, CAN_IT_ERR))
      CAN_ClearITPendingBit(CAN2, CAN_IT_ERR);
    if(CAN_GetITStatus(CAN2, CAN_IT_LEC))
      CAN_ClearITPendingBit(CAN2, CAN_IT_LEC);
    if(CAN_GetITStatus(CAN2, CAN_IT_EPV))
      CAN_ClearITPendingBit(CAN2, CAN_IT_EPV);
    if(CAN_GetITStatus(CAN2, CAN_IT_EWG))
      CAN_ClearITPendingBit(CAN2, CAN_IT_EWG);
    if(CAN_GetITStatus(CAN2, CAN_IT_BOF))
      CAN_ClearITPendingBit(CAN2, CAN_IT_BOF);
  }
}