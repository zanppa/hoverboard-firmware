// Current measurement in SVM mode

#include "stm32f1xx_hal.h"

static ADC_HandleTypeDef hadc1;

uint16_t rdson_meas[4];

// ADC1 init function. ADC1 is used to measure motor currents from lower switch Rds,on
void ADC1_init(void) {
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance                   = ADC1;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  // TODO: Could use timer 8 update (or underflow?) event as the source so software would not need to trigger ADC
  //hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T8_TRGO;	// Trigger ADC on timer8 update event (right timer)
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 4;	// 2 currents for both motors
  HAL_ADC_Init(&hadc1);

  // Use a rather short sampling time...
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;

  sConfig.Channel = ADC_CHANNEL_0;  // pa0 right a phase voltage
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_13;  // pc3 right b phase voltage
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_14;  // pc4 left b phase voltage
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_15;  // pc5 left c phase voltage
  sConfig.Rank    = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // Enable DMA for ADC1 i.e. current sampling
  // Values will be copied in the modulator
  hadc1.Instance->CR2 |= ADC_CR2_DMA;

  __HAL_RCC_DMA1_CLK_ENABLE();

  DMA1_Channel1->CCR   = 0;
  DMA1_Channel1->CNDTR = 4;
  DMA1_Channel1->CPAR  = (uint32_t)&(ADC1->DR);
  DMA1_Channel1->CMAR  = &rdson_meas[0];

  // 32 bit peripheral to 16 bit memory --> only low 16 bits copied
  // no end-of-conversion interrupt
  DMA1_Channel1->CCR   = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_1;
  DMA1_Channel1->CCR  |= DMA_CCR_EN;

  // Enable end of conversion interrupt
  //hadc1.Instance->CR1 |= ADC_CR1_EOCIE; // Interrupt is not needed, will be handled in the timer update
  //HAL_NVIC_SetPriority(ADC1_2_IRQn, 10, 0);
  //HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

  __HAL_ADC_ENABLE(&hadc1);
}
