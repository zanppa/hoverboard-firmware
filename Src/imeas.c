// Current measurement in SVM mode

#include "stm32f1xx_hal.h"

static ADC_HandleTypeDef hadc2;
static ADC_HandleTypeDef hadc3;


// ADC2 init function. ADC2 is used to measure left motor
// current and phase voltages
void ADC2_init(void) {
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC2_CLK_ENABLE();

  hadc2.Instance                   = ADC2;
  hadc2.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode    = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion       = 1;
  HAL_ADC_Init(&hadc2);

  // Sample about 3.5 us after switching
  // Total conversion time 5.125 us
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;

  // TODO: Check is this really left or right!
  sConfig.Channel = ADC_CHANNEL_11; // Left motor shunt current
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  // Enable end of conversion interrupt
  hadc2.Instance->CR1 |= ADC_CR1_EOCIE;
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

  __HAL_ADC_ENABLE(&hadc2);
}


// ADC3 init function. ADC3 is used to measure right motor
// current and phase voltages
void ADC3_init(void) {
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC3_CLK_ENABLE();

  hadc3.Instance                   = ADC3;
  hadc3.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode    = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion       = 1;
  HAL_ADC_Init(&hadc3);

  // Sample about 3.5 us after switching
  // Total conversion time 5.125 us
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;

  // TODO: Check is this really left or right!
  sConfig.Channel = ADC_CHANNEL_10; // Right motor shunt current
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

  // Enable end of conversion interrupt
  hadc3.Instance->CR1 |= ADC_CR1_EOCIE;
  HAL_NVIC_SetPriority(ADC3_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(ADC3_IRQn);

  __HAL_ADC_ENABLE(&hadc3);
}



// TIM1 (left) capture/compare trigger handles launching
// A/D conversion for one of the motors (ADC2)
void TIM1_CC_IRQHandler(void) {
  // Clear all capture/compare interrupt flags
  TIM1->SR &= ~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);

  ADC2->CR2 |= ADC_CR2_SWSTART;
}

// TIM8 (right) capture/compare trigger handles launching
// A/D conversion for the other motor (ADC3)
void TIM8_CC_IRQHandler(void) {
  TIM8->SR &= ~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);

  ADC3->CR2 |= ADC_CR2_SWSTART;
}


// Handle ADC2 end of conversion interrupt
void ADC1_2_IRQHandler(void) {
  // Read data register, which also clears the interrupt flag
  uint16_t data = ADC2->DR;
}


// Handle ADC3 end of conversion interrupt
void ADC3_IRQhandler(void) {
  // Read data register, which also clears the interrupt flag
  uint16_t data = ADC3->DR;
}
