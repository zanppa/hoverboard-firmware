#include "defines.h"
#include "config.h"

// ADC runs at 8 MHz (main.c)
// Sampling time is tsample + 12.5 cycles, i.e.
// 7.5 sampling time is 7.5+12.5 = 20 cycles @ 8 MHz = 2.5 us
//	tsample	sampling	conversion
//			time us		time us
// 	1.5		0.1875		1.75
//	7.5		0.9375		2.5
//	13.5	1.6875		3.25
//	28.5	3.5625		5.125
//	41.5	5.1875		6.75
//	55.5	6.9375		8.5
//	71.5	8.9375		10.5
//	239.5	29.9375		31.5

#define ADC_MAX_CH	16		// Maximum analog channels to sample

ADC_HandleTypeDef hadc1;
//ADC_HandleTypeDef hadc2;
volatile adc_buf_t analog_meas;

//static volatile uint16_t adc_raw_data[ADC_MAX_CH] = {0};	// Max 16 conversions
volatile uint16_t adc_raw_data[ADC_MAX_CH] = {0};	// DEBUG: Not static and do not copy

// Pointers to where to store analog variables
// order must match the channel mapping of ADC1, and NULL means do not copy
static volatile uint16_t *adc_map[ADC_MAX_CH] = {
  &analog_meas.v_battery,
  &analog_meas.v_switch,
  &analog_meas.analog_ref_1,
  &analog_meas.analog_ref_2,
  &analog_meas.v_ref,
  &analog_meas.temperature,
  NULL, NULL,
  NULL, NULL, NULL, NULL,
  NULL, NULL, NULL, NULL
};

// ADC1 init function
// This is used to sample all non-motor related parameters
// like temperature, battery voltage etc.
void ADC1_init(void) {
  //ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance                   = ADC1;
  hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 6;		// Up to 16 conversions
  HAL_ADC_Init(&hadc1);

  /**Enable or disable the remapping of ADC1_ETRGREG:
    * ADC1 External Event regular conversion is connected to TIM8 TRG0
    */
  // Only software trigger is used at this point
  //__HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();

  // No multi-mode needed
  //Configure the ADC multi-mode
  //multimode.Mode = ADC_DUALMODE_REGSIMULT;
  //HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;

  sConfig.Channel = ADC_CHANNEL_12; // Battery voltage
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_1; // Power switch voltage
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2; // Left UART TX
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_3; // Left UART RX
  sConfig.Rank    = 4;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_VREFINT; // Internal reference voltage
  sConfig.Rank    = 5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // Internal temperature must be sampled with long sample time
  // Recommended is 17.1 us which is not possible with 8 MHz clock
  // so use the next larger one
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR; //internal temperature
  sConfig.Rank    = 6;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // Enable DMA
  hadc1.Instance->CR2 |= ADC_CR2_DMA;

  __HAL_ADC_ENABLE(&hadc1);

  __HAL_RCC_DMA1_CLK_ENABLE();

  DMA1_Channel1->CCR   = 0;
  DMA1_Channel1->CNDTR = 6;
  DMA1_Channel1->CPAR  = (uint32_t)&(ADC1->DR);
  DMA1_Channel1->CMAR  = (uint32_t)(&adc_raw_data[0]);  //(uint32_t)&adc_buffer;

  //ADC DMA settings:
  //Mem size 32-bit,
  //Peripheral size 32-bit, I
  //Increment memory address,
  //Circular operation,
  //Peripheral-to-memory
  //Transfer complete interrupt
  //Priority level high

  // This transfers 32 bits since it used to transfer also the ADC2 values
  //DMA1_Channel1->CCR   = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_PL_1;

  // Read 32 bits (16xADC2 16xADC1) from peripheral then write lowest 16 bits to memory
  DMA1_Channel1->CCR  = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_PL_1;
  DMA1_Channel1->CCR |= DMA_CCR_EN;

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}


// This function samples ADC1 multiple times and
// averages the output to offset register(s)
// ADC1 must be initialized first
void ADC1_calibrate(void) {
  // Do internal ADC calibration
  hadc1.Instance->CR2 |= ADC_CR2_CAL;
  // Wait until internal calibration is finished
  while(hadc1.Instance->CR2 & ADC_CR2_CAL);

  // It would be possible to now calibrate offsets
  // of external circuits, but none of the selected
  // channels have zero volt on them (on ADC1), so
  // this step can be skipped
}


/* ADC2 init function */
// This is not used at the moment
void MX_ADC2_Init(void) {
#if 0
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC2_CLK_ENABLE();

  hadc2.Instance                   = ADC2;
  hadc2.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode    = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion       = 4;
  HAL_ADC_Init(&hadc2);

  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;

  sConfig.Channel = ADC_CHANNEL_15; //right motor phace C sense
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_13; //left motor phace B sense
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;

  sConfig.Channel = ADC_CHANNEL_10; //left motor shunt current
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR; //internal temperature
  sConfig.Rank    = 4;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  hadc2.Instance->CR2 |= ADC_CR2_DMA;
  __HAL_ADC_ENABLE(&hadc2);
#endif
}



// Handle ADC1 end-of-conversion interrupt
// This function copies all data from the DMA buffer
// to correct variales
void DMA1_Channel1_IRQHandler() {
  uint8_t i;

  // Clear interrupt flag
  DMA1->IFCR = DMA_IFCR_CTCIF1;

  // DEBUG: Toggle led in ADC/DMA
  HAL_GPIO_TogglePin(LED_PORT,LED_PIN);

  // Copy data from ADC DMA buffer to variables
  for(i=0; i<ADC_MAX_CH; i++)
    if(adc_map[i]) *(adc_map[i]) = adc_raw_data[i];
}

