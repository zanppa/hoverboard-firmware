// Generic analog-to-digital conversions
// e.g. temperature, battery voltage
/*
Copyright (C) 2019 Lauri Peltonen

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

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

ADC_HandleTypeDef hadc3;
volatile adc_buf_t analog_meas;

volatile uint8_t generic_adc_conv_done = 0;

static volatile uint16_t adc_raw_data[ADC_MAX_CH] = {0};	// Max 16 conversions

// Pointers to where to store analog variables
// order must match the channel mapping of ADC3, and NULL means do not copy
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

// ADC3 init function
// This is used to sample all non-motor related parameters
// like temperature, battery voltage etc.
void ADC3_init(void) {
  //ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC3_CLK_ENABLE();

  hadc3.Instance                   = ADC3;
  hadc3.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode    = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion       = 6;		// Up to 16 conversions
  HAL_ADC_Init(&hadc3);

  /**Enable or disable the remapping of ADC1_ETRGREG:
    * ADC1 External Event regular conversion is connected to TIM8 TRG0
    */
  // Only software trigger is used at this point
  //__HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();

  // No multi-mode needed
  //Configure the ADC multi-mode
  //multimode.Mode = ADC_DUALMODE_REGSIMULT;
  //HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode);

  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;

  sConfig.Channel = ADC_CHANNEL_12; // Battery voltage
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

  sConfig.Channel = ADC_CHANNEL_1; // Power switch voltage
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

  sConfig.Channel = ADC_CHANNEL_2; // Left UART TX
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

  sConfig.Channel = ADC_CHANNEL_3; // Left UART RX
  sConfig.Rank    = 4;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

  sConfig.Channel = ADC_CHANNEL_VREFINT; // Internal reference voltage
  sConfig.Rank    = 5;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

  // Internal temperature must be sampled with long sample time
  // Recommended is 17.1 us which is not possible with 8 MHz clock
  // so use the next larger one
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR; //internal temperature
  sConfig.Rank    = 6;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

  // Enable DMA
  hadc3.Instance->CR2 |= ADC_CR2_DMA;

  // Enable VREF and temperature
  hadc3.Instance->CR2 |= ADC_CR2_TSVREFE;

  __HAL_ADC_ENABLE(&hadc3);

  // ADC3 uses DMA2 channel 5
  __HAL_RCC_DMA2_CLK_ENABLE();

  DMA2_Channel5->CCR   = 0;
  DMA2_Channel5->CNDTR = 6;
  DMA2_Channel5->CPAR  = (uint32_t)&(ADC3->DR);
  DMA2_Channel5->CMAR  = (uint32_t)(&adc_raw_data[0]);  //(uint32_t)&adc_buffer;

  //ADC DMA settings:
  //Mem size 16-bit,
  //Peripheral size 32-bit, I
  //Increment memory address,
  //Circular operation,
  //Peripheral-to-memory
  //Transfer complete interrupt
  //Priority level high

  // This transfers 32 bits since it used to transfer also the ADC2 values
  //DMA2_Channel5->CCR   = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_PL_1;

  // Read 32 bits from peripheral then write lowest 16 bits to memory
  DMA2_Channel5->CCR  = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_PL_1;
  DMA2_Channel5->CCR |= DMA_CCR_EN;

  HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);
}


// This function samples ADC3 multiple times and
// averages the output to offset register(s)
// ADC3 must be initialized first
void ADC3_calibrate(void) {
  // Do internal ADC calibration
  hadc3.Instance->CR2 |= ADC_CR2_CAL;
  // Wait until internal calibration is finished
  while(hadc3.Instance->CR2 & ADC_CR2_CAL);

  // It would be possible to now calibrate offsets
  // of external circuits, but none of the selected
  // channels have zero volt on them (on ADC1), so
  // this step can be skipped
}



// Handle ADC3 end-of-conversion interrupt
// This function copies all data from the DMA buffer
// to correct variales
void DMA2_Channel4_5_IRQHandler() {
  uint8_t i;

  // Clear interrupt flag of channel 5
  DMA2->IFCR = DMA_IFCR_CTCIF5;

  // DEBUG: Toggle led in ADC/DMA
  //HAL_GPIO_TogglePin(LED_PORT,LED_PIN);

  // Copy data from ADC DMA buffer to variables
  for(i=0; i<ADC_MAX_CH; i++)
    if(adc_map[i]) *(adc_map[i]) = adc_raw_data[i];

  // Mark ADC conversion done
  generic_adc_conv_done = 1;
}

