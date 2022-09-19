// Current measurement in SVM mode
// using voltage measurement over the lower
// transistor Rds,on
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

#include "stm32f1xx_hal.h"
#include "imeas.h"
#include "config.h"
#include "defines.h"
#include "math.h"

#define RDSON_MEAS_COUNT	4

ADC_HandleTypeDef adc_rdson;

// Rds,on measurement order:
// Left A, Left B, Right B, Right C phases
volatile uint16_t rdson_meas[4];
volatile uint16_t rdson_offset[4];

#if defined(I_MEAS_RDSON)
volatile uint8_t imeas_calibration_done = 0;
#else
volatile uint8_t imeas_calibration_done = 1; // Pass this check if current measurement is not used
#endif


// Current measurement result structure
volatile i_meas_t i_meas = {0};

static volatile uint8_t rdson_adc_conv_done = 0;

// Convert ADC measurement value (after offset compensation) to P.U. current value
// volts = value * 3.3 / 4096
// current = volts * RDSON
// current_pu = current / motor_nominal_current
// Without division by 4096 we directly get fixed point value
const uint16_t rdson_to_i = (3.3 / RDSON) / MOTOR_CUR; // --> FIXED_ONE equals MOTOR_CUR

// ADC1 init function. ADC1 is used to measure motor currents from lower switch Rds,on
void ADC1_init(void) {
  ADC_ChannelConfTypeDef sConfig;

  __HAL_RCC_ADC1_CLK_ENABLE();

  adc_rdson.Instance                   = ADC1;
  adc_rdson.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  adc_rdson.Init.ContinuousConvMode    = DISABLE;
  adc_rdson.Init.DiscontinuousConvMode = DISABLE;
  adc_rdson.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  // TODO: Could use timer 8 update (or underflow?) event as the source so software would not need to trigger ADC
  //adc_rdson.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T8_TRGO;	// Trigger ADC on timer8 update event (right timer)
  adc_rdson.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  adc_rdson.Init.NbrOfConversion       = RDSON_MEAS_COUNT;	// 2 currents for both motors
  HAL_ADC_Init(&adc_rdson);

  // Use the fastest possible sampling time => 1.75 us * 4 = 7 us total
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

  sConfig.Channel = ADC_CHANNEL_0;  // PA0 Left A phase voltage
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&adc_rdson, &sConfig);

  sConfig.Channel = ADC_CHANNEL_13;  // PC3 Left B phase voltage
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&adc_rdson, &sConfig);

  sConfig.Channel = ADC_CHANNEL_14;  // PC4 Right B phase voltage (Blue cable)
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&adc_rdson, &sConfig);

  sConfig.Channel = ADC_CHANNEL_15;  // PC5 Right C phase voltage (Yellow cable)
  sConfig.Rank    = 4;
  HAL_ADC_ConfigChannel(&adc_rdson, &sConfig);

  // Enable DMA for ADC1 i.e. current sampling
  // Values will be copied in the modulator
  adc_rdson.Instance->CR2 |= ADC_CR2_DMA;

  __HAL_RCC_DMA1_CLK_ENABLE();

  DMA1_Channel1->CCR   = 0;
  DMA1_Channel1->CNDTR = RDSON_MEAS_COUNT;
  DMA1_Channel1->CPAR  = (uint32_t)&(ADC1->DR);
  DMA1_Channel1->CMAR  = (uint32_t)&rdson_meas[0];

  // 32 bit peripheral to 16 bit memory --> only low 16 bits copied
  // no end-of-conversion interrupt
  DMA1_Channel1->CCR   = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_1 | DMA_CCR_TCIE;
  DMA1_Channel1->CCR  |= DMA_CCR_EN;

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);


  // Enable end of conversion interrupt
  //adc_rdson.Instance->CR1 |= ADC_CR1_EOCIE; // Interrupt is not needed, will be handled in the timer update
  //HAL_NVIC_SetPriority(ADC1_2_IRQn, 10, 0);
  //HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

  __HAL_ADC_ENABLE(&adc_rdson);
}


// This function samples ADC1 multiple times and
// averages the output to offset register(s)
// ADC1 must be initialized first
void ADC1_calibrate(void) {
  uint32_t offsets[RDSON_MEAS_COUNT] = {0};

  // Do internal ADC calibration
  adc_rdson.Instance->CR2 |= ADC_CR2_CAL;
  // Wait until internal calibration is finished
  while(adc_rdson.Instance->CR2 & ADC_CR2_CAL);

  // Calibrate zero point offsets
  for(int i=0; i < ADC_OFFSET_SAMPLES; i++) {
    // Clear DMA transfer complete flag
    DMA2->ISR |= DMA_ISR_TCIF1;	// Channel 1

    // Trigger conversion
    // Conversion must be triggered by SVM so that it happens at correct point in modulation sequence
    //adc_rdson.Instance->CR2 |= ADC_CR2_SWSTART;

    // Wait until DMA has finished transfer, Channel 5 end of transfer flag set
    while(!rdson_adc_conv_done);
    rdson_adc_conv_done = 0;

    // Accumulate offsets
    for(int j = 0; j < RDSON_MEAS_COUNT; j++)
      offsets[j] += rdson_meas[j];
  }

  // Copy the final averaged offsets to the offset variable
  for(int j = 0; j < RDSON_MEAS_COUNT; j++) {
    offsets[j] /= ADC_OFFSET_SAMPLES;
    rdson_offset[j] = offsets[j];
  }

  imeas_calibration_done = 1;
}

// End of transfer interrupt handler for DMA1 channel 1 (Rds,on measurement)
void DMA1_Channel1_IRQHandler(void) {
  DMA1->IFCR |= DMA_IFCR_CGIF1;	// Clear all interrupt flags for channel 1
  //HAL_GPIO_TogglePin(LED_PORT,LED_PIN);

  // Copy measurements to current measurement array taking into account
  // the offsets
  // The values are valid after the calibration is done
  // Measurement is from motor into inverter, while we want it the other way so it is inverted
  i_meas.i_lA = (rdson_offset[0] - rdson_meas[0]) * rdson_to_i;
  i_meas.i_lB = (rdson_offset[1] - rdson_meas[1]) * rdson_to_i;
  i_meas.i_rB = (rdson_offset[2] - rdson_meas[2]) * rdson_to_i;
  i_meas.i_rC = (rdson_offset[3] - rdson_meas[3]) * rdson_to_i;

  rdson_adc_conv_done = 1;	// This is used for calibration
}
