// Handle power button sequences for powering up and down the system
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
#include "config.h"
#include "defines.h"
#include "powersw.h"
#include "control.h"
#include "math.h"

#define TONE_LENGTH 200

// Power switch voltage is an analog measurement
extern volatile adc_buf_t analog_meas;
extern volatile uint8_t generic_adc_conv_done;

// Control tick is used to make power on/off sounds
extern uint16_t control_tick;


#if defined(POWER_BUTTON_ESTOP)
static uint8_t powersw_samples = 0;
#elif defined(POWER_BUTTON_NORMAL)
static uint8_t powersw_state = 0;
static uint16_t powersw_timer = 0;
extern volatile motor_state_t motor_state[2];
#endif


// Make a sequence of tones to be played during power on and off
// A three tone high to low melody during power off and low to high during power on
// tune = 0 --> power off tune, 1 (or nonzero) --> power on tune
void power_tune(uint8_t tune)
{
  uint16_t sound_timer;

  sound_timer = control_tick;
  set_buzzer(tune ? 0xAAAA : 0x8080, 0xFFFF);
  while((uint16_t)(control_tick - sound_timer) < TONE_LENGTH);

  sound_timer = control_tick;
  set_buzzer(0x8888, 0xFFFF);
  while((uint16_t)(control_tick - sound_timer) < TONE_LENGTH);

  sound_timer = control_tick;
  set_buzzer(tune ? 0x8080 : 0xAAAA, 0xFFFF);
  while((uint16_t)(control_tick - sound_timer) < TONE_LENGTH);
}


// Disable motors, release power latch and wait forever
void power_off(void) {

  disable_motors(0x01 | 0x02);

  // Release the power latch
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);

  // TODO: Beep or timeout or something, if the
  // power button is pressed, will not power
  // down but just hang there doing nothing
  power_tune(0);

  while(1);
}


// Check that the power button is pressed in correct sequence for power on,
// power off if not
void powersw_on_sequence(void) {
  uint16_t sw_timer;

  // Power button must be pressed twice and held for power on
  // Wait until power button reaches threshold, if not then when button
  // is released we shut down immediately
  while(analog_meas.v_switch < ADC_POWERSW_THRESHOLD);
  // Wait until power button is released
  while(analog_meas.v_switch > ADC_POWERSW_THRESHOLD);

  // Release timer
  sw_timer = POWERSW_OFF_TIMER;
  while(sw_timer) {
    if(analog_meas.v_switch > ADC_POWERSW_THRESHOLD)
      break;

    while(!generic_adc_conv_done);
    generic_adc_conv_done = 0;	// Can be reset here because this function blocks
    sw_timer--;
  }
  if(!sw_timer)     // Button was not re-pressed soon enough
    power_off();

  // Button has to be pressed long enough the second time
  sw_timer = POWERSW_ON_TIMER;
  while(sw_timer) {
    if(analog_meas.v_switch < ADC_POWERSW_THRESHOLD)
      break;
    while(!generic_adc_conv_done);
    generic_adc_conv_done = 0;	// Can be reset here because this function blocks
    sw_timer--;
  }
  if(sw_timer)  // Released too early
    power_off();

  // Sequence was done correctly, can power up
}


// Check if power button has been pressed in proper
// sequence for powering of the system
void powersw_off_sequence(void) {
  // Check power button presses
  if(generic_adc_conv_done) {

#if defined(POWER_BUTTON_NORMAL)
    // Same routine as for power on, needs 2 pressed
    // and the latter is held for long time, and
    // speed must be below threshold
    if(powersw_state == 0 && analog_meas.v_switch >= ADC_POWERSW_THRESHOLD) {
      // Initial button press
      powersw_state = 1;
      powersw_timer = POWERSW_OFF_TIMER;

    } else if(powersw_state == 1 && powersw_timer && analog_meas.v_switch < ADC_POWERSW_THRESHOLD) {
      // Button must be released shortly
      powersw_state = 2;
      powersw_timer = POWERSW_OFF_TIMER;

    } else if(powersw_state == 2 && powersw_timer && analog_meas.v_switch >= ADC_POWERSW_THRESHOLD) {
      // Button was re-pressed shortly again
      powersw_state = 3;
      powersw_timer = POWERSW_ON_TIMER;

    } else if(powersw_state == 3 && analog_meas.v_switch >= ADC_POWERSW_THRESHOLD) {
      if(!powersw_timer) {
        // Power off if speed is below limit
        if(ABS(motor_state[STATE_LEFT].act.speed) < POWEROFF_SPEED_LIMIT && ABS(motor_state[STATE_RIGHT].act.speed) < POWEROFF_SPEED_LIMIT) {
          power_off();
        } else {
          // TODO: Beep?
          powersw_state = 0;
          powersw_timer = 0;
        }
      }

    } else {
      // Sequence failed, return to normal
      powersw_state = 0;
      powersw_timer = 0;
    }

    // If timer has run out, return to normal
    // Note that the final button press where the timer must
    // run out was aleady handled above
    if(!powersw_timer) {
      powersw_state = 0;
    } else {
      powersw_timer--;
    }


#elif defined(POWER_BUTTON_ESTOP)
    // In e-stop mode, when the button is released
    // and kept open for given samples, shut down
    // everything

    if(analog_meas.v_switch < ADC_POWERSW_THRESHOLD) {
      // Power button released
      powersw_samples++;
    } else {
      powersw_samples = 0;
    }

    if(powersw_samples > POWERSW_ESTOP_SAMPLES) {
      // Emergency shutdown (or normal in some cases)
      disable_motors(0x01 | 0x02);
      power_off();
    }
#endif
  }

}


// Long power button press is used as a fault reset
static uint16_t powersw_reset_timer = 0;

uint8_t powersw_fault_reset(void) {
  if(generic_adc_conv_done && analog_meas.v_switch >= ADC_POWERSW_THRESHOLD) {
    if((uint16_t)(control_tick - powersw_reset_timer) > POWERSW_FAULT_RESET)
      return 1; // Power button pressed long enough for reset
  } else {
    powersw_reset_timer = control_tick;
  }
  return 0;
}
