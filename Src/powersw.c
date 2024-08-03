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

// Control tick is used to make power on/off sounds
extern volatile uint16_t control_tick;

// Status bits
extern volatile uint8_t status_bits;
extern volatile uint8_t imeas_calibration_done; // TODO: Trying this hack...

static uint16_t powersw_timer = 0;
#if defined(POWER_BUTTON_NORMAL)
static uint8_t powersw_state = 0;
extern volatile motor_state_t motor_state[2];
#endif


// Make a sequence of tones to be played during power on and off
// A three tone high to low melody during power off and low to high during power on
// tune = 0 --> power off tune, 1 (or nonzero) --> power on tune
void power_tune(uint8_t tune)
{
  uint16_t sound_timer;

  set_buzzer(tune ? 0x8080 : 0xAAAA, 0xFFFF, 1);
  sound_timer = control_tick;
  while((uint16_t)(control_tick - sound_timer) < TONE_LENGTH);

  set_buzzer(0x8888, 0xFFFF, 1);
  sound_timer = control_tick;
  while((uint16_t)(control_tick - sound_timer) < TONE_LENGTH);

  set_buzzer(tune ? 0xAAAA : 0x8080, 0xFFFF, 1);
  sound_timer = control_tick;
  while((uint16_t)(control_tick - sound_timer) < TONE_LENGTH);

  set_buzzer(0, 0, 0); // Off
}


// Disable motors, release power latch and wait forever
void power_off(void) {
  disable_motors(0x01 | 0x02);
  status_bits &= ~STATUS_READY;	// Clear ready flag
  imeas_calibration_done = 0; // TODO: Hack to prevent some control functions from running during power off..

  // Play a turn-off tune (if buzzer is enabled), if the
  // power button is kept pressed, board will not power
  // down but just hang there doing nothing
  power_tune(0);

  // Release the power latch
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 0);

  while(1);
}


// Check that the power button is pressed in correct sequence for power on,
// power off if not
// This function is blocking
void powersw_on_sequence(void) {
  uint16_t sw_timer;
  uint8_t ok = 0;

  // Power button must be pressed twice and held for power on
  // Wait until power button reaches threshold, if not then when button
  // is released we shut down immediately
  while(analog_meas.v_switch < ADC_POWERSW_THRESHOLD);
  // Wait until power button is released
  while(analog_meas.v_switch > ADC_POWERSW_THRESHOLD);


  // Release timer
  sw_timer = control_tick;
  while((uint16_t)(control_tick - sw_timer) < POWERSW_OFF_TIMER) {
    if(analog_meas.v_switch > ADC_POWERSW_THRESHOLD) {
      ok = 1; // Button pressed before timeout
      break;
    }
  }
  if(!ok)     // Button was not re-pressed soon enough
    power_off();

  // Button has to be pressed long enough the second time
  sw_timer = control_tick;
  while((uint16_t)(control_tick - sw_timer) < POWERSW_ON_TIMER) {
    if(analog_meas.v_switch < ADC_POWERSW_THRESHOLD) {
      // Button released too early
      ok = 0;
      break;
    }
  }
  if(!ok)  // Released too early
    power_off();

  // Sequence was done correctly, can power up
}


// Check if power button has been pressed in proper
// sequence for powering of the system
void powersw_off_sequence(void) {
  // Check power button presses

#if defined(POWER_BUTTON_NORMAL)
  // Same routine as for power on, needs 2 pressed
  // and the latter is held for long time, and
  // speed must be below threshold
  if(powersw_state == 0 && analog_meas.v_switch >= ADC_POWERSW_THRESHOLD) {
    // Initial button press
    powersw_state = 1;
    powersw_timer = control_tick; //POWERSW_OFF_TIMER;

  } else if(powersw_state == 1) {
     if((uint16_t)(control_tick - powersw_timer) > POWERSW_OFF_TIMER) {
       // Reset sequence, button was kept pressed too long
       powersw_state = 0;
     } else if(analog_meas.v_switch < ADC_POWERSW_THRESHOLD) {
      // Button must be released shortly
      powersw_state = 2;
      powersw_timer = control_tick; //POWERSW_OFF_TIMER;
    }

  } else if(powersw_state == 2) {
    if((uint16_t)(control_tick - powersw_timer) > POWERSW_OFF_TIMER) {
       // Reset sequence, button was not pressed quickly enough
       powersw_state = 0;
    } else if(analog_meas.v_switch >= ADC_POWERSW_THRESHOLD) {
      // Button was re-pressed shortly again
      powersw_state = 3;
      powersw_timer = control_tick; //POWERSW_ON_TIMER;
    }

  } else if(powersw_state == 3) {
    if(analog_meas.v_switch < ADC_POWERSW_THRESHOLD) {
       // Reset sequence, button was released too early
       powersw_state = 0;
    } else if((uint16_t)(control_tick - powersw_timer) > POWERSW_ON_TIMER) {
      // Button was kept pressed long enough
      // Power off if speed is below limit
//      if(ABS(motor_state[STATE_LEFT].act.speed) < POWEROFF_SPEED_LIMIT && ABS(motor_state[STATE_RIGHT].act.speed) < POWEROFF_SPEED_LIMIT) {
//        power_off();
//      } else {
        // Speed was too high, cannot turn off while running!
        // TODO: Beep or some other indication of error?
//        powersw_state = 0;
//      }
      power_off();	// Allow turning of even if running, could help if software goes haywire
    }

  } else {
    // Sequence failed, return to normal
    powersw_state = 0;
  }

#elif defined(POWER_BUTTON_ESTOP)
    // In e-stop mode, when the button is released
    // and kept open for given samples, shut down
    // everything

    if(analog_meas.v_switch < ADC_POWERSW_THRESHOLD) {
      // Power button released
      if((uint16_t)(control_tick - powersw_timer) > POWERSW_ESTOP_SAMPLES) {
        // Emergency shutdown (or normal in some cases)
        disable_motors(0x01 | 0x02);
        power_off();
      }
    } else {
      powersw_timer = control_tick;
    }
#endif
}


// Long power button press is used as a fault reset
static uint16_t powersw_reset_timer = 0;

uint8_t powersw_fault_reset(void) {
  if(analog_meas.v_switch >= ADC_POWERSW_THRESHOLD) {
    if((uint16_t)(control_tick - powersw_reset_timer) > POWERSW_FAULT_RESET)
      return 1; // Power button pressed long enough for reset
  } else {
    powersw_reset_timer = control_tick;
  }
  return 0;
}
