// Current measurement in SVM mode

#include "stm32f1xx_hal.h"


// TIM1 capture/compare trigger handles launching
// A/D conversion for one of the motors (TIM1)
void TIM1_CC_IRQHandler(void) {
  // Clear all capture/compare interrupt flags
  TIM1->SR &= ~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);

}

// TIM1 capture/compare trigger handles launching
// A/D conversion for the other motor (TIM8)
void TIM8_CC_IRQHandler(void) {
  TIM8->SR &= ~(TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF);

}
