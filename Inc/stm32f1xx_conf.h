/**
 **************************************************************************
 * File Name    : stm32f1xx_conf.h
 * Description  : stm32f1xx configuration file
 * Date         : 2018-02-12
 * Version      : V1.0.3
 **************************************************************************
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1xx_CONF_H
#define __STM32F1xx_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment/Comment the line below to enable/disable peripheral header file inclusion */
/*#include "stm32f1xx_adc.h"
#include "stm32f1xx_bkp.h"
#include "stm32f1xx_can.h"
#include "stm32f1xx_crc.h"
#include "stm32f1xx_dac.h"
#include "stm32f1xx_dbgmcu.h"
#include "stm32f1xx_dma.h"
#include "stm32f1xx_exti.h"
#include "stm32f1xx_flash.h"
#include "stm32f1xx_xmc.h"
#include "stm32f1xx_gpio.h"
#include "stm32f1xx_i2c.h"
#include "stm32f1xx_iwdg.h"
#include "stm32f1xx_pwr.h"
#include "stm32f1xx_rcc.h"
#include "stm32f1xx_rtc.h"
#include "stm32f1xx_sdio.h"
#include "stm32f1xx_spi.h"
#include "stm32f1xx_tim.h"
#include "stm32f1xx_usart.h"
#include "stm32f1xx_wwdg.h"
#include "misc.h" *//* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/* Uncomment the line below to expanse the "assert_param" macro in the
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports
  *         the name of the source file and the source line number of the call
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void assert_failed(uint8_t* file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F1xx_CONF_H */


