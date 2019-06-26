/**
 **************************************************************************
 * File Name    : system_at32f4xx.h
 * Description  : CMSIS Cortex-M4 system header file.
 * Date         : 2018-02-26
 * Version      : V1.0.4
 **************************************************************************
 */


/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup AT32F4xx_system
  * @{
  */

/**
  * @brief Define to prevent recursive inclusion
  */
#ifndef __SYSTEM_AT32F4xx_H
#define __SYSTEM_AT32F4xx_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup AT32F4xx_System_Includes
  * @{
  */

/**
  * @}
  */


/** @addtogroup AT32F4xx_System_Exported_types
  * @{
  */

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */

/**
  * @}
  */

/** @addtogroup AT32F4xx_System_Exported_Constants
  * @{
  */
extern __I uint8_t  AHBPrescTable[16U];  /*!< AHB prescalers table values */
extern __I uint8_t  APBPrescTable[8U];   /*!< APB prescalers table values */


#define RCC_PLLRefClk_HSI_Div2          ((uint32_t)0x00000000)

#define RCC_PLLRefClk_HSE_Div1          ((uint32_t)0x00010000)
#define RCC_PLLRefClk_HSE_Div2          ((uint32_t)0x00030000)
#define IS_RCC_PLL_CFG(CFG)             (((CFG) == RCC_PLLRefClk_HSI_Div2) || \
                                         ((CFG) == RCC_PLLRefClk_HSE_Div1) || \
                                         ((CFG) == RCC_PLLRefClk_HSE_Div2))

/**
  * @}
  */

/** @addtogroup AT32F4xx_System_Exported_Macros
  * @{
  */
#define HSE_STABLE_DELAY             (5000u)
#define PLL_STABLE_DELAY             (500u)
/**
  * @}
  */

/** @addtogroup AT32F4xx_System_Exported_Functions
  * @{
  */

extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_AT32F4xx_H */

/**
  * @}
  */

/**
  * @}
  */

