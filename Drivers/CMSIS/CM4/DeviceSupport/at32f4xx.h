/**
 **************************************************************************
 * File Name    : at32f4xx.h
 * Description  : at32f4xx peripheral access layer header file
 * Date         : 2018-02-26
 * Version      : V1.0.4
 **************************************************************************
 */


/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup at32f4xx
  * @{
  */

#ifndef __AT32F4xx_H
#define __AT32F4xx_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Library_configuration_section
  * @{
  */

/* Uncomment the line below according to the target AT32 device used in your
   application
  */
#if !defined (AT32F403Cx_MD) && !defined (AT32F403Cx_HD) && \
    !defined (AT32F403Cx_XL) && !defined (AT32F403Rx_HD) && \
    !defined (AT32F403Rx_XL) && !defined (AT32F403Vx_HD) && \
    !defined (AT32F403Vx_XL) && !defined (AT32F403Zx_HD) && \
    !defined (AT32F403Zx_XL)
/* #define AT32F403Cx_MD */   /*!< AT32F403Cx_MD: LQFP48, Medium density devices: AT32F403CB */
/* #define AT32F403Cx_HD */   /*!< AT32F403Cx_HD: LQFP48, High density devices: AT32F403CC, AT32F403CE */
/* #define AT32F403Cx_XL */   /*!< AT32F403Cx_XL: LQFP48, XL-density devices: AT32F403CG */
/* #define AT32F403Rx_HD */   /*!< AT32F403Rx_HD: LQFP64, High density devices: AT32F403RC, AT32F403RE */
/* #define AT32F403Rx_XL */   /*!< AT32F403Rx_XL: LQFP64, XL-density devices: AT32F403RG */
/* #define AT32F403Vx_HD */   /*!< AT32F403Vx_HD: LQFP100, High density devices: AT32F403VC, AT32F403VE */
/* #define AT32F403Vx_XL */   /*!< AT32F403Vx_XL: LQFP100, XL-density devices: AT32F403VG */
/* #define AT32F403Zx_HD */   /*!< AT32F403Zx_HD: LQFP144, High density devices: AT32F403ZC, AT32F403ZE */
/* #define AT32F403Zx_XL */   /*!< AT32F403Zx_XL: LQFP144, XL-density devices: AT32F403ZG */
#endif
/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.

 - High-density devices are at32f403xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 - XL-density devices are at32f403xx microcontrollers where
   the Flash memory density ranges between 512 and 1024 Kbytes.
  */

#if !defined (AT32F403Cx_MD) && !defined (AT32F403Cx_HD) && \
    !defined (AT32F403Cx_XL) && !defined (AT32F403Rx_HD) && \
    !defined (AT32F403Rx_XL) && !defined (AT32F403Vx_HD) && \
    !defined (AT32F403Vx_XL) && !defined (AT32F403Zx_HD) && \
    !defined (AT32F403Zx_XL)
#error "Please select first the target at32f4xx device used in your application (in at32f4xx.h file)"
#endif






#if !defined  LIBRARY_VERSION
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will
   be based on direct access to peripherals registers
   */
/*#define LIBRARY_VERSION*/
#endif

/**
 * @brief In the following line adjust the value of External High Speed oscillator (HSE)
   used in your application

   Tip: To avoid modifying this file each time you need to use different HSE, you
        can define the HSE value in your toolchain compiler preprocessor.
  */
#if !defined  HSE_VALUE
#define HSE_VALUE               ((uint32_t)8000000) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */


/**
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup
   Timeout value
   */
#define HSE_STARTUP_TIMEOUT     ((uint16_t)0x0500) /*!< Time out for HSE start up */

#define HSI_VALUE               ((uint32_t)8000000) /*!< Value of the Internal oscillator in Hz*/

/**
 * @brief at32f4xx Standard Peripheral Library version number
   */
#define __AT32F4xx_LIBRARY_VERSION_MAIN     (0x01) /*!< [31:24] main version */
#define __AT32F4xx_LIBRARY_VERSION_MIDDLE   (0x00) /*!< [23:16] middle version */
#define __AT32F4xx_LIBRARY_VERSION_MINOR    (0x01) /*!< [15:8]  minor version */
#define __AT32F4xx_LIBRARY_VERSION_RC       (0x00) /*!< [7:0]  release candidate */
#define __AT32F4xx_LIBRARY_VERSION          ( (__AT32F4xx_LIBRARY_VERSION_MAIN << 24)\
                                             |(__AT32F4xx_LIBRARY_VERSION_MIDDLE << 16)\
                                             |(__AT32F4xx_LIBRARY_VERSION_MINOR << 8)\
                                             |(__AT32F4xx_LIBRARY_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals
 */
#define __CM4_REV                 0x0001U  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             0     /*!< AT32 devices do not provide an MPU           */
#define __NVIC_PRIO_BITS          4     /*!< AT32 uses 4 Bits for the Priority Levels     */
#define __Vendor_SysTickConfig    0     /*!< Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             1U    /*!< FPU present                                  */

/**
 * @brief at32f4xx Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
typedef enum IRQn
{
    /******  Cortex-M4 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
    MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt              */
    BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                      */
    UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                    */
    SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                       */
    DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                 */
    PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                       */
    SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                   */

    /******  AT32 specific Interrupt Numbers *********************************************************/
    WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                            */
    PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt            */
    TAMPER_IRQn                 = 2,      /*!< Tamper Interrupt                                     */
    RTC_IRQn                    = 3,      /*!< RTC global Interrupt                                 */
    FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                               */
    RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                 */
    EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                 */
    EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                 */
    EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                 */
    EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                 */
    EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                 */
    DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                      */
    DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                      */
    DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                      */
    DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                      */
    DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                      */
    DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                      */
    DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                      */

#ifdef AT32F403Cx_MD
    ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
    USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
    TMR1_BRK_TMR9_IRQn          = 24,     /*!< TMR1 Break Interrupt                                 */
    TMR1_OV_TMR10_IRQn          = 25,     /*!< TMR1 Update Interrupt                                */
    TMR1_TRG_HALL_TMR11_IRQn    = 26,     /*!< TMR1 Trigger and Commutation Interrupt               */
    TMR1_CC_IRQn                = 27,     /*!< TMR1 Capture Compare Interrupt                       */
    TMR2_GLOBAL_IRQn            = 28,     /*!< TMR2 global Interrupt                                */
    TMR3_GLOBAL_IRQn            = 29,     /*!< TMR3 global Interrupt                                */
    TMR4_GLOBAL_IRQn            = 30,     /*!< TMR4 global Interrupt                                */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
    RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
    USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
    TMR8_BRK_TMR12_IRQn         = 43,     /*!< TMR8 Break Interrupt                                 */
    TMR8_OV_TMR13_IRQn          = 44,     /*!< TMR8 Update Interrupt                                */
    TMR8_TRG_HALL_TMR14_IRQn    = 45,     /*!< TMR8 Trigger and Commutation Interrupt               */
    TMR8_CC_IRQn                = 46,     /*!< TMR8 Capture Compare Interrupt                       */
    ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
    TMR5_GLOBAL_IRQn            = 50,     /*!< TMR5 global Interrupt                                */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
    TMR6_GLOBAL_IRQn            = 54,     /*!< TMR6 global Interrupt                                */
    TMR7_GLOBAL_IRQn            = 55,     /*!< TMR7 global Interrupt                                */
    DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
    DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
    DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
    DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
    SDIO2_IRQn                  = 60,     /*!< SDIO2 global Interrupt                               */
    I2C3_EV_IRQn                = 61,     /*!< I2C3 event interrupt                                 */
    I2C3_ER_IRQn                = 62,     /*!< I2C3 error interrupt                                 */
#endif /* AT32F403Cx_MD */

#ifdef AT32F403Cx_HD
    ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
    USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
    TMR1_BRK_TMR9_IRQn          = 24,     /*!< TMR1 Break Interrupt                                 */
    TMR1_OV_TMR10_IRQn          = 25,     /*!< TMR1 Update Interrupt                                */
    TMR1_TRG_HALL_TMR11_IRQn    = 26,     /*!< TMR1 Trigger and Commutation Interrupt               */
    TMR1_CC_IRQn                = 27,     /*!< TMR1 Capture Compare Interrupt                       */
    TMR2_GLOBAL_IRQn            = 28,     /*!< TMR2 global Interrupt                                */
    TMR3_GLOBAL_IRQn            = 29,     /*!< TMR3 global Interrupt                                */
    TMR4_GLOBAL_IRQn            = 30,     /*!< TMR4 global Interrupt                                */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
    RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
    USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
    TMR8_BRK_TMR12_IRQn         = 43,     /*!< TMR8 Break Interrupt                                 */
    TMR8_OV_TMR13_IRQn          = 44,     /*!< TMR8 Update Interrupt                                */
    TMR8_TRG_HALL_TMR14_IRQn    = 45,     /*!< TMR8 Trigger and Commutation Interrupt               */
    TMR8_CC_IRQn                = 46,     /*!< TMR8 Capture Compare Interrupt                       */
    ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
    TMR5_GLOBAL_IRQn            = 50,     /*!< TMR5 global Interrupt                                */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
    TMR6_GLOBAL_IRQn            = 54,     /*!< TMR6 global Interrupt                                */
    TMR7_GLOBAL_IRQn            = 55,     /*!< TMR7 global Interrupt                                */
    DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
    DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
    DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
    DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
    SDIO2_IRQn                  = 60,     /*!< SDIO2 global Interrupt                               */
    I2C3_EV_IRQn                = 61,     /*!< I2C3 event interrupt                                 */
    I2C3_ER_IRQn                = 62,     /*!< I2C3 error interrupt                                 */
#endif /* AT32F403Cx_HD */

#ifdef AT32F403Cx_XL
    ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
    USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
    TMR1_BRK_TMR9_IRQn          = 24,     /*!< TMR1 Break Interrupt                                 */
    TMR1_OV_TMR10_IRQn          = 25,     /*!< TMR1 Update Interrupt                                */
    TMR1_TRG_HALL_TMR11_IRQn    = 26,     /*!< TMR1 Trigger and Commutation Interrupt               */
    TMR1_CC_IRQn                = 27,     /*!< TMR1 Capture Compare Interrupt                       */
    TMR2_GLOBAL_IRQn            = 28,     /*!< TMR2 global Interrupt                                */
    TMR3_GLOBAL_IRQn            = 29,     /*!< TMR3 global Interrupt                                */
    TMR4_GLOBAL_IRQn            = 30,     /*!< TMR4 global Interrupt                                */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
    RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
    USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
    TMR8_BRK_TMR12_IRQn         = 43,     /*!< TMR8 Break Interrupt                                 */
    TMR8_OV_TMR13_IRQn          = 44,     /*!< TMR8 Update Interrupt                                */
    TMR8_TRG_HALL_TMR14_IRQn    = 45,     /*!< TMR8 Trigger and Commutation Interrupt               */
    TMR8_CC_IRQn                = 46,     /*!< TMR8 Capture Compare Interrupt                       */
    ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
    TMR5_GLOBAL_IRQn            = 50,     /*!< TMR5 global Interrupt                                */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
    TMR6_GLOBAL_IRQn            = 54,     /*!< TMR6 global Interrupt                                */
    TMR7_GLOBAL_IRQn            = 55,     /*!< TMR7 global Interrupt                                */
    DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
    DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
    DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
    DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
    SDIO2_IRQn                  = 60,     /*!< SDIO2 global Interrupt                               */
    I2C3_EV_IRQn                = 61,     /*!< I2C3 event interrupt                                 */
    I2C3_ER_IRQn                = 62,     /*!< I2C3 error interrupt                                 */
#endif /* AT32F403Cx_XL */

#ifdef AT32F403Rx_HD
    ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
    USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
    TMR1_BRK_TMR9_IRQn          = 24,     /*!< TMR1 Break Interrupt                                 */
    TMR1_OV_TMR10_IRQn          = 25,     /*!< TMR1 Update Interrupt                                */
    TMR1_TRG_HALL_TMR11_IRQn    = 26,     /*!< TMR1 Trigger and Commutation Interrupt               */
    TMR1_CC_IRQn                = 27,     /*!< TMR1 Capture Compare Interrupt                       */
    TMR2_GLOBAL_IRQn            = 28,     /*!< TMR2 global Interrupt                                */
    TMR3_GLOBAL_IRQn            = 29,     /*!< TMR3 global Interrupt                                */
    TMR4_GLOBAL_IRQn            = 30,     /*!< TMR4 global Interrupt                                */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
    RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
    USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
    TMR8_BRK_TMR12_IRQn         = 43,     /*!< TMR8 Break Interrupt                                 */
    TMR8_OV_TMR13_IRQn          = 44,     /*!< TMR8 Update Interrupt                                */
    TMR8_TRG_HALL_TMR14_IRQn    = 45,     /*!< TMR8 Trigger and Commutation Interrupt               */
    TMR8_CC_IRQn                = 46,     /*!< TMR8 Capture Compare Interrupt                       */
    ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
    SDIO1_IRQn                  = 49,     /*!< SDIO global Interrupt                                */
    TMR5_GLOBAL_IRQn            = 50,     /*!< TMR5 global Interrupt                                */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
    UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
    UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
    TMR6_GLOBAL_IRQn            = 54,     /*!< TMR6 global Interrupt                                */
    TMR7_GLOBAL_IRQn            = 55,     /*!< TMR7 global Interrupt                                */
    DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
    DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
    DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
    DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
    SDIO2_IRQn                  = 60,     /*!< SDIO2 global Interrupt                               */
    I2C3_EV_IRQn                = 61,     /*!< I2C3 event interrupt                                 */
    I2C3_ER_IRQn                = 62,     /*!< I2C3 error interrupt                                 */
#endif /* AT32F403Rx_HD */


#ifdef AT32F403Rx_XL
    ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
    USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
    TMR1_BRK_TMR9_IRQn          = 24,     /*!< TMR1 Break Interrupt                                 */
    TMR1_OV_TMR10_IRQn          = 25,     /*!< TMR1 Update Interrupt                                */
    TMR1_TRG_HALL_TMR11_IRQn    = 26,     /*!< TMR1 Trigger and Commutation Interrupt               */
    TMR1_CC_IRQn                = 27,     /*!< TMR1 Capture Compare Interrupt                       */
    TMR2_GLOBAL_IRQn            = 28,     /*!< TMR2 global Interrupt                                */
    TMR3_GLOBAL_IRQn            = 29,     /*!< TMR3 global Interrupt                                */
    TMR4_GLOBAL_IRQn            = 30,     /*!< TMR4 global Interrupt                                */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
    RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
    USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
    TMR8_BRK_TMR12_IRQn         = 43,     /*!< TMR8 Break Interrupt                                 */
    TMR8_OV_TMR13_IRQn          = 44,     /*!< TMR8 Update Interrupt                                */
    TMR8_TRG_HALL_TMR14_IRQn    = 45,     /*!< TMR8 Trigger and Commutation Interrupt               */
    TMR8_CC_IRQn                = 46,     /*!< TMR8 Capture Compare Interrupt                       */
    ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
    SDIO1_IRQn                  = 49,     /*!< SDIO global Interrupt                                */
    TMR5_GLOBAL_IRQn            = 50,     /*!< TMR5 global Interrupt                                */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
    UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
    UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
    TMR6_GLOBAL_IRQn            = 54,     /*!< TMR6 global Interrupt                                */
    TMR7_GLOBAL_IRQn            = 55,     /*!< TMR7 global Interrupt                                */
    DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
    DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
    DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
    DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
    SDIO2_IRQn                  = 60,     /*!< SDIO2 global Interrupt                               */
    I2C3_EV_IRQn                = 61,     /*!< I2C3 event interrupt                                 */
    I2C3_ER_IRQn                = 62,     /*!< I2C3 error interrupt                                 */
#endif /* AT32F403Rx_XL */

#ifdef AT32F403Vx_HD
    ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
    USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
    TMR1_BRK_TMR9_IRQn          = 24,     /*!< TMR1 Break Interrupt                                 */
    TMR1_OV_TMR10_IRQn          = 25,     /*!< TMR1 Update Interrupt                                */
    TMR1_TRG_HALL_TMR11_IRQn    = 26,     /*!< TMR1 Trigger and Commutation Interrupt               */
    TMR1_CC_IRQn                = 27,     /*!< TMR1 Capture Compare Interrupt                       */
    TMR2_GLOBAL_IRQn            = 28,     /*!< TMR2 global Interrupt                                */
    TMR3_GLOBAL_IRQn            = 29,     /*!< TMR3 global Interrupt                                */
    TMR4_GLOBAL_IRQn            = 30,     /*!< TMR4 global Interrupt                                */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
    RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
    USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
    TMR8_BRK_TMR12_IRQn         = 43,     /*!< TMR8 Break Interrupt                                 */
    TMR8_OV_TMR13_IRQn          = 44,     /*!< TMR8 Update Interrupt                                */
    TMR8_TRG_HALL_TMR14_IRQn    = 45,     /*!< TMR8 Trigger and Commutation Interrupt               */
    TMR8_CC_IRQn                = 46,     /*!< TMR8 Capture Compare Interrupt                       */
    ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
    XMC_IRQn                    = 48,     /*!< XMC global Interrupt                                 */
    SDIO1_IRQn                  = 49,     /*!< SDIO global Interrupt                                */
    TMR5_GLOBAL_IRQn            = 50,     /*!< TMR5 global Interrupt                                */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
    UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
    UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
    TMR6_GLOBAL_IRQn            = 54,     /*!< TMR6 global Interrupt                                */
    TMR7_GLOBAL_IRQn            = 55,     /*!< TMR7 global Interrupt                                */
    DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
    DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
    DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
    DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
    SDIO2_IRQn                  = 60,     /*!< SDIO2 global Interrupt                               */
    I2C3_EV_IRQn                = 61,     /*!< I2C3 event interrupt                                 */
    I2C3_ER_IRQn                = 62,     /*!< I2C3 error interrupt                                 */
    SPI4_IRQn                   = 63,     /*!< SPI4 global Interrupt                                */
#endif /* AT32F403Vx_HD */


#ifdef AT32F403Vx_XL
    ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
    USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
    TMR1_BRK_TMR9_IRQn          = 24,     /*!< TMR1 Break Interrupt                                 */
    TMR1_OV_TMR10_IRQn          = 25,     /*!< TMR1 Update Interrupt                                */
    TMR1_TRG_HALL_TMR11_IRQn    = 26,     /*!< TMR1 Trigger and Commutation Interrupt               */
    TMR1_CC_IRQn                = 27,     /*!< TMR1 Capture Compare Interrupt                       */
    TMR2_GLOBAL_IRQn            = 28,     /*!< TMR2 global Interrupt                                */
    TMR3_GLOBAL_IRQn            = 29,     /*!< TMR3 global Interrupt                                */
    TMR4_GLOBAL_IRQn            = 30,     /*!< TMR4 global Interrupt                                */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
    RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
    USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
    TMR8_BRK_TMR12_IRQn         = 43,     /*!< TMR8 Break Interrupt                                 */
    TMR8_OV_TMR13_IRQn          = 44,     /*!< TMR8 Update Interrupt                                */
    TMR8_TRG_HALL_TMR14_IRQn    = 45,     /*!< TMR8 Trigger and Commutation Interrupt               */
    TMR8_CC_IRQn                = 46,     /*!< TMR8 Capture Compare Interrupt                       */
    ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
    XMC_IRQn                    = 48,     /*!< XMC global Interrupt                                 */
    SDIO1_IRQn                  = 49,     /*!< SDIO global Interrupt                                */
    TMR5_GLOBAL_IRQn            = 50,     /*!< TMR5 global Interrupt                                */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
    UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
    UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
    TMR6_GLOBAL_IRQn            = 54,     /*!< TMR6 global Interrupt                                */
    TMR7_GLOBAL_IRQn            = 55,     /*!< TMR7 global Interrupt                                */
    DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
    DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
    DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
    DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
    SDIO2_IRQn                  = 60,     /*!< SDIO2 global Interrupt                               */
    I2C3_EV_IRQn                = 61,     /*!< I2C3 event interrupt                                 */
    I2C3_ER_IRQn                = 62,     /*!< I2C3 error interrupt                                 */
    SPI4_IRQn                   = 63,     /*!< SPI4 global Interrupt                                */
#endif /* AT32F403Vx_XL */


#ifdef AT32F403Zx_HD
    ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
    USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
    TMR1_BRK_TMR9_IRQn          = 24,     /*!< TMR1 Break Interrupt                                 */
    TMR1_OV_TMR10_IRQn          = 25,     /*!< TMR1 Update Interrupt                                */
    TMR1_TRG_HALL_TMR11_IRQn    = 26,     /*!< TMR1 Trigger and Commutation Interrupt               */
    TMR1_CC_IRQn                = 27,     /*!< TMR1 Capture Compare Interrupt                       */
    TMR2_GLOBAL_IRQn            = 28,     /*!< TMR2 global Interrupt                                */
    TMR3_GLOBAL_IRQn            = 29,     /*!< TMR3 global Interrupt                                */
    TMR4_GLOBAL_IRQn            = 30,     /*!< TMR4 global Interrupt                                */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
    RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
    USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
    TMR8_BRK_TMR12_IRQn         = 43,     /*!< TMR8 Break Interrupt                                 */
    TMR8_OV_TMR13_IRQn          = 44,     /*!< TMR8 Update Interrupt                                */
    TMR8_TRG_HALL_TMR14_IRQn    = 45,     /*!< TMR8 Trigger and Commutation Interrupt               */
    TMR8_CC_IRQn                = 46,     /*!< TMR8 Capture Compare Interrupt                       */
    ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
    XMC_IRQn                    = 48,     /*!< XMC global Interrupt                                 */
    SDIO1_IRQn                  = 49,     /*!< SDIO global Interrupt                                */
    TMR5_GLOBAL_IRQn            = 50,     /*!< TMR5 global Interrupt                                */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
    UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
    UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
    TMR6_GLOBAL_IRQn            = 54,     /*!< TMR6 global Interrupt                                */
    TMR7_GLOBAL_IRQn            = 55,     /*!< TMR7 global Interrupt                                */
    DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
    DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
    DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
    DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
    SDIO2_IRQn                  = 60,     /*!< SDIO2 global Interrupt                               */
    I2C3_EV_IRQn                = 61,     /*!< I2C3 event interrupt                                 */
    I2C3_ER_IRQn                = 62,     /*!< I2C3 error interrupt                                 */
    SPI4_IRQn                   = 63,     /*!< SPI4 global Interrupt                                */
    TMR15_BRK_IRQn              = 64,     /*!< TMR15 Break interrupt                                */
    TMR15_OV_IRQn               = 65,     /*!< TMR15 Update interrupt                               */
    TMR15_TRG_HALL_IRQn         = 66,     /*!< TMR15 Trigger and Commutation Interrupt              */
    TMR15_CC_IRQn               = 67      /*!< TMR15 Capture Compare Interrupt                      */
#endif /* AT32F403Zx_HD */


#ifdef AT32F403Zx_XL
    ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
    USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
    USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
    CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
    CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
    EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
    TMR1_BRK_TMR9_IRQn          = 24,     /*!< TMR1 Break Interrupt                                 */
    TMR1_OV_TMR10_IRQn          = 25,     /*!< TMR1 Update Interrupt                                */
    TMR1_TRG_HALL_TMR11_IRQn    = 26,     /*!< TMR1 Trigger and Commutation Interrupt               */
    TMR1_CC_IRQn                = 27,     /*!< TMR1 Capture Compare Interrupt                       */
    TMR2_GLOBAL_IRQn            = 28,     /*!< TMR2 global Interrupt                                */
    TMR3_GLOBAL_IRQn            = 29,     /*!< TMR3 global Interrupt                                */
    TMR4_GLOBAL_IRQn            = 30,     /*!< TMR4 global Interrupt                                */
    I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
    I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
    I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
    I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
    SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
    SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
    USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
    USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
    USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
    EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
    RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
    USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
    TMR8_BRK_TMR12_IRQn         = 43,     /*!< TMR8 Break Interrupt                                 */
    TMR8_OV_TMR13_IRQn          = 44,     /*!< TMR8 Update Interrupt                                */
    TMR8_TRG_HALL_TMR14_IRQn    = 45,     /*!< TMR8 Trigger and Commutation Interrupt               */
    TMR8_CC_IRQn                = 46,     /*!< TMR8 Capture Compare Interrupt                       */
    ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
    XMC_IRQn                    = 48,     /*!< XMC global Interrupt                                 */
    SDIO1_IRQn                  = 49,     /*!< SDIO global Interrupt                                */
    TMR5_GLOBAL_IRQn            = 50,     /*!< TMR5 global Interrupt                                */
    SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
    UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
    UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
    TMR6_GLOBAL_IRQn            = 54,     /*!< TMR6 global Interrupt                                */
    TMR7_GLOBAL_IRQn            = 55,     /*!< TMR7 global Interrupt                                */
    DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
    DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
    DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
    DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
    SDIO2_IRQn                  = 60,     /*!< SDIO2 global Interrupt                               */
    I2C3_EV_IRQn                = 61,     /*!< I2C3 event interrupt                                 */
    I2C3_ER_IRQn                = 62,     /*!< I2C3 error interrupt                                 */
    SPI4_IRQn                   = 63,     /*!< SPI4 global Interrupt                                */
    TMR15_BRK_IRQn              = 64,     /*!< TMR15 Break interrupt                                */
    TMR15_OV_IRQn               = 65,     /*!< TMR15 Update interrupt                               */
    TMR15_TRG_HALL_IRQn         = 66,     /*!< TMR15 Trigger and Commutation Interrupt              */
    TMR15_CC_IRQn               = 67      /*!< TMR15 Capture Compare Interrupt                      */
#endif /* AT32F403Zx_XL */
} IRQn_Type;

/**
  * @}
  */

#include "core_cm4.h"
#include "system_at32f4xx.h"
#include <stdint.h>

/** @addtogroup Exported_types
  * @{
  */

typedef int32_t  INT32;
typedef int16_t  INT16;
typedef int8_t   INT8;
typedef uint32_t UINT32;
typedef uint16_t UINT16;
typedef uint8_t  UINT8;





/** at32f4xx Standard Peripheral Library old types (maintained for legacy purpose) */
typedef int32_t  s32;
typedef int16_t  s16;
typedef int8_t   s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t  sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t  vsc8;   /*!< Read Only */

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t  uc8;   /*!< Read Only */

typedef __IO uint32_t vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t  vuc8;   /*!< Read Only */

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/** at32f4xx Standard Peripheral Library old definitions (maintained for legacy purpose) */
#define HSEStartUp_TimeOut   HSE_STARTUP_TIMEOUT
#define HSE_Value            HSE_VALUE
#define HSI_Value            HSI_VALUE
/**
  * @}
  */

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief Analog to Digital Converter
  */
typedef struct
{
    __IO uint32_t STS;
    __IO uint32_t CTRL1;
    __IO uint32_t CTRL2;
    __IO uint32_t SMPT1;
    __IO uint32_t SMPT2;
    __IO uint32_t JOFS1;
    __IO uint32_t JOFS2;
    __IO uint32_t JOFS3;
    __IO uint32_t JOFS4;
    __IO uint32_t WHTR;
    __IO uint32_t WLTR;
    __IO uint32_t RSQ1;
    __IO uint32_t RSQ2;
    __IO uint32_t RSQ3;
    __IO uint32_t JSQ;
    __IO uint32_t JDOR1;
    __IO uint32_t JDOR2;
    __IO uint32_t JDOR3;
    __IO uint32_t JDOR4;
    __IO uint32_t RDOR;
} ADC_Type;

//Copied definition from stm32 file
typedef struct
{
  __IO uint32_t SR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SMPR1;
  __IO uint32_t SMPR2;
  __IO uint32_t JOFR1;
  __IO uint32_t JOFR2;
  __IO uint32_t JOFR3;
  __IO uint32_t JOFR4;
  __IO uint32_t HTR;
  __IO uint32_t LTR;
  __IO uint32_t SQR1;
  __IO uint32_t SQR2;
  __IO uint32_t SQR3;
  __IO uint32_t JSQR;
  __IO uint32_t JDR1;
  __IO uint32_t JDR2;
  __IO uint32_t JDR3;
  __IO uint32_t JDR4;
  __IO uint32_t DR;
} ADC_TypeDef;
//#define ADC_TypeDef ADC_Type

/**
  * @brief Backup Registers
  */
typedef struct
{
    uint32_t  RESERVED0;
    __IO uint16_t DT1;
    uint16_t  RESERVED1;
    __IO uint16_t DT2;
    uint16_t  RESERVED2;
    __IO uint16_t DT3;
    uint16_t  RESERVED3;
    __IO uint16_t DT4;
    uint16_t  RESERVED4;
    __IO uint16_t DT5;
    uint16_t  RESERVED5;
    __IO uint16_t DT6;
    uint16_t  RESERVED6;
    __IO uint16_t DT7;
    uint16_t  RESERVED7;
    __IO uint16_t DT8;
    uint16_t  RESERVED8;
    __IO uint16_t DT9;
    uint16_t  RESERVED9;
    __IO uint16_t DT10;
    uint16_t  RESERVED10;
    __IO uint16_t RTCCAL;
    uint16_t  RESERVED11;
    __IO uint16_t CTRL;
    uint16_t  RESERVED12;
    __IO uint16_t CTRLSTS;
    uint16_t  RESERVED13[5];
    __IO uint16_t DT11;
    uint16_t  RESERVED14;
    __IO uint16_t DT12;
    uint16_t  RESERVED15;
    __IO uint16_t DT13;
    uint16_t  RESERVED16;
    __IO uint16_t DT14;
    uint16_t  RESERVED17;
    __IO uint16_t DT15;
    uint16_t  RESERVED18;
    __IO uint16_t DT16;
    uint16_t  RESERVED19;
    __IO uint16_t DT17;
    uint16_t  RESERVED20;
    __IO uint16_t DT18;
    uint16_t  RESERVED21;
    __IO uint16_t DT19;
    uint16_t  RESERVED22;
    __IO uint16_t DT20;
    uint16_t  RESERVED23;
    __IO uint16_t DT21;
    uint16_t  RESERVED24;
    __IO uint16_t DT22;
    uint16_t  RESERVED25;
    __IO uint16_t DT23;
    uint16_t  RESERVED26;
    __IO uint16_t DT24;
    uint16_t  RESERVED27;
    __IO uint16_t DT25;
    uint16_t  RESERVED28;
    __IO uint16_t DT26;
    uint16_t  RESERVED29;
    __IO uint16_t DT27;
    uint16_t  RESERVED30;
    __IO uint16_t DT28;
    uint16_t  RESERVED31;
    __IO uint16_t DT29;
    uint16_t  RESERVED32;
    __IO uint16_t DT30;
    uint16_t  RESERVED33;
    __IO uint16_t DT31;
    uint16_t  RESERVED34;
    __IO uint16_t DT32;
    uint16_t  RESERVED35;
    __IO uint16_t DT33;
    uint16_t  RESERVED36;
    __IO uint16_t DT34;
    uint16_t  RESERVED37;
    __IO uint16_t DT35;
    uint16_t  RESERVED38;
    __IO uint16_t DT36;
    uint16_t  RESERVED39;
    __IO uint16_t DT37;
    uint16_t  RESERVED40;
    __IO uint16_t DT38;
    uint16_t  RESERVED41;
    __IO uint16_t DT39;
    uint16_t  RESERVED42;
    __IO uint16_t DT40;
    uint16_t  RESERVED43;
    __IO uint16_t DT41;
    uint16_t  RESERVED44;
    __IO uint16_t DT42;
    uint16_t  RESERVED45;
} BKP_Type;

/**
  * @brief Controller Area Network TxMailBox
  */
typedef struct
{
    __IO uint32_t TMI;
    __IO uint32_t TDT;
    __IO uint32_t TDL;
    __IO uint32_t TDH;
} CAN_TxMailBox_Type;

/**
  * @brief Controller Area Network FIFOMailBox
  */
typedef struct
{
    __IO uint32_t RFI;
    __IO uint32_t RDT;
    __IO uint32_t RDL;
    __IO uint32_t RDH;
} CAN_FIFOMailBox_Type;

/**
  * @brief Controller Area Network FilterRegister
  */
typedef struct
{
    __IO uint32_t FBR1;
    __IO uint32_t FBR2;
} CAN_FilterRegister_Type;

/**
  * @brief Controller Area Network
  */
typedef struct
{
    __IO uint32_t MCTRL;
    __IO uint32_t MSTS;
    __IO uint32_t TSTS;
    __IO uint32_t RF0;
    __IO uint32_t RF1;
    __IO uint32_t INTEN;
    __IO uint32_t ESTS;
    __IO uint32_t BTMG;
    uint32_t  RESERVED0[88];
    CAN_TxMailBox_Type TxMailBox[3];
    CAN_FIFOMailBox_Type FIFOMailBox[2];
    uint32_t  RESERVED1[12];
    __IO uint32_t FM;
    __IO uint32_t FM1;
    uint32_t  RESERVED2;
    __IO uint32_t FS1;
    uint32_t  RESERVED3;
    __IO uint32_t FFA1;
    uint32_t  RESERVED4;
    __IO uint32_t FA1;
    uint32_t  RESERVED5[8];
    CAN_FilterRegister_Type FilterRegister[14];
} CAN_Type;

/**
  * @brief CRC calculation unit
  */
typedef struct
{
    __IO uint32_t DT;
    __IO uint8_t  IDT;
    uint8_t   RESERVED0;
    uint16_t  RESERVED1;
    __IO uint32_t CTRL;
} CRC_Type;

/**
  * @brief Digital to Analog Converter
  */
typedef struct
{
    __IO uint32_t CTRL;
    __IO uint32_t SWTRG;
    __IO uint32_t HDR12R1;
    __IO uint32_t HDR12L1;
    __IO uint32_t HDR8R1;
    __IO uint32_t HDR12R2;
    __IO uint32_t HDR12L2;
    __IO uint32_t HDR8R2;
    __IO uint32_t HDR12RD;
    __IO uint32_t HDR12LD;
    __IO uint32_t HDR8RD;
    __IO uint32_t ODT1;
    __IO uint32_t ODT2;
} DAC_Type;

/**
  * @brief MCU Debug
  */
typedef struct
{
//changed to match HAl names
 //   __IO uint32_t IDCR;
 //   __IO uint32_t CTRL;
  __IO uint32_t IDCODE;
  __IO uint32_t CR;
} MCUDBG_Type;

#define DBGMCU_TypeDef MCUDBG_Type

/**
  * @brief DMA Controller
  */
typedef struct
{
//changed to match HAL names
    /*__IO uint32_t CHCTRL;
    __IO uint32_t TCNT;
    __IO uint32_t CPBA;
    __IO uint32_t CMBA;*/
	__IO uint32_t CCR;
	__IO uint32_t CNDTR;
	__IO uint32_t CPAR;
	__IO uint32_t CMAR;
} DMA_Channel_Type;

#define DMA_Channel_TypeDef  DMA_Channel_Type

typedef struct
{
//changed to match HAL names
    //__IO uint32_t ISTS;
    __IO uint32_t ISR;
    //__IO uint32_t ICLR;
    __IO uint32_t IFCR;
} DMA_Type;
#define DMA_TypeDef DMA_Type

/**
  * @brief External Interrupt/Event Controller
  */
typedef struct
{
//changed to match HAL names
    /*__IO uint32_t INTEN;
    __IO uint32_t EVTEN;
    __IO uint32_t RTRSEL;
    __IO uint32_t FTRSEL;
    __IO uint32_t SWIE;
    __IO uint32_t PND;*/
	__IO uint32_t IMR;
	__IO uint32_t EMR;
	__IO uint32_t RTSR;
	__IO uint32_t FTSR;
	__IO uint32_t SWIER;
	__IO uint32_t PR;
} EXTI_Type;

/**
  * @brief FLASH Registers
  */
typedef struct
{
//Changed this to match STM32 HAL names
 //   __IO uint32_t ACR;
 //   __IO uint32_t FCKEY;
 //   __IO uint32_t OPTKEYR;
 //   __IO uint32_t STS;
 //   __IO uint32_t CTRL;
 //   __IO uint32_t ADDR;
 //   __IO uint32_t RESERVED11;
 //   __IO uint32_t UOB;
 //   __IO uint32_t WRPRT;
	__IO uint32_t ACR;
    __IO uint32_t KEYR;
    __IO uint32_t OPTKEYR;
    __IO uint32_t SR;
    __IO uint32_t CR;
    __IO uint32_t AR;
    __IO uint32_t RESERVED11;
    __IO uint32_t OBR;
    __IO uint32_t WRPR;
    uint32_t RESERVED20[8];
    __IO uint32_t FCKEY2;
    uint32_t RESERVED21;
    __IO uint32_t STS2;
    __IO uint32_t CTRL2;
    __IO uint32_t ADDR2;
    uint32_t RESERVED30[11];
    __IO uint32_t FCKEY3;
    __IO uint32_t B3SEL;
    __IO uint32_t STS3;
    __IO uint32_t CTRL3;
    __IO uint32_t ADDR3;
    __IO uint32_t DA;
} FLASH_Type;

/**
  * @brief Option Bytes Registers
  */
typedef struct
{
    __IO uint16_t RDPRT;
    __IO uint16_t USR;
    __IO uint16_t DATA0;
    __IO uint16_t DATA1;
    __IO uint16_t WRPRT0;
    __IO uint16_t WRPRT1;
    __IO uint16_t WRPRT2;
    __IO uint16_t WRPRT3;
    __IO uint16_t EOPB0;
    __IO uint16_t EOPB1;
    __IO uint16_t HID[4];
    __IO uint32_t Reserved;
    __IO uint32_t BANK3SCRKEY[4];
} UOB_Type;

/**
  * @brief Flexible Static Memory Controller
  */
typedef struct
{
    __IO uint32_t BK1CTRLR[8];
} XMC_Bank1_Type;

/**
  * @brief Flexible Static Memory Controller Bank1E
  */
typedef struct
{
    __IO uint32_t BK1TMGWR[7];
} XMC_Bank1Ext_Type;

/**
  * @brief Flexible Static Memory Controller Bank2
  */
typedef struct
{
    __IO uint32_t BK2CTRL;
    __IO uint32_t BK2STS;
    __IO uint32_t BK2TMGMEM;
    __IO uint32_t BK2TMGATT;
    uint32_t  RESERVED0;
    __IO uint32_t BK2ECC;
} XMC_Bank2_Type;

/**
  * @brief Flexible Static Memory Controller Bank3
  */
typedef struct
{
    __IO uint32_t BK3CTRL;
    __IO uint32_t BK3STS;
    __IO uint32_t BK3TMGMEM;
    __IO uint32_t BK3TMGATT;
    uint32_t  RESERVED0;
    __IO uint32_t BK3ECC;
} XMC_Bank3_Type;

/**
  * @brief Flexible Static Memory Controller Bank4
  */
typedef struct
{
    __IO uint32_t BK4CTRL;
    __IO uint32_t BK4STS;
    __IO uint32_t BK4TMGMEM;
    __IO uint32_t BK4TMGATT;
    __IO uint32_t BK4TMGIO;
} XMC_Bank4_Type;

/**
  * @brief General Purpose I/O
  */
typedef struct
{
//changed to match HAL names
    /*__IO uint32_t CTRLL;
    __IO uint32_t CTRLH;
    __IO uint32_t IPTDT;
    __IO uint32_t OPTDT;
    __IO uint32_t BSRE;
    __IO uint32_t BRE;
    __IO uint32_t LOCK;*/
	__IO uint32_t CRL;
	__IO uint32_t CRH;
	__IO uint32_t IDR;
	__IO uint32_t ODR;
	__IO uint32_t BSRR;
	__IO uint32_t BRR;
	__IO uint32_t LCKR;
} GPIO_Type;

#define GPIO_TypeDef GPIO_Type

/**
  * @brief Alternate Function I/O
  */
typedef struct
{
	//changed to match stm32 hal
    //__IO uint32_t EVCTRL;
    //__IO uint32_t MAP;
    //__IO uint32_t EXTIC[4];
    //uint32_t RESERVED0;
    //__IO uint32_t MAP2;
	__IO uint32_t EVCR;
	__IO uint32_t MAPR;
	__IO uint32_t EXTICR[4];
	uint32_t RESERVED0;
	__IO uint32_t MAPR2; 
} AFIO_Type;

#define AFIO_TypeDef AFIO_Type
/**
  * @brief Inter Integrated Circuit Interface
  */
// typedef struct
// {
// //changed to match HAL names
   // /* __IO uint16_t CTRL1;
    // uint16_t  RESERVED0;
    // __IO uint16_t CTRL2;
    // uint16_t  RESERVED1;
    // __IO uint16_t OADDR1;
    // uint16_t  RESERVED2;
    // __IO uint16_t OADDR2;
    // uint16_t  RESERVED3;
    // __IO uint16_t DT;
    // uint16_t  RESERVED4;
    // __IO uint16_t STS1;
    // uint16_t  RESERVED5;
    // __IO uint16_t STS2;
    // uint16_t  RESERVED6;
    // __IO uint16_t CLKCTRL;
    // uint16_t  RESERVED7;
    // __IO uint16_t TMRISE;
    // uint16_t  RESERVED8;*/
	// __IO uint16_t CR1;
	// uint16_t  RESERVED0;
	// __IO uint16_t CR2;
	// uint16_t  RESERVED1;
	// __IO uint16_t OAR1;
	// uint16_t  RESERVED2;
	// __IO uint16_t OAR2;
	// uint16_t  RESERVED3;
	// __IO uint16_t DR;
	// uint16_t  RESERVED4;
	// __IO uint16_t SR1;
	// uint16_t  RESERVED5;
	// __IO uint16_t SR2;
	// uint16_t  RESERVED6;
	// __IO uint16_t CCR;
	// uint16_t  RESERVED7;
	// __IO uint16_t TRISE;
	// uint16_t  RESERVED8;
// } I2C_Type;
typedef struct
{
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t OAR1;
  __IO uint32_t OAR2;
  __IO uint32_t DR;
  __IO uint32_t SR1;
  __IO uint32_t SR2;
  __IO uint32_t CCR;
  __IO uint32_t TRISE;
} I2C_Type;
#define I2C_TypeDef I2C_Type

/**
  * @brief Independent WATCHDOG
  */
typedef struct
{
    __IO uint32_t KEY;
    __IO uint32_t PSC;
    __IO uint32_t RLD;
    __IO uint32_t STS;
} IWDG_Type;

/**
  * @brief Power Control
  */
typedef struct
{
//changed to amtch STM32 hal
    //__IO uint32_t CTRL;
    __IO uint32_t CR;
    __IO uint32_t CTRLSTS;
} PWR_Type;

/**
  * @brief Reset and Clock Control
  */
typedef struct
{
    //__IO uint32_t CTRL;
	__IO uint32_t CR;
    __IO uint32_t CFGR;
	//renamed to match STM32 HAL
    //__IO uint32_t CLKINT;
	__IO uint32_t CIR;
    //__IO uint32_t APB2RST;
    __IO uint32_t APB2RSTR;
    //__IO uint32_t APB1RST;
    __IO uint32_t APB1RSTR;
    //__IO uint32_t AHBEN;
    __IO uint32_t AHBENR;
    //__IO uint32_t APB2EN;
    __IO uint32_t APB2ENR;
    //__IO uint32_t APB1EN;
    __IO uint32_t APB1ENR;
    //__IO uint32_t BDC;
    __IO uint32_t BDCR;
    //__IO uint32_t CTRLSTS;
    __IO uint32_t CSR;
    __IO uint32_t RESERVED[2];
    __IO uint32_t MISC;
} RCC_Type;

/**
  * @brief Real-Time Clock
  */
typedef struct
{
    __IO uint16_t CTRLH;
    uint16_t  RESERVED0;
    __IO uint16_t CTRLL;
    uint16_t  RESERVED1;
    __IO uint16_t DIVH;
    uint16_t  RESERVED2;
    __IO uint16_t DIVL;
    uint16_t  RESERVED3;
    __IO uint16_t DIVCNTH;
    uint16_t  RESERVED4;
    __IO uint16_t DIVCNTL;
    uint16_t  RESERVED5;
    __IO uint16_t CNTH;
    uint16_t  RESERVED6;
    __IO uint16_t CNTL;
    uint16_t  RESERVED7;
    __IO uint16_t ALAH;
    uint16_t  RESERVED8;
    __IO uint16_t ALAL;
    uint16_t  RESERVED9;
} RTC_Type;

/**
  * @brief SD host Interface
  */
typedef struct
{
    __IO uint32_t POWER;
    __IO uint32_t CLKCTRL;
    __IO uint32_t ARG;
    __IO uint32_t CMD;
    __I uint32_t RSPCMD;
    __I uint32_t RSP1;
    __I uint32_t RSP2;
    __I uint32_t RSP3;
    __I uint32_t RSP4;
    __IO uint32_t DTTMR;
    __IO uint32_t DTLEN;
    __IO uint32_t DTCTRL;
    __I uint32_t DTCNTR;
    __I uint32_t STS;
    __IO uint32_t INTCLR;
    __IO uint32_t INTEN;
    uint32_t  RESERVED0[2];
    __I uint32_t BUFCNTR;
    uint32_t  RESERVED1[13];
    __IO uint32_t BUF;
} SDIO_Type;

/**
  * @brief Serial Peripheral Interface
  */
// typedef struct
// {
    // __IO uint16_t CTRL1;
    // uint16_t  RESERVED0;
    // __IO uint16_t CTRL2;
    // uint16_t  RESERVED1;
    // __IO uint16_t STS;
    // uint16_t  RESERVED2;
    // __IO uint16_t DT;
    // uint16_t  RESERVED3;
    // __IO uint16_t CPOLY;
    // uint16_t  RESERVED4;
    // __IO uint16_t RCRC;
    // uint16_t  RESERVED5;
    // __IO uint16_t TCRC;
    // uint16_t  RESERVED6;
    // __IO uint16_t I2SCTRL;
    // uint16_t  RESERVED7;
    // __IO uint16_t I2SCLKP;
    // uint16_t  RESERVED8;
// } SPI_Type;

typedef struct
{
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SR;
  __IO uint32_t DR;
  __IO uint32_t CRCPR;
  __IO uint32_t RXCRCR;
  __IO uint32_t TXCRCR;
  __IO uint32_t I2SCFGR;
  __IO uint32_t I2SPR;
} SPI_Type;

#define SPI_TypeDef SPI_Type

/**
  * @brief TIMER
  */
typedef struct
{
    __IO uint16_t CTRL1;
    uint16_t  RESERVED0;
    __IO uint16_t CTRL2;
    uint16_t  RESERVED1;
    __IO uint16_t SMC;
    uint16_t  RESERVED2;
    __IO uint16_t DIE;
    uint16_t  RESERVED3;
    __IO uint16_t STS;
    uint16_t  RESERVED4;
    __IO uint16_t EVEG;
    uint16_t  RESERVED5;
    __IO uint16_t CCM1;
    uint16_t  RESERVED6;
    __IO uint16_t CCM2;
    uint16_t  RESERVED7;
    __IO uint16_t CER;
    uint16_t  RESERVED8;
	__IO uint32_t CNT;
    __IO uint16_t DIV;
    uint16_t  RESERVED10;
	__IO uint32_t AR;
    __IO uint16_t RC;
    uint16_t  RESERVED12;
    __IO uint32_t CC1;
    __IO uint32_t CC2;
    __IO uint32_t CC3;
    __IO uint32_t CC4;
    __IO uint16_t BRKDT;
    uint16_t  RESERVED17;
    __IO uint16_t DMAC;
    uint16_t  RESERVED18;
    __IO uint16_t DMABA;
    uint16_t  RESERVED19;
} TMR_Type;

//added the complete tim_typeDef and removed the alias, these two should match up
//The registers mostly have the upper 16 bit free
//ST did not care and defined everything as 32-bit registers
//The chinese however use 16-bit registers and reserved values
//So be carefull
typedef struct
{
  __IO uint32_t CR1;             /*!< TIM control register 1,                      Address offset: 0x00 */
  __IO uint32_t CR2;             /*!< TIM control register 2,                      Address offset: 0x04 */
  __IO uint32_t SMCR;            /*!< TIM slave Mode Control register,             Address offset: 0x08 */
  __IO uint32_t DIER;            /*!< TIM DMA/interrupt enable register,           Address offset: 0x0C */
  __IO uint32_t SR;              /*!< TIM status register,                         Address offset: 0x10 */
  __IO uint32_t EGR;             /*!< TIM event generation register,               Address offset: 0x14 */
  __IO uint32_t CCMR1;           /*!< TIM  capture/compare mode register 1,        Address offset: 0x18 */
  __IO uint32_t CCMR2;           /*!< TIM  capture/compare mode register 2,        Address offset: 0x1C */
  __IO uint32_t CCER;            /*!< TIM capture/compare enable register,         Address offset: 0x20 */
  __IO uint32_t CNT;             /*!< TIM counter register,                        Address offset: 0x24 */
  __IO uint32_t PSC;             /*!< TIM prescaler register,                      Address offset: 0x28 */
  __IO uint32_t ARR;             /*!< TIM auto-reload register,                    Address offset: 0x2C */
  __IO uint32_t RCR;             /*!< TIM  repetition counter register,            Address offset: 0x30 */
  __IO uint32_t CCR1;            /*!< TIM capture/compare register 1,              Address offset: 0x34 */
  __IO uint32_t CCR2;            /*!< TIM capture/compare register 2,              Address offset: 0x38 */
  __IO uint32_t CCR3;            /*!< TIM capture/compare register 3,              Address offset: 0x3C */
  __IO uint32_t CCR4;            /*!< TIM capture/compare register 4,              Address offset: 0x40 */
  __IO uint32_t BDTR;            /*!< TIM break and dead-time register,            Address offset: 0x44 */
  __IO uint32_t DCR;             /*!< TIM DMA control register,                    Address offset: 0x48 */
  __IO uint32_t DMAR;            /*!< TIM DMA address for full transfer register,  Address offset: 0x4C */
 // __IO uint32_t OR;              /*!< TIM option register,                         Address offset: 0x50 */
}TIM_TypeDef;

//Added alias
//#define TIM_TypeDef TMR_Type

/**
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
typedef struct
{
//changed to match HAL names
    /*__IO uint16_t STS;
    uint16_t  RESERVED0;
    __IO uint16_t DT;
    uint16_t  RESERVED1;
    __IO uint16_t BAUDR;
    uint16_t  RESERVED2;
    __IO uint16_t CTRL1;
    uint16_t  RESERVED3;
    __IO uint16_t CTRL2;
    uint16_t  RESERVED4;
    __IO uint16_t CTRL3;
    uint16_t  RESERVED5;
    __IO uint16_t GTP;
    uint16_t  RESERVED6;*/
	__IO uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
	__IO uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
	__IO uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
	__IO uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
	__IO uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
	__IO uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
	__IO uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_Type;

#define USART_TypeDef USART_Type

/**
  * @brief Window WATCHDOG
  */
typedef struct
{
    __IO uint32_t CTRL;
    __IO uint32_t CFG;
    __IO uint32_t STS;
} WWDG_Type;

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */

#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH base address in the alias region */
#define FLASH_BANK1_END       ((uint32_t)0x0803FFFF)/*!< FLASH END address of bank1 */
#define EXT_FLASH_BASE        ((uint32_t)0x08400000) /*!< External FLASH base address in the alias region */
#define SRAM_BASE             ((uint32_t)0x20000000) /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE          ((uint32_t)0x22000000) /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region */

//////////////////////////////////////////////////////////////////
// copied from stm file
// It seems the AT has no uniqe ID, maybe this call will cause a hard fault, which would 
// be a pitty
#define UID_BASE              0x1FFFF7E8U    /*!< Unique device ID register base address */
//////////////////////////////////////////////////////////////////

#define XMC_R_BASE            ((uint32_t)0xA0000000) /*!< XMC registers base address */

/** Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define TMR2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TMR3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TMR4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TMR5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TMR6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TMR7_BASE             (APB1PERIPH_BASE + 0x1400)
#define TMR12_BASE            (APB1PERIPH_BASE + 0x1800)
#define TMR13_BASE            (APB1PERIPH_BASE + 0x1C00)
#define TMR14_BASE            (APB1PERIPH_BASE + 0x2000)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#define SPI4_BASE             (APB1PERIPH_BASE + 0x4000)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x6800)
#define BKP_BASE              (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400)

#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE            (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE            (APB2PERIPH_BASE + 0x2000)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800)
#define TMR1_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define TMR8_BASE             (APB2PERIPH_BASE + 0x3400)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x3C00)
#define TMR15_BASE            (APB2PERIPH_BASE + 0x4000)
#define TMR9_BASE             (APB2PERIPH_BASE + 0x4C00)
#define TMR10_BASE            (APB2PERIPH_BASE + 0x5000)
#define TMR11_BASE            (APB2PERIPH_BASE + 0x5400)

#define SDIO1_BASE            (PERIPH_BASE + 0x18000)
#define SDIO2_BASE            (AHBPERIPH_BASE + 0x3400)


#define DMA1_BASE             (AHBPERIPH_BASE + 0x0000)
#define DMA1_Channel1_BASE    (AHBPERIPH_BASE + 0x0008)
#define DMA1_Channel2_BASE    (AHBPERIPH_BASE + 0x001C)
#define DMA1_Channel3_BASE    (AHBPERIPH_BASE + 0x0030)
#define DMA1_Channel4_BASE    (AHBPERIPH_BASE + 0x0044)
#define DMA1_Channel5_BASE    (AHBPERIPH_BASE + 0x0058)
#define DMA1_Channel6_BASE    (AHBPERIPH_BASE + 0x006C)
#define DMA1_Channel7_BASE    (AHBPERIPH_BASE + 0x0080)
#define DMA2_BASE             (AHBPERIPH_BASE + 0x0400)
#define DMA2_Channel1_BASE    (AHBPERIPH_BASE + 0x0408)
#define DMA2_Channel2_BASE    (AHBPERIPH_BASE + 0x041C)
#define DMA2_Channel3_BASE    (AHBPERIPH_BASE + 0x0430)
#define DMA2_Channel4_BASE    (AHBPERIPH_BASE + 0x0444)
#define DMA2_Channel5_BASE    (AHBPERIPH_BASE + 0x0458)
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)
#define CRC_BASE              (AHBPERIPH_BASE + 0x3000)

#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x2000) /*!< Flash registers base address */
#define UOB_BASE              ((uint32_t)0x1FFFF800)    /*!< Flash Option Bytes base address */

#define XMC_Bank1_R_BASE      (XMC_R_BASE + 0x0000) /*!< XMC Bank1 registers base address */
#define XMC_Bank1E_R_BASE     (XMC_R_BASE + 0x0104) /*!< XMC Bank1E registers base address */
#define XMC_Bank2_R_BASE      (XMC_R_BASE + 0x0060) /*!< XMC Bank2 registers base address */
#define XMC_Bank3_R_BASE      (XMC_R_BASE + 0x0080) /*!< XMC Bank3 registers base address */
#define XMC_Bank4_R_BASE      (XMC_R_BASE + 0x00A0) /*!< XMC Bank4 registers base address */

#define DBGMCU_BASE           ((uint32_t)0xE0042000) /*!< Debug MCU registers base address */

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */

#define AT_TMR2                ((TMR_Type *) TMR2_BASE)
#define AT_TMR3                ((TMR_Type *) TMR3_BASE)
#define AT_TMR4                ((TMR_Type *) TMR4_BASE)
#define AT_TMR5                ((TMR_Type *) TMR5_BASE)
#define AT_TMR6                ((TMR_Type *) TMR6_BASE)
#define AT_TMR7                ((TMR_Type *) TMR7_BASE)
#define AT_TMR12               ((TMR_Type *) TMR12_BASE)
#define AT_TMR13               ((TMR_Type *) TMR13_BASE)
#define AT_TMR14               ((TMR_Type *) TMR14_BASE)
#define TIM2                ((TIM_TypeDef *) TMR2_BASE)
#define TIM3                ((TIM_TypeDef *) TMR3_BASE)
#define TIM4                ((TIM_TypeDef *) TMR4_BASE)
#define TIM5                ((TIM_TypeDef *) TMR5_BASE)
#define TIM6                ((TIM_TypeDef *) TMR6_BASE)
#define TIM7                ((TIM_TypeDef *) TMR7_BASE)
#define TIM12               ((TIM_TypeDef *) TMR12_BASE)
#define TIM13               ((TIM_TypeDef *) TMR13_BASE)
#define TIM14               ((TIM_TypeDef *) TMR14_BASE)
#define RTC                 ((RTC_Type *) RTC_BASE)
#define WWDG                ((WWDG_Type *) WWDG_BASE)
#define IWDG                ((IWDG_Type *) IWDG_BASE)
#define SPI2                ((SPI_Type *) SPI2_BASE)
#define SPI3                ((SPI_Type *) SPI3_BASE)
#define SPI4                ((SPI_Type *) SPI4_BASE)
#define USART2              ((USART_Type *) USART2_BASE)
#define USART3              ((USART_Type *) USART3_BASE)
#define UART4               ((USART_Type *) UART4_BASE)
#define UART5               ((USART_Type *) UART5_BASE)
#define I2C1                ((I2C_Type *) I2C1_BASE)
#define I2C2                ((I2C_Type *) I2C2_BASE)
#define I2C3                ((I2C_Type *) I2C3_BASE)
#define CAN1                ((CAN_Type *) CAN1_BASE)
#define BKP                 ((BKP_Type *) BKP_BASE)
#define PWR                 ((PWR_Type *) PWR_BASE)
#define DAC                 ((DAC_Type *) DAC_BASE)
#define AFIO                ((AFIO_Type *) AFIO_BASE)
#define EXTI                ((EXTI_Type *) EXTI_BASE)
#define GPIOA               ((GPIO_Type *) GPIOA_BASE)
#define GPIOB               ((GPIO_Type *) GPIOB_BASE)
#define GPIOC               ((GPIO_Type *) GPIOC_BASE)
#define GPIOD               ((GPIO_Type *) GPIOD_BASE)
#define GPIOE               ((GPIO_Type *) GPIOE_BASE)
#define GPIOF               ((GPIO_Type *) GPIOF_BASE)
#define GPIOG               ((GPIO_Type *) GPIOG_BASE)
//Incompatible type definitions as we have both, I think it is better to remove one, but
//that can be done later
#define AT_ADC1                ((ADC_Type *) ADC1_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define AT_ADC2                ((ADC_Type *) ADC2_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define TMR1                ((TMR_Type *) TMR1_BASE)
#define TIM1                ((TIM_TypeDef *) TMR1_BASE)
#define SPI1                ((SPI_Type *) SPI1_BASE)
#define TMR8                ((TMR_Type *) TMR8_BASE)
#define TIM8                ((TIM_TypeDef *) TMR8_BASE)
#define USART1              ((USART_Type *) USART1_BASE)
#define ADC3                ((ADC_Type *) ADC3_BASE)
#define TMR15               ((TMR_Type *) TMR15_BASE)
#define TMR9                ((TMR_Type *) TMR9_BASE)
#define TMR10               ((TMR_Type *) TMR10_BASE)
#define TMR11               ((TMR_Type *) TMR11_BASE)
#define SDIO1               ((SDIO_Type *) SDIO1_BASE)
#define SDIO2               ((SDIO_Type *) SDIO2_BASE)
#define DMA1                ((DMA_Type *) DMA1_BASE)
#define DMA2                ((DMA_Type *) DMA2_BASE)
#define DMA1_Channel1       ((DMA_Channel_Type *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_Type *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_Type *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_Type *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_Type *) DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_Type *) DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_Type *) DMA1_Channel7_BASE)
#define DMA2_Channel1       ((DMA_Channel_Type *) DMA2_Channel1_BASE)
#define DMA2_Channel2       ((DMA_Channel_Type *) DMA2_Channel2_BASE)
#define DMA2_Channel3       ((DMA_Channel_Type *) DMA2_Channel3_BASE)
#define DMA2_Channel4       ((DMA_Channel_Type *) DMA2_Channel4_BASE)
#define DMA2_Channel5       ((DMA_Channel_Type *) DMA2_Channel5_BASE)
#define RCC                 ((RCC_Type *) RCC_BASE)
#define CRC                 ((CRC_Type *) CRC_BASE)
#define FLASH               ((FLASH_Type *) FLASH_R_BASE)
#define UOPTB               ((UOB_Type *) UOB_BASE)
//Added alias
#define OB UOPTB               
#define XMC_Bank1           ((XMC_Bank1_Type *) XMC_Bank1_R_BASE)
#define XMC_Bank1E          ((XMC_Bank1Ext_Type *) XMC_Bank1E_R_BASE)
#define XMC_Bank2           ((XMC_Bank2_Type *) XMC_Bank2_R_BASE)
#define XMC_Bank3           ((XMC_Bank3_Type *) XMC_Bank3_R_BASE)
#define XMC_Bank4           ((XMC_Bank4_Type *) XMC_Bank4_R_BASE)
#define DBGMCU              ((MCUDBG_Type *) DBGMCU_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */

/** @addtogroup Peripheral_Registers_Bits_Definition
* @{
*/

/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for CRC_DT register  *********************/
#define  CRC_DT_DR                              ((uint32_t)0xFFFFFFFF) /*!< Data register bits */


/*******************  Bit definition for CRC_IDT register  ********************/
#define  CRC_IDT_IDR                            ((uint8_t)0xFF)        /*!< General-purpose 8-bit data register bits */


/********************  Bit definition for CRC_CTRL register  ********************/
#define  CRC_CTRL_RST                           ((uint8_t)0x01)        /*!< RESET bit */

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for PWR_CTRL register  ********************/
#define  PWR_CTRL_PDDS                          ((uint16_t)0x0002)     /*!< Power Down Deepsleep */
#define  PWR_CTRL_CLWUF                           ((uint16_t)0x0004)     /*!< Clear Wakeup Flag */
#define  PWR_CTRL_CLSBF                           ((uint16_t)0x0008)     /*!< Clear Standby Flag */
#define  PWR_CTRL_PVDEN                         ((uint16_t)0x0010)     /*!< Power Voltage Detector Enable */

#define  PWR_CTRL_PVDS                          ((uint16_t)0x00E0)     /*!< PLS[2:0] bits (PVD Level Selection) */
#define  PWR_CTRL_PVDS_0                        ((uint16_t)0x0020)     /*!< Bit 0 */
#define  PWR_CTRL_PVDS_1                        ((uint16_t)0x0040)     /*!< Bit 1 */
#define  PWR_CTRL_PVDS_2                        ((uint16_t)0x0080)     /*!< Bit 2 */

/** PVD level configuration */
#define  PWR_CTRL_PVDS_2V2                      ((uint16_t)0x0000)     /*!< PVD level 2.2V */
#define  PWR_CTRL_PVDS_2V3                      ((uint16_t)0x0020)     /*!< PVD level 2.3V */
#define  PWR_CTRL_PVDS_2V4                      ((uint16_t)0x0040)     /*!< PVD level 2.4V */
#define  PWR_CTRL_PVDS_2V5                      ((uint16_t)0x0060)     /*!< PVD level 2.5V */
#define  PWR_CTRL_PVDS_2V6                      ((uint16_t)0x0080)     /*!< PVD level 2.6V */
#define  PWR_CTRL_PVDS_2V7                      ((uint16_t)0x00A0)     /*!< PVD level 2.7V */
#define  PWR_CTRL_PVDS_2V8                      ((uint16_t)0x00C0)     /*!< PVD level 2.8V */
#define  PWR_CTRL_PVDS_2V9                      ((uint16_t)0x00E0)     /*!< PVD level 2.9V */

#define  PWR_CTRL_DBP                           ((uint16_t)0x0100)     /*!< Disable Backup Domain write protection */
#define PWR_CR_DBP	PWR_CTRL_DBP

//Copied from STM file
/*******************  Bit definition for PWR_CTRLSTS register  ********************/
#define  PWR_CTRLSTS_WUF                        ((uint16_t)0x0001)     /*!< Wakeup Flag */
#define  PWR_CTRLSTS_SBF                        ((uint16_t)0x0002)     /*!< Standby Flag */
#define  PWR_CTRLSTS_PVD                        ((uint16_t)0x0004)     /*!< PVD Output */
#define  PWR_CTRLSTS_WUPEN                      ((uint16_t)0x0100)     /*!< Enable WKUP pin */

/********************  Bit definition for PWR_CR register  ********************/
#define PWR_CR_LPDS_Pos                     (0U)                               
#define PWR_CR_LPDS_Msk                     (0x1U << PWR_CR_LPDS_Pos)          /*!< 0x00000001 */
#define PWR_CR_LPDS                         PWR_CR_LPDS_Msk                    /*!< Low-Power Deepsleep */
#define PWR_CR_PDDS_Pos                     (1U)                               
#define PWR_CR_PDDS_Msk                     (0x1U << PWR_CR_PDDS_Pos)          /*!< 0x00000002 */
#define PWR_CR_PDDS                         PWR_CR_PDDS_Msk                    /*!< Power Down Deepsleep */
#define PWR_CR_CWUF_Pos                     (2U)                               
#define PWR_CR_CWUF_Msk                     (0x1U << PWR_CR_CWUF_Pos)          /*!< 0x00000004 */
#define PWR_CR_CWUF                         PWR_CR_CWUF_Msk                    /*!< Clear Wakeup Flag */
#define PWR_CR_CSBF_Pos                     (3U)                               
#define PWR_CR_CSBF_Msk                     (0x1U << PWR_CR_CSBF_Pos)          /*!< 0x00000008 */
#define PWR_CR_CSBF                         PWR_CR_CSBF_Msk                    /*!< Clear Standby Flag */
#define PWR_CR_PVDE_Pos                     (4U)                               
#define PWR_CR_PVDE_Msk                     (0x1U << PWR_CR_PVDE_Pos)          /*!< 0x00000010 */
#define PWR_CR_PVDE                         PWR_CR_PVDE_Msk                    /*!< Power Voltage Detector Enable */

#define PWR_CR_PLS_Pos                      (5U)                               
#define PWR_CR_PLS_Msk                      (0x7U << PWR_CR_PLS_Pos)           /*!< 0x000000E0 */
#define PWR_CR_PLS                          PWR_CR_PLS_Msk                     /*!< PLS[2:0] bits (PVD Level Selection) */
#define PWR_CR_PLS_0                        (0x1U << PWR_CR_PLS_Pos)           /*!< 0x00000020 */
#define PWR_CR_PLS_1                        (0x2U << PWR_CR_PLS_Pos)           /*!< 0x00000040 */
#define PWR_CR_PLS_2                        (0x4U << PWR_CR_PLS_Pos)           /*!< 0x00000080 */

/*!< PVD level configuration */
#define PWR_CR_PLS_LEV0                      0x00000000U                           /*!< PVD level 2.2V */
#define PWR_CR_PLS_LEV1                      0x00000020U                           /*!< PVD level 2.3V */
#define PWR_CR_PLS_LEV2                      0x00000040U                           /*!< PVD level 2.4V */
#define PWR_CR_PLS_LEV3                      0x00000060U                           /*!< PVD level 2.5V */
#define PWR_CR_PLS_LEV4                      0x00000080U                           /*!< PVD level 2.6V */
#define PWR_CR_PLS_LEV5                      0x000000A0U                           /*!< PVD level 2.7V */
#define PWR_CR_PLS_LEV6                      0x000000C0U                           /*!< PVD level 2.8V */
#define PWR_CR_PLS_LEV7                      0x000000E0U                           /*!< PVD level 2.9V */

/* Legacy defines */
#define PWR_CR_PLS_2V2                       PWR_CR_PLS_LEV0
#define PWR_CR_PLS_2V3                       PWR_CR_PLS_LEV1
#define PWR_CR_PLS_2V4                       PWR_CR_PLS_LEV2
#define PWR_CR_PLS_2V5                       PWR_CR_PLS_LEV3
#define PWR_CR_PLS_2V6                       PWR_CR_PLS_LEV4
#define PWR_CR_PLS_2V7                       PWR_CR_PLS_LEV5
#define PWR_CR_PLS_2V8                       PWR_CR_PLS_LEV6
#define PWR_CR_PLS_2V9                       PWR_CR_PLS_LEV7

#define PWR_CR_DBP_Pos                      (8U)                               
#define PWR_CR_DBP_Msk                      (0x1U << PWR_CR_DBP_Pos)           /*!< 0x00000100 */
//#define PWR_CR_DBP                          PWR_CR_DBP_Msk                     /*!< Disable Backup Domain write protection */


/*******************  Bit definition for PWR_CSR register  ********************/
#define PWR_CSR_WUF_Pos                     (0U)                               
#define PWR_CSR_WUF_Msk                     (0x1U << PWR_CSR_WUF_Pos)          /*!< 0x00000001 */
#define PWR_CSR_WUF                         PWR_CSR_WUF_Msk                    /*!< Wakeup Flag */
#define PWR_CSR_SBF_Pos                     (1U)                               
#define PWR_CSR_SBF_Msk                     (0x1U << PWR_CSR_SBF_Pos)          /*!< 0x00000002 */
#define PWR_CSR_SBF                         PWR_CSR_SBF_Msk                    /*!< Standby Flag */
#define PWR_CSR_PVDO_Pos                    (2U)                               
#define PWR_CSR_PVDO_Msk                    (0x1U << PWR_CSR_PVDO_Pos)         /*!< 0x00000004 */
#define PWR_CSR_PVDO                        PWR_CSR_PVDO_Msk                   /*!< PVD Output */
#define PWR_CSR_EWUP_Pos                    (8U)                               
#define PWR_CSR_EWUP_Msk                    (0x1U << PWR_CSR_EWUP_Pos)         /*!< 0x00000100 */
#define PWR_CSR_EWUP                        PWR_CSR_EWUP_Msk                   /*!< Enable WKUP pin */


/******************************************************************************/
/*                                                                            */
/*                            Backup registers                                */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for BKP_DT1 register  ********************/
#define  BKP_DT1_D                              ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT2 register  ********************/
#define  BKP_DT2_D                              ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT3 register  ********************/
#define  BKP_DT3_D                              ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT4 register  ********************/
#define  BKP_DT4_D                              ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT5 register  ********************/
#define  BKP_DT5_D                              ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT6 register  ********************/
#define  BKP_DT6_D                              ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT7 register  ********************/
#define  BKP_DT7_D                              ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT8 register  ********************/
#define  BKP_DT8_D                              ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT9 register  ********************/
#define  BKP_DT9_D                              ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT10 register  *******************/
#define  BKP_DT10_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT11 register  *******************/
#define  BKP_DT11_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT12 register  *******************/
#define  BKP_DT12_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT13 register  *******************/
#define  BKP_DT13_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT14 register  *******************/
#define  BKP_DT14_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT15 register  *******************/
#define  BKP_DT15_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT16 register  *******************/
#define  BKP_DT16_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT17 register  *******************/
#define  BKP_DT17_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/******************  Bit definition for BKP_DT18 register  ********************/
#define  BKP_DT18_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT19 register  *******************/
#define  BKP_DT19_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT20 register  *******************/
#define  BKP_DT20_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT21 register  *******************/
#define  BKP_DT21_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT22 register  *******************/
#define  BKP_DT22_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT23 register  *******************/
#define  BKP_DT23_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT24 register  *******************/
#define  BKP_DT24_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT25 register  *******************/
#define  BKP_DT25_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT26 register  *******************/
#define  BKP_DT26_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT27 register  *******************/
#define  BKP_DT27_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT28 register  *******************/
#define  BKP_DT28_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT29 register  *******************/
#define  BKP_DT29_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT30 register  *******************/
#define  BKP_DT30_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT31 register  *******************/
#define  BKP_DT31_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT32 register  *******************/
#define  BKP_DT32_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT33 register  *******************/
#define  BKP_DT33_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT34 register  *******************/
#define  BKP_DT34_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT35 register  *******************/
#define  BKP_DT35_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT36 register  *******************/
#define  BKP_DT36_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT37 register  *******************/
#define  BKP_DT37_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT38 register  *******************/
#define  BKP_DT38_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT39 register  *******************/
#define  BKP_DT39_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT40 register  *******************/
#define  BKP_DT40_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT41 register  *******************/
#define  BKP_DT41_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DT42 register  *******************/
#define  BKP_DT42_D                             ((uint16_t)0xFFFF)     /*!< Backup data */

/******************  Bit definition for BKP_RTCCAL register  *******************/
#define  BKP_RTCCAL_CAL                         ((uint16_t)0x007F)     /*!< Calibration value */
#define  BKP_RTCCAL_OT1CAL                      ((uint16_t)0x0080)     /*!< Calibration Clock Output */
#define  BKP_RTCCAL_OT2EN                       ((uint16_t)0x0100)     /*!< Alarm or Second Output Enable */
#define  BKP_RTCCAL_OT2SEL                      ((uint16_t)0x0200)     /*!< Alarm or Second Output Selection */

/********************  Bit definition for BKP_CTRL register  ********************/
#define  BKP_CTRL_TPEN                          ((uint8_t)0x01)        /*!< TAMPER pin enable */
#define  BKP_CTRL_TPALV                         ((uint8_t)0x02)        /*!< TAMPER pin active level */

/*******************  Bit definition for BKP_CTRLSTS register  ********************/
#define  BKP_CTRLSTS_CTPEF                      ((uint16_t)0x0001)     /*!< Clear Tamper event */
#define  BKP_CTRLSTS_CTPIF                      ((uint16_t)0x0002)     /*!< Clear Tamper Interrupt */
#define  BKP_CTRLSTS_TPIEN                      ((uint16_t)0x0004)     /*!< TAMPER Pin interrupt enable */
#define  BKP_CTRLSTS_TPEF                       ((uint16_t)0x0100)     /*!< Tamper Event Flag */
#define  BKP_CTRLSTS_TPIF                       ((uint16_t)0x0200)     /*!< Tamper Interrupt Flag */

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

//#define APBPrescTable APBPscTable
//#define AHBPrescTable AHBPscTable

/********************  Bit definition for RCC_CTRL register  ********************/
#define  RCC_CTRL_HSIEN                         ((uint32_t)0x00000001)        /*!< Internal High Speed clock enable */
#define  RCC_CR_HSION							RCC_CTRL_HSIEN
#define  RCC_CTRL_HSISTBL                       ((uint32_t)0x00000002)        /*!< Internal High Speed clock ready flag */
#define  RCC_CTRL_HSITWK                        ((uint32_t)0x000000F8)        /*!< Internal High Speed clock trimming */
#define  RCC_CTRL_HSICAL                        ((uint32_t)0x0000FF00)        /*!< Internal High Speed clock Calibration */
#define  RCC_CR_HSITRIM							RCC_CTRL_HSICAL
#define  RCC_CTRL_HSEEN                         ((uint32_t)0x00010000)        /*!< External High Speed clock enable */
#define  RCC_CR_HSEON							RCC_CTRL_HSEEN
#define  RCC_CTRL_HSESTBL                       ((uint32_t)0x00020000)        /*!< External High Speed clock ready flag */
#define  RCC_CTRL_HSEBYPS                       ((uint32_t)0x00040000)        /*!< External High Speed clock Bypass */
#define  RCC_CR_HSEBYP							RCC_CTRL_HSEBYPS
#define  RCC_CTRL_HSECFDEN                      ((uint32_t)0x00080000)        /*!< Clock Security System enable */
#define  RCC_CTRL_PLLEN                         ((uint32_t)0x01000000)        /*!< PLL enable */
#define  RCC_CR_PLLON							RCC_CTRL_PLLEN
#define  RCC_CTRL_PLLSTBL                       ((uint32_t)0x02000000)        /*!< PLL clock ready flag */

#define RCC_CR_HSITRIM_Pos                   (3U)                              
#define RCC_CR_HSITRIM_Msk                   (0x1FU << RCC_CR_HSITRIM_Pos)     /*!< 0x000000F8 */

/*******************  Bit definition for RCC_CFG register  *******************/
/** SYSCLKSEL configuration */
#define  RCC_CFG_SYSCLKSEL                      ((uint32_t)0x00000003)        /*!< SYSCLKSEL[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW							RCC_CFG_SYSCLKSEL
#define  RCC_CFG_SYSCLKSEL_0                    ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFG_SYSCLKSEL_1                    ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  RCC_CFG_SYSCLKSEL_HSI                  ((uint32_t)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFG_SYSCLKSEL_HSE                  ((uint32_t)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFG_SYSCLKSEL_PLL                  ((uint32_t)0x00000002)        /*!< PLL selected as system clock */

#define RCC_CFGR_SW_HSI                      0x00000000U                       /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                      0x00000001U                       /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                      0x00000002U                       /*!< PLL selected as system clock */

/** SYSCLKSTST configuration */
#define  RCC_CFG_SYSCLKSTS                      ((uint32_t)0x0000000C)        /*!< SYSCLKSTST[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS							RCC_CFG_SYSCLKSTS
#define  RCC_CFG_SYSCLKSTS_0                    ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  RCC_CFG_SYSCLKSTS_1                    ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  RCC_CFG_SYSCLKSTS_HSI                  ((uint32_t)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFG_SYSCLKSTS_HSE                  ((uint32_t)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFG_SYSCLKSTS_PLL                  ((uint32_t)0x00000008)        /*!< PLL used as system clock */

#define RCC_CFGR_SWS_HSI                     0x00000000U                       /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                     0x00000004U                       /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                     0x00000008U                       /*!< PLL used as system clock */

/** AHBPSC configuration */
#define  RCC_CFG_AHBPSC                         ((uint32_t)0x000000F0)        /*!< AHBPSC[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE 							RCC_CFG_AHBPSC
#define  RCC_CFG_AHBPSC_0                       ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  RCC_CFG_AHBPSC_1                       ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  RCC_CFG_AHBPSC_2                       ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  RCC_CFG_AHBPSC_3                       ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  RCC_CFG_AHBPSC_DIV1                    ((uint32_t)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFG_AHBPSC_DIV2                    ((uint32_t)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFG_AHBPSC_DIV4                    ((uint32_t)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFG_AHBPSC_DIV8                    ((uint32_t)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFG_AHBPSC_DIV16                   ((uint32_t)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFG_AHBPSC_DIV64                   ((uint32_t)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFG_AHBPSC_DIV128                  ((uint32_t)0x000000D0)        /*!< SYSCLK divided by 128 */
#define  RCC_CFG_AHBPSC_DIV256                  ((uint32_t)0x000000E0)        /*!< SYSCLK divided by 256 */
#define  RCC_CFG_AHBPSC_DIV512                  ((uint32_t)0x000000F0)        /*!< SYSCLK divided by 512 */

/** APB1PSC configuration */
#define  RCC_CFG_APB1PSC                        ((uint32_t)0x00000700)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1							RCC_CFG_APB1PSC
#define  RCC_CFG_APB1PSC_0                      ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  RCC_CFG_APB1PSC_1                      ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  RCC_CFG_APB1PSC_2                      ((uint32_t)0x00000400)        /*!< Bit 2 */

#define  RCC_CFG_APB1PSC_DIV1                   ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFG_APB1PSC_DIV2                   ((uint32_t)0x00000400)        /*!< HCLK divided by 2 */
#define  RCC_CFG_APB1PSC_DIV4                   ((uint32_t)0x00000500)        /*!< HCLK divided by 4 */
#define  RCC_CFG_APB1PSC_DIV8                   ((uint32_t)0x00000600)        /*!< HCLK divided by 8 */
#define  RCC_CFG_APB1PSC_DIV16                  ((uint32_t)0x00000700)        /*!< HCLK divided by 16 */

/** APB2PSC configuration */
#define  RCC_CFG_APB2PSC                        ((uint32_t)0x00003800)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2							RCC_CFG_APB2PSC
#define  RCC_CFG_APB2PSC_0                      ((uint32_t)0x00000800)        /*!< Bit 0 */
#define  RCC_CFG_APB2PSC_1                      ((uint32_t)0x00001000)        /*!< Bit 1 */
#define  RCC_CFG_APB2PSC_2                      ((uint32_t)0x00002000)        /*!< Bit 2 */

#define  RCC_CFG_APB2PSC_DIV1                   ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFG_APB2PSC_DIV2                   ((uint32_t)0x00002000)        /*!< HCLK divided by 2 */
#define  RCC_CFG_APB2PSC_DIV4                   ((uint32_t)0x00002800)        /*!< HCLK divided by 4 */
#define  RCC_CFG_APB2PSC_DIV8                   ((uint32_t)0x00003000)        /*!< HCLK divided by 8 */
#define  RCC_CFG_APB2PSC_DIV16                  ((uint32_t)0x00003800)        /*!< HCLK divided by 16 */

#define  RCC_CFG_PLLRC                          ((uint32_t)0x00010000)        /*!< PLL entry clock source */
#define  RCC_CFGR_PLLSRC						RCC_CFG_PLLRC  
#define  RCC_CFG_PLLHSEPSC                      ((uint32_t)0x00020000)        /*!< HSE divider for PLL entry */
#define  RCC_CFGR_PLLXTPRE						RCC_CFG_PLLHSEPSC

///////////////////////////////////////////////////////////////////////////////////
//From STM32 file

#define RCC_CFGR_HPRE_Pos                    (4U)                              
#define RCC_CFGR_HPRE_Msk                    (0xFU << RCC_CFGR_HPRE_Pos)       /*!< 0x000000F0 */
//#define RCC_CFGR_HPRE                        RCC_CFGR_HPRE_Msk                 /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                      (0x1U << RCC_CFGR_HPRE_Pos)       /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                      (0x2U << RCC_CFGR_HPRE_Pos)       /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                      (0x4U << RCC_CFGR_HPRE_Pos)       /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                      (0x8U << RCC_CFGR_HPRE_Pos)       /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                   0x00000000U                       /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV2                   0x00000080U                       /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4                   0x00000090U                       /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8                   0x000000A0U                       /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16                  0x000000B0U                       /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64                  0x000000C0U                       /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128                 0x000000D0U                       /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256                 0x000000E0U                       /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512                 0x000000F0U                       /*!< SYSCLK divided by 512 */


#define RCC_CR_HSION_Pos                     (0U)                              
#define RCC_CR_HSION_Msk                     (0x1U << RCC_CR_HSION_Pos)        /*!< 0x00000001 */
//#define RCC_CR_HSION                         RCC_CR_HSION_Msk                  /*!< Internal High Speed clock enable */
#define RCC_CR_HSIRDY_Pos                    (1U)                              
#define RCC_CR_HSIRDY_Msk                    (0x1U << RCC_CR_HSIRDY_Pos)       /*!< 0x00000002 */
//#define RCC_CR_HSIRDY                        RCC_CR_HSIRDY_Msk                 /*!< Internal High Speed clock ready flag */
#define RCC_CR_HSITRIM_Pos                   (3U)                              
#define RCC_CR_HSITRIM_Msk                   (0x1FU << RCC_CR_HSITRIM_Pos)     /*!< 0x000000F8 */
//#define RCC_CR_HSITRIM                       RCC_CR_HSITRIM_Msk                /*!< Internal High Speed clock trimming */
#define RCC_CR_HSICAL_Pos                    (8U)                              
#define RCC_CR_HSICAL_Msk                    (0xFFU << RCC_CR_HSICAL_Pos)      /*!< 0x0000FF00 */
//#define RCC_CR_HSICAL                        RCC_CR_HSICAL_Msk                 /*!< Internal High Speed clock Calibration */
#define RCC_CR_HSEON_Pos                     (16U)                             
#define RCC_CR_HSEON_Msk                     (0x1U << RCC_CR_HSEON_Pos)        /*!< 0x00010000 */
//#define RCC_CR_HSEON                         RCC_CR_HSEON_Msk                  /*!< External High Speed clock enable */
#define RCC_CR_HSERDY_Pos                    (17U)                             
#define RCC_CR_HSERDY_Msk                    (0x1U << RCC_CR_HSERDY_Pos)       /*!< 0x00020000 */
#define RCC_CR_HSERDY                        RCC_CR_HSERDY_Msk                 /*!< External High Speed clock ready flag */
#define RCC_CR_HSEBYP_Pos                    (18U)                             
#define RCC_CR_HSEBYP_Msk                    (0x1U << RCC_CR_HSEBYP_Pos)       /*!< 0x00040000 */
//#define RCC_CR_HSEBYP                        RCC_CR_HSEBYP_Msk                 /*!< External High Speed clock Bypass */
#define RCC_CR_CSSON_Pos                     (19U)                             
#define RCC_CR_CSSON_Msk                     (0x1U << RCC_CR_CSSON_Pos)        /*!< 0x00080000 */
//#define RCC_CR_CSSON                         RCC_CR_CSSON_Msk                  /*!< Clock Security System enable */
#define RCC_CR_PLLON_Pos                     (24U)                             
#define RCC_CR_PLLON_Msk                     (0x1U << RCC_CR_PLLON_Pos)        /*!< 0x01000000 */
//#define RCC_CR_PLLON                         RCC_CR_PLLON_Msk                  /*!< PLL enable */
#define RCC_CR_PLLRDY_Pos                    (25U)                             
#define RCC_CR_PLLRDY_Msk                    (0x1U << RCC_CR_PLLRDY_Pos)       /*!< 0x02000000 */
#define RCC_CR_PLLRDY                        RCC_CR_PLLRDY_Msk                 /*!< PLL clock ready flag */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                   (8U)                              
#define RCC_CFGR_PPRE1_Msk                   (0x7U << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000700 */
//#define RCC_CFGR_PPRE1                       RCC_CFGR_PPRE1_Msk                /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0                     (0x1U << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000100 */
#define RCC_CFGR_PPRE1_1                     (0x2U << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000200 */
#define RCC_CFGR_PPRE1_2                     (0x4U << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000400 */

#define RCC_CFGR_PPRE1_DIV1                  0x00000000U                       /*!< HCLK not divided */
#define RCC_CFGR_PPRE1_DIV2                  0x00000400U                       /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV4                  0x00000500U                       /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8                  0x00000600U                       /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16                 0x00000700U                       /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                   (11U)                             
#define RCC_CFGR_PPRE2_Msk                   (0x7U << RCC_CFGR_PPRE2_Pos)      /*!< 0x00003800 */
//#define RCC_CFGR_PPRE2                       RCC_CFGR_PPRE2_Msk                /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0                     (0x1U << RCC_CFGR_PPRE2_Pos)      /*!< 0x00000800 */
#define RCC_CFGR_PPRE2_1                     (0x2U << RCC_CFGR_PPRE2_Pos)      /*!< 0x00001000 */
#define RCC_CFGR_PPRE2_2                     (0x4U << RCC_CFGR_PPRE2_Pos)      /*!< 0x00002000 */

#define RCC_CFGR_PPRE2_DIV1                  0x00000000U                       /*!< HCLK not divided */
#define RCC_CFGR_PPRE2_DIV2                  0x00002000U                       /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE2_DIV4                  0x00002800U                       /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE2_DIV8                  0x00003000U                       /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE2_DIV16                 0x00003800U                       /*!< HCLK divided by 16 */

/*!< ADCPPRE configuration */
#define RCC_CFGR_ADCPRE_Pos                  (14U)                             
#define RCC_CFGR_ADCPRE_Msk                  (0x3U << RCC_CFGR_ADCPRE_Pos)     /*!< 0x0000C000 */
#define RCC_CFGR_ADCPRE                      RCC_CFGR_ADCPRE_Msk               /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define RCC_CFGR_ADCPRE_0                    (0x1U << RCC_CFGR_ADCPRE_Pos)     /*!< 0x00004000 */
#define RCC_CFGR_ADCPRE_1                    (0x2U << RCC_CFGR_ADCPRE_Pos)     /*!< 0x00008000 */

#define RCC_CFGR_ADCPRE_DIV2                 0x00000000U                       /*!< PCLK2 divided by 2 */
#define RCC_CFGR_ADCPRE_DIV4                 0x00004000U                       /*!< PCLK2 divided by 4 */
#define RCC_CFGR_ADCPRE_DIV6                 0x00008000U                       /*!< PCLK2 divided by 6 */
#define RCC_CFGR_ADCPRE_DIV8                 0x0000C000U                       /*!< PCLK2 divided by 8 */

#define RCC_CFGR_PLLSRC_Pos                  (16U)                             
#define RCC_CFGR_PLLSRC_Msk                  (0x1U << RCC_CFGR_PLLSRC_Pos)     /*!< 0x00010000 */
//#define RCC_CFGR_PLLSRC                      RCC_CFGR_PLLSRC_Msk               /*!< PLL entry clock source */

#define RCC_CFGR_PLLXTPRE_Pos                (17U)                             
#define RCC_CFGR_PLLXTPRE_Msk                (0x1U << RCC_CFGR_PLLXTPRE_Pos)   /*!< 0x00020000 */
//#define RCC_CFGR_PLLXTPRE                    RCC_CFGR_PLLXTPRE_Msk             /*!< HSE divider for PLL entry */

/*!< PLLMUL configuration */
#define RCC_CFGR_PLLMULL_Pos                 (18U)                             
#define RCC_CFGR_PLLMULL_Msk                 (0xFU << RCC_CFGR_PLLMULL_Pos)    /*!< 0x003C0000 */
//#define RCC_CFGR_PLLMULL                     RCC_CFGR_PLLMULL_Msk              /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define RCC_CFGR_PLLMULL_0                   (0x1U << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00040000 */
#define RCC_CFGR_PLLMULL_1                   (0x2U << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00080000 */
#define RCC_CFGR_PLLMULL_2                   (0x4U << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00100000 */
#define RCC_CFGR_PLLMULL_3                   (0x8U << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00200000 */

#define RCC_CFGR_PLLXTPRE_HSE                0x00000000U                      /*!< HSE clock not divided for PLL entry */
#define RCC_CFGR_PLLXTPRE_HSE_DIV2           0x00020000U                      /*!< HSE clock divided by 2 for PLL entry */

#define RCC_CFGR_PLLMULL2                    0x00000000U                       /*!< PLL input clock*2 */
#define RCC_CFGR_PLLMULL3_Pos                (18U)                             
#define RCC_CFGR_PLLMULL3_Msk                (0x1U << RCC_CFGR_PLLMULL3_Pos)   /*!< 0x00040000 */
#define RCC_CFGR_PLLMULL3                    RCC_CFGR_PLLMULL3_Msk             /*!< PLL input clock*3 */
#define RCC_CFGR_PLLMULL4_Pos                (19U)                             
#define RCC_CFGR_PLLMULL4_Msk                (0x1U << RCC_CFGR_PLLMULL4_Pos)   /*!< 0x00080000 */
#define RCC_CFGR_PLLMULL4                    RCC_CFGR_PLLMULL4_Msk             /*!< PLL input clock*4 */
#define RCC_CFGR_PLLMULL5_Pos                (18U)                             
#define RCC_CFGR_PLLMULL5_Msk                (0x3U << RCC_CFGR_PLLMULL5_Pos)   /*!< 0x000C0000 */
#define RCC_CFGR_PLLMULL5                    RCC_CFGR_PLLMULL5_Msk             /*!< PLL input clock*5 */
#define RCC_CFGR_PLLMULL6_Pos                (20U)                             
#define RCC_CFGR_PLLMULL6_Msk                (0x1U << RCC_CFGR_PLLMULL6_Pos)   /*!< 0x00100000 */
#define RCC_CFGR_PLLMULL6                    RCC_CFGR_PLLMULL6_Msk             /*!< PLL input clock*6 */
#define RCC_CFGR_PLLMULL7_Pos                (18U)                             
#define RCC_CFGR_PLLMULL7_Msk                (0x5U << RCC_CFGR_PLLMULL7_Pos)   /*!< 0x00140000 */
#define RCC_CFGR_PLLMULL7                    RCC_CFGR_PLLMULL7_Msk             /*!< PLL input clock*7 */
#define RCC_CFGR_PLLMULL8_Pos                (19U)                             
#define RCC_CFGR_PLLMULL8_Msk                (0x3U << RCC_CFGR_PLLMULL8_Pos)   /*!< 0x00180000 */
#define RCC_CFGR_PLLMULL8                    RCC_CFGR_PLLMULL8_Msk             /*!< PLL input clock*8 */
#define RCC_CFGR_PLLMULL9_Pos                (18U)                             
#define RCC_CFGR_PLLMULL9_Msk                (0x7U << RCC_CFGR_PLLMULL9_Pos)   /*!< 0x001C0000 */
#define RCC_CFGR_PLLMULL9                    RCC_CFGR_PLLMULL9_Msk             /*!< PLL input clock*9 */
#define RCC_CFGR_PLLMULL10_Pos               (21U)                             
#define RCC_CFGR_PLLMULL10_Msk               (0x1U << RCC_CFGR_PLLMULL10_Pos)  /*!< 0x00200000 */
#define RCC_CFGR_PLLMULL10                   RCC_CFGR_PLLMULL10_Msk            /*!< PLL input clock10 */
#define RCC_CFGR_PLLMULL11_Pos               (18U)                             
#define RCC_CFGR_PLLMULL11_Msk               (0x9U << RCC_CFGR_PLLMULL11_Pos)  /*!< 0x00240000 */
#define RCC_CFGR_PLLMULL11                   RCC_CFGR_PLLMULL11_Msk            /*!< PLL input clock*11 */
#define RCC_CFGR_PLLMULL12_Pos               (19U)                             
#define RCC_CFGR_PLLMULL12_Msk               (0x5U << RCC_CFGR_PLLMULL12_Pos)  /*!< 0x00280000 */
#define RCC_CFGR_PLLMULL12                   RCC_CFGR_PLLMULL12_Msk            /*!< PLL input clock*12 */
#define RCC_CFGR_PLLMULL13_Pos               (18U)                             
#define RCC_CFGR_PLLMULL13_Msk               (0xBU << RCC_CFGR_PLLMULL13_Pos)  /*!< 0x002C0000 */
#define RCC_CFGR_PLLMULL13                   RCC_CFGR_PLLMULL13_Msk            /*!< PLL input clock*13 */
#define RCC_CFGR_PLLMULL14_Pos               (20U)                             
#define RCC_CFGR_PLLMULL14_Msk               (0x3U << RCC_CFGR_PLLMULL14_Pos)  /*!< 0x00300000 */
#define RCC_CFGR_PLLMULL14                   RCC_CFGR_PLLMULL14_Msk            /*!< PLL input clock*14 */
#define RCC_CFGR_PLLMULL15_Pos               (18U)                             
#define RCC_CFGR_PLLMULL15_Msk               (0xDU << RCC_CFGR_PLLMULL15_Pos)  /*!< 0x00340000 */
#define RCC_CFGR_PLLMULL15                   RCC_CFGR_PLLMULL15_Msk            /*!< PLL input clock*15 */
#define RCC_CFGR_PLLMULL16_Pos               (19U)                             
#define RCC_CFGR_PLLMULL16_Msk               (0x7U << RCC_CFGR_PLLMULL16_Pos)  /*!< 0x00380000 */
#define RCC_CFGR_PLLMULL16                   RCC_CFGR_PLLMULL16_Msk            /*!< PLL input clock*16 */
#define RCC_CFGR_USBPRE_Pos                  (22U)                             
#define RCC_CFGR_USBPRE_Msk                  (0x1U << RCC_CFGR_USBPRE_Pos)     /*!< 0x00400000 */
#define RCC_CFGR_USBPRE                      RCC_CFGR_USBPRE_Msk               /*!< USB Device prescaler */

/*!< MCO configuration */
#define RCC_CFGR_MCO_Pos                     (24U)                             
#define RCC_CFGR_MCO_Msk                     (0x7U << RCC_CFGR_MCO_Pos)        /*!< 0x07000000 */
#define RCC_CFGR_MCO                         RCC_CFGR_MCO_Msk                  /*!< MCO[2:0] bits (Microcontroller Clock Output) */
#define RCC_CFGR_MCO_0                       (0x1U << RCC_CFGR_MCO_Pos)        /*!< 0x01000000 */
#define RCC_CFGR_MCO_1                       (0x2U << RCC_CFGR_MCO_Pos)        /*!< 0x02000000 */
#define RCC_CFGR_MCO_2                       (0x4U << RCC_CFGR_MCO_Pos)        /*!< 0x04000000 */

#define RCC_CFGR_MCO_NOCLOCK                 0x00000000U                        /*!< No clock */
#define RCC_CFGR_MCO_SYSCLK                  0x04000000U                        /*!< System clock selected as MCO source */
#define RCC_CFGR_MCO_HSI                     0x05000000U                        /*!< HSI clock selected as MCO source */
#define RCC_CFGR_MCO_HSE                     0x06000000U                        /*!< HSE clock selected as MCO source  */
#define RCC_CFGR_MCO_PLLCLK_DIV2             0x07000000U                        /*!< PLL clock divided by 2 selected as MCO source */

/******************  Bit definition for RCC_APB2ENR register  *****************/
#define RCC_APB2ENR_AFIOEN_Pos               (0U)                              
#define RCC_APB2ENR_AFIOEN_Msk               (0x1U << RCC_APB2ENR_AFIOEN_Pos)  /*!< 0x00000001 */
#define RCC_APB2ENR_AFIOEN                   RCC_APB2EN_AFIOEN            /*!< Alternate Function I/O clock enable */
#define RCC_APB2ENR_IOPAEN_Pos               (2U)                              
#define RCC_APB2ENR_IOPAEN_Msk               (0x1U << RCC_APB2ENR_IOPAEN_Pos)  /*!< 0x00000004 */
#define RCC_APB2ENR_IOPAEN                   RCC_APB2EN_GPIOAEN            /*!< I/O port A clock enable */
#define RCC_APB2ENR_IOPBEN_Pos               (3U)                              
#define RCC_APB2ENR_IOPBEN_Msk               (0x1U << RCC_APB2ENR_IOPBEN_Pos)  /*!< 0x00000008 */
#define RCC_APB2ENR_IOPBEN                   RCC_APB2EN_GPIOBEN            /*!< I/O port B clock enable */
#define RCC_APB2ENR_IOPCEN_Pos               (4U)                              
#define RCC_APB2ENR_IOPCEN_Msk               (0x1U << RCC_APB2ENR_IOPCEN_Pos)  /*!< 0x00000010 */
#define RCC_APB2ENR_IOPCEN                   RCC_APB2EN_GPIOCEN            /*!< I/O port C clock enable */
#define RCC_APB2ENR_IOPDEN_Pos               (5U)                              
#define RCC_APB2ENR_IOPDEN_Msk               (0x1U << RCC_APB2ENR_IOPDEN_Pos)  /*!< 0x00000020 */
#define RCC_APB2ENR_IOPDEN                   RCC_APB2EN_GPIODEN            /*!< I/O port D clock enable */
#define RCC_APB2ENR_ADC1EN_Pos               (9U)                              
#define RCC_APB2ENR_ADC1EN_Msk               (0x1U << RCC_APB2ENR_ADC1EN_Pos)  /*!< 0x00000200 */
#define RCC_APB2ENR_ADC1EN                   RCC_APB2EN_ADC1EN            /*!< ADC 1 interface clock enable */

#define RCC_APB2ENR_ADC2EN_Pos               (10U)                             
#define RCC_APB2ENR_ADC2EN_Msk               (0x1U << RCC_APB2ENR_ADC2EN_Pos)  /*!< 0x00000400 */
#define RCC_APB2ENR_ADC2EN                   RCC_APB2EN_ADC2EN            /*!< ADC 2 interface clock enable */

#define RCC_APB2ENR_TIM1EN_Pos               (11U)                             
#define RCC_APB2ENR_TIM1EN_Msk               (0x1U << RCC_APB2ENR_TIM1EN_Pos)  /*!< 0x00000800 */
#define RCC_APB2ENR_TIM1EN                   RCC_APB2EN_TMR1EN            /*!< TIM1 Timer clock enable */
#define RCC_APB2ENR_SPI1EN_Pos               (12U)                             
#define RCC_APB2ENR_SPI1EN_Msk               (0x1U << RCC_APB2ENR_SPI1EN_Pos)  /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                   RCC_APB2EN_SPI1EN            /*!< SPI 1 clock enable */
#define RCC_APB2ENR_USART1EN_Pos             (14U)                             
#define RCC_APB2ENR_USART1EN_Msk             (0x1U << RCC_APB2ENR_USART1EN_Pos) /*!< 0x00004000 */
#define RCC_APB2ENR_USART1EN                 RCC_APB2EN_USART1EN          /*!< USART1 clock enable */


#define RCC_APB2ENR_IOPEEN_Pos               (6U)                              
#define RCC_APB2ENR_IOPEEN_Msk               (0x1U << RCC_APB2ENR_IOPEEN_Pos)  /*!< 0x00000040 */
#define RCC_APB2ENR_IOPEEN                   RCC_APB2EN_GPIOEEN            /*!< I/O port E clock enable */

#define RCC_APB2ENR_IOPFEN_Pos               (7U)                              
#define RCC_APB2ENR_IOPFEN_Msk               (0x1U << RCC_APB2ENR_IOPFEN_Pos)  /*!< 0x00000080 */
#define RCC_APB2ENR_IOPFEN                   RCC_APB2EN_GPIOFEN            /*!< I/O port F clock enable */
#define RCC_APB2ENR_IOPGEN_Pos               (8U)                              
#define RCC_APB2ENR_IOPGEN_Msk               (0x1U << RCC_APB2ENR_IOPGEN_Pos)  /*!< 0x00000100 */
#define RCC_APB2ENR_IOPGEN                   RCC_APB2EN_GPIOGEN            /*!< I/O port G clock enable */
#define RCC_APB2ENR_TIM8EN_Pos               (13U)                             
#define RCC_APB2ENR_TIM8EN_Msk               (0x1U << RCC_APB2ENR_TIM8EN_Pos)  /*!< 0x00002000 */
#define RCC_APB2ENR_TIM8EN                   RCC_APB2EN_TMR8EN            /*!< TIM8 Timer clock enable */
#define RCC_APB2ENR_ADC3EN_Pos               (15U)                             
#define RCC_APB2ENR_ADC3EN_Msk               (0x1U << RCC_APB2ENR_ADC3EN_Pos)  /*!< 0x00008000 */
#define RCC_APB2ENR_ADC3EN                   RCC_APB2EN_ADC3EN            /*!< DMA1 clock enable */

#define RCC_BDCR_LSEON_Pos                   (0U)                              
#define RCC_BDCR_LSEON_Msk                   (0x1U << RCC_BDCR_LSEON_Pos)      /*!< 0x00000001 */
//#define RCC_BDCR_LSEON                       RCC_BDCR_LSEON_Msk                /*!< External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY_Pos                  (1U)                              
#define RCC_BDCR_LSERDY_Msk                  (0x1U << RCC_BDCR_LSERDY_Pos)     /*!< 0x00000002 */
#define RCC_BDCR_LSERDY                      RCC_BDCR_LSERDY_Msk               /*!< External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP_Pos                  (2U)                              
#define RCC_BDCR_LSEBYP_Msk                  (0x1U << RCC_BDCR_LSEBYP_Pos)     /*!< 0x00000004 */
//#define RCC_BDCR_LSEBYP                      RCC_BDCR_LSEBYP_Msk               /*!< External Low Speed oscillator Bypass */

#define RCC_BDCR_RTCSEL_Pos                  (8U)                              
#define RCC_BDCR_RTCSEL_Msk                  (0x3U << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL                      RCC_BDCR_RTCSEL_Msk               /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define RCC_BDCR_RTCSEL_0                    (0x1U << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1                    (0x2U << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000200 */

#define RCC_APB1ENR_BKPEN_Pos                (27U)                             
#define RCC_APB1ENR_BKPEN_Msk                (0x1U << RCC_APB1ENR_BKPEN_Pos)   /*!< 0x08000000 */
#define RCC_APB1ENR_BKPEN                    RCC_APB1ENR_BKPEN_Msk             /*!< Backup interface clock enable */
#define RCC_APB1ENR_PWREN_Pos                (28U)                             
#define RCC_APB1ENR_PWREN_Msk                (0x1U << RCC_APB1ENR_PWREN_Pos)   /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                    RCC_APB1ENR_PWREN_Msk             /*!< Power interface clock enable */

#define RCC_CSR_LSION_Pos                    (0U)                              
#define RCC_CSR_LSION_Msk                    (0x1U << RCC_CSR_LSION_Pos)       /*!< 0x00000001 */
//#define RCC_CSR_LSION                        RCC_CSR_LSION_Msk                 /*!< Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY_Pos                   (1U)                              
#define RCC_CSR_LSIRDY_Msk                   (0x1U << RCC_CSR_LSIRDY_Pos)      /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                       RCC_CSR_LSIRDY_Msk                /*!< Internal Low Speed oscillator Ready */
#define RCC_CSR_RMVF_Pos                     (24U)                             
#define RCC_CSR_RMVF_Msk                     (0x1U << RCC_CSR_RMVF_Pos)        /*!< 0x01000000 */
#define RCC_CSR_RMVF                         RCC_CSR_RMVF_Msk                  /*!< Remove reset flag */
#define RCC_CSR_PINRSTF_Pos                  (26U)                             
#define RCC_CSR_PINRSTF_Msk                  (0x1U << RCC_CSR_PINRSTF_Pos)     /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                      RCC_CSR_PINRSTF_Msk               /*!< PIN reset flag */
#define RCC_CSR_PORRSTF_Pos                  (27U)                             
#define RCC_CSR_PORRSTF_Msk                  (0x1U << RCC_CSR_PORRSTF_Pos)     /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                      RCC_CSR_PORRSTF_Msk               /*!< POR/PDR reset flag */
#define RCC_CSR_SFTRSTF_Pos                  (28U)                             
#define RCC_CSR_SFTRSTF_Msk                  (0x1U << RCC_CSR_SFTRSTF_Pos)     /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                      RCC_CSR_SFTRSTF_Msk               /*!< Software Reset flag */
#define RCC_CSR_IWDGRSTF_Pos                 (29U)                             
#define RCC_CSR_IWDGRSTF_Msk                 (0x1U << RCC_CSR_IWDGRSTF_Pos)    /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                     RCC_CSR_IWDGRSTF_Msk              /*!< Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF_Pos                 (30U)                             
#define RCC_CSR_WWDGRSTF_Msk                 (0x1U << RCC_CSR_WWDGRSTF_Pos)    /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF                     RCC_CSR_WWDGRSTF_Msk              /*!< Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF_Pos                 (31U)                             
#define RCC_CSR_LPWRRSTF_Msk                 (0x1U << RCC_CSR_LPWRRSTF_Pos)    /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF                     RCC_CSR_LPWRRSTF_Msk              /*!< Low-Power reset flag */

#define RCC_CR_CSSON_Pos                     (19U)                             
#define RCC_CR_CSSON_Msk                     (0x1U << RCC_CR_CSSON_Pos)        /*!< 0x00080000 */
#define RCC_CR_CSSON                         RCC_CR_CSSON_Msk                  /*!< Clock Security System enable */


/*****************  Bit definition for RCC_APB1RSTR register  *****************/
#define RCC_APB1RSTR_TIM2RST_Pos             (0U)                              
#define RCC_APB1RSTR_TIM2RST_Msk             (0x1U << RCC_APB1RSTR_TIM2RST_Pos) /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST                 RCC_APB1RSTR_TIM2RST_Msk          /*!< Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST_Pos             (1U)                              
#define RCC_APB1RSTR_TIM3RST_Msk             (0x1U << RCC_APB1RSTR_TIM3RST_Pos) /*!< 0x00000002 */
#define RCC_APB1RSTR_TIM3RST                 RCC_APB1RSTR_TIM3RST_Msk          /*!< Timer 3 reset */
#define RCC_APB1RSTR_WWDGRST_Pos             (11U)                             
#define RCC_APB1RSTR_WWDGRST_Msk             (0x1U << RCC_APB1RSTR_WWDGRST_Pos) /*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST                 RCC_APB1RSTR_WWDGRST_Msk          /*!< Window Watchdog reset */
#define RCC_APB1RSTR_USART2RST_Pos           (17U)                             
#define RCC_APB1RSTR_USART2RST_Msk           (0x1U << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_USART2RST               RCC_APB1RSTR_USART2RST_Msk        /*!< USART 2 reset */
#define RCC_APB1RSTR_I2C1RST_Pos             (21U)                             
#define RCC_APB1RSTR_I2C1RST_Msk             (0x1U << RCC_APB1RSTR_I2C1RST_Pos) /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST                 RCC_APB1RSTR_I2C1RST_Msk          /*!< I2C 1 reset */

#define RCC_APB1RSTR_CAN1RST_Pos             (25U)                             
#define RCC_APB1RSTR_CAN1RST_Msk             (0x1U << RCC_APB1RSTR_CAN1RST_Pos) /*!< 0x02000000 */
#define RCC_APB1RSTR_CAN1RST                 RCC_APB1RSTR_CAN1RST_Msk          /*!< CAN1 reset */

#define RCC_APB1RSTR_BKPRST_Pos              (27U)                             
#define RCC_APB1RSTR_BKPRST_Msk              (0x1U << RCC_APB1RSTR_BKPRST_Pos) /*!< 0x08000000 */
#define RCC_APB1RSTR_BKPRST                  RCC_APB1RSTR_BKPRST_Msk           /*!< Backup interface reset */
#define RCC_APB1RSTR_PWRRST_Pos              (28U)                             
#define RCC_APB1RSTR_PWRRST_Msk              (0x1U << RCC_APB1RSTR_PWRRST_Pos) /*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST                  RCC_APB1RSTR_PWRRST_Msk           /*!< Power interface reset */

#define RCC_APB1RSTR_TIM4RST_Pos             (2U)                              
#define RCC_APB1RSTR_TIM4RST_Msk             (0x1U << RCC_APB1RSTR_TIM4RST_Pos) /*!< 0x00000004 */
#define RCC_APB1RSTR_TIM4RST                 RCC_APB1RSTR_TIM4RST_Msk          /*!< Timer 4 reset */
#define RCC_APB1RSTR_SPI2RST_Pos             (14U)                             
#define RCC_APB1RSTR_SPI2RST_Msk             (0x1U << RCC_APB1RSTR_SPI2RST_Pos) /*!< 0x00004000 */
#define RCC_APB1RSTR_SPI2RST                 RCC_APB1RSTR_SPI2RST_Msk          /*!< SPI 2 reset */
#define RCC_APB1RSTR_USART3RST_Pos           (18U)                             
#define RCC_APB1RSTR_USART3RST_Msk           (0x1U << RCC_APB1RSTR_USART3RST_Pos) /*!< 0x00040000 */
#define RCC_APB1RSTR_USART3RST               RCC_APB1RSTR_USART3RST_Msk        /*!< USART 3 reset */
#define RCC_APB1RSTR_I2C2RST_Pos             (22U)                             
#define RCC_APB1RSTR_I2C2RST_Msk             (0x1U << RCC_APB1RSTR_I2C2RST_Pos) /*!< 0x00400000 */
#define RCC_APB1RSTR_I2C2RST                 RCC_APB1RSTR_I2C2RST_Msk          /*!< I2C 2 reset */

#define RCC_APB1RSTR_USBRST_Pos              (23U)                             
#define RCC_APB1RSTR_USBRST_Msk              (0x1U << RCC_APB1RSTR_USBRST_Pos) /*!< 0x00800000 */
#define RCC_APB1RSTR_USBRST                  RCC_APB1RSTR_USBRST_Msk           /*!< USB Device reset */

#define RCC_APB1RSTR_TIM5RST_Pos             (3U)                              
#define RCC_APB1RSTR_TIM5RST_Msk             (0x1U << RCC_APB1RSTR_TIM5RST_Pos) /*!< 0x00000008 */
#define RCC_APB1RSTR_TIM5RST                 RCC_APB1RSTR_TIM5RST_Msk          /*!< Timer 5 reset */
#define RCC_APB1RSTR_TIM6RST_Pos             (4U)                              
#define RCC_APB1RSTR_TIM6RST_Msk             (0x1U << RCC_APB1RSTR_TIM6RST_Pos) /*!< 0x00000010 */
#define RCC_APB1RSTR_TIM6RST                 RCC_APB1RSTR_TIM6RST_Msk          /*!< Timer 6 reset */
#define RCC_APB1RSTR_TIM7RST_Pos             (5U)                              
#define RCC_APB1RSTR_TIM7RST_Msk             (0x1U << RCC_APB1RSTR_TIM7RST_Pos) /*!< 0x00000020 */
#define RCC_APB1RSTR_TIM7RST                 RCC_APB1RSTR_TIM7RST_Msk          /*!< Timer 7 reset */
#define RCC_APB1RSTR_SPI3RST_Pos             (15U)                             
#define RCC_APB1RSTR_SPI3RST_Msk             (0x1U << RCC_APB1RSTR_SPI3RST_Pos) /*!< 0x00008000 */
#define RCC_APB1RSTR_SPI3RST                 RCC_APB1RSTR_SPI3RST_Msk          /*!< SPI 3 reset */
#define RCC_APB1RSTR_UART4RST_Pos            (19U)                             
#define RCC_APB1RSTR_UART4RST_Msk            (0x1U << RCC_APB1RSTR_UART4RST_Pos) /*!< 0x00080000 */
#define RCC_APB1RSTR_UART4RST                RCC_APB1RSTR_UART4RST_Msk         /*!< UART 4 reset */
#define RCC_APB1RSTR_UART5RST_Pos            (20U)                             
#define RCC_APB1RSTR_UART5RST_Msk            (0x1U << RCC_APB1RSTR_UART5RST_Pos) /*!< 0x00100000 */
#define RCC_APB1RSTR_UART5RST                RCC_APB1RSTR_UART5RST_Msk         /*!< UART 5 reset */


////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** ADCPSC configuration */
#define  RCC_CFG_ADCPSC                         ((uint32_t)0x1000C000)        /*!< ADCPSC[1:0] bits (ADC prescaler) */
#define  RCC_CFG_ADCPSC_0                       ((uint32_t)0x00004000)        /*!< Bit 0 */
#define  RCC_CFG_ADCPSC_1                       ((uint32_t)0x00008000)        /*!< Bit 1 */
#define  RCC_CFG_ADCPSC_2                       ((uint32_t)0x10000000)        /*!< Bit 2 */
#define  RCC_CFG_ADCPSC_DIV2                    ((uint32_t)0x00000000)        /*!< PCLK2 divided by 2 */
#define  RCC_CFG_ADCPSC_DIV4                    ((uint32_t)0x00004000)        /*!< PCLK2 divided by 4 */
#define  RCC_CFG_ADCPSC_DIV6                    ((uint32_t)0x00008000)        /*!< PCLK2 divided by 6 */
#define  RCC_CFG_ADCPSC_DIV8                    ((uint32_t)0x0000C000)        /*!< PCLK2 divided by 8 */
#define  RCC_CFG_ADCPSC_DIV12                   ((uint32_t)0x10004000)        /*!< PCLK2 divided by 12 */
#define  RCC_CFG_ADCPSC_DIV16                   ((uint32_t)0x1000C000)        /*!< PCLK2 divided by 16 */

#define  RCC_CFG_PLLCFG_MASK                    ((uint32_t)0x1FC0FFFF)        /*!< Mask for PLLRANGE, PLLHSEPSC, PLLMULT, PLLRC */


/** PLLMULT configuration */
#define  RCC_CFG_PLLMULT                        ((uint32_t)0x603C0000)        /*!< PLLMUL[5:0] bits (PLL multiplication factor) */
#define  RCC_CFGR_PLLMULL						RCC_CFG_PLLMULT
#define  RCC_CFG_PLLMULT_0                      ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  RCC_CFG_PLLMULT_1                      ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  RCC_CFG_PLLMULT_2                      ((uint32_t)0x00100000)        /*!< Bit 2 */
#define  RCC_CFG_PLLMULT_3                      ((uint32_t)0x00200000)        /*!< Bit 3 */
#define  RCC_CFG_PLLMULT_4                      ((uint32_t)0x20000000)        /*!< Bit 4 */
#define  RCC_CFG_PLLMULT_5                      ((uint32_t)0x40000000)        /*!< Bit 5 */
#define  RCC_CFG_PLLMULT_LB_MASK                ((uint32_t)0x003C0000)
#define  RCC_CFG_PLLMULT_HB_MASK                ((uint32_t)0x60000000)
#define  RCC_CFG_PLLMULT_LB_POS                 ((uint32_t)18)
#define  RCC_CFG_PLLMULT_HB_POS                 ((uint32_t)29)
#define  RCC_CFG_PLLMULT_HB_OFFSET              ((uint32_t)4)


#define  RCC_CFG_PLLRANGE                       ((uint32_t)0x80000000)        /*!< PLL Frequency range.  */
#define  RCC_CFG_PLLRANGE_LE72MHZ               ((uint32_t)0x00000000)        /*!< When PLL frequency is less than or equal to 72MHz */
#define  RCC_CFG_PLLRANGE_GT72MHZ               ((uint32_t)0x80000000)        /*!< When PLL frequency is greater than 72MHz */

#define  RCC_CFG_PLLRC_HSI_DIV2                 ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
#define  RCC_CFG_PLLRC_HSE                      ((uint32_t)0x00010000)        /*!< HSE clock selected as PLL entry clock source */

#define  RCC_CFG_PLLHSEPSC_HSE                  ((uint32_t)0x00000000)        /*!< HSE clock not divided for PLL entry */
#define  RCC_CFG_PLLHSEPSC_HSE_DIV2             ((uint32_t)0x00020000)        /*!< HSE clock divided by 2 for PLL entry */

#define  RCC_CFG_PLLMULT2                       ((uint32_t)0x00000000)        /*!< PLL input clock*2 */
#define  RCC_CFG_PLLMULT3                       ((uint32_t)0x00040000)        /*!< PLL input clock*3 */
#define  RCC_CFG_PLLMULT4                       ((uint32_t)0x00080000)        /*!< PLL input clock*4 */
#define  RCC_CFG_PLLMULT5                       ((uint32_t)0x000C0000)        /*!< PLL input clock*5 */
#define  RCC_CFG_PLLMULT6                       ((uint32_t)0x00100000)        /*!< PLL input clock*6 */
#define  RCC_CFG_PLLMULT7                       ((uint32_t)0x00140000)        /*!< PLL input clock*7 */
#define  RCC_CFG_PLLMULT8                       ((uint32_t)0x00180000)        /*!< PLL input clock*8 */
#define  RCC_CFG_PLLMULT9                       ((uint32_t)0x001C0000)        /*!< PLL input clock*9 */
#define  RCC_CFG_PLLMULT10                      ((uint32_t)0x00200000)        /*!< PLL input clock*10 */
#define  RCC_CFG_PLLMULT11                      ((uint32_t)0x00240000)        /*!< PLL input clock*11 */
#define  RCC_CFG_PLLMULT12                      ((uint32_t)0x00280000)        /*!< PLL input clock*12 */
#define  RCC_CFG_PLLMULT13                      ((uint32_t)0x002C0000)        /*!< PLL input clock*13 */
#define  RCC_CFG_PLLMULT14                      ((uint32_t)0x00300000)        /*!< PLL input clock*14 */
#define  RCC_CFG_PLLMULT15                      ((uint32_t)0x00340000)        /*!< PLL input clock*15 */
#define  RCC_CFG_PLLMULT16                      ((uint32_t)0x00380000)        /*!< PLL input clock*16 */

#define  RCC_CFG_PLLMULT17                      ((uint32_t)0x20000000)        /*!< PLL input clock*17 */
#define  RCC_CFG_PLLMULT18                      ((uint32_t)0x20040000)        /*!< PLL input clock*18 */
#define  RCC_CFG_PLLMULT19                      ((uint32_t)0x20080000)        /*!< PLL input clock*19 */
#define  RCC_CFG_PLLMULT20                      ((uint32_t)0x200C0000)        /*!< PLL input clock*20 */
#define  RCC_CFG_PLLMULT21                      ((uint32_t)0x20100000)        /*!< PLL input clock*21 */
#define  RCC_CFG_PLLMULT22                      ((uint32_t)0x20140000)        /*!< PLL input clock*22 */
#define  RCC_CFG_PLLMULT23                      ((uint32_t)0x20180000)        /*!< PLL input clock*23 */
#define  RCC_CFG_PLLMULT24                      ((uint32_t)0x201C0000)        /*!< PLL input clock*24 */
#define  RCC_CFG_PLLMULT25                      ((uint32_t)0x20200000)        /*!< PLL input clock*25 */
#define  RCC_CFG_PLLMULT26                      ((uint32_t)0x20240000)        /*!< PLL input clock*26 */
#define  RCC_CFG_PLLMULT27                      ((uint32_t)0x20280000)        /*!< PLL input clock*27 */
#define  RCC_CFG_PLLMULT28                      ((uint32_t)0x202C0000)        /*!< PLL input clock*28 */
#define  RCC_CFG_PLLMULT29                      ((uint32_t)0x20300000)        /*!< PLL input clock*29 */
#define  RCC_CFG_PLLMULT30                      ((uint32_t)0x20340000)        /*!< PLL input clock*30 */
#define  RCC_CFG_PLLMULT31                      ((uint32_t)0x20380000)        /*!< PLL input clock*31 */
#define  RCC_CFG_PLLMULT32                      ((uint32_t)0x203C0000)        /*!< PLL input clock*32 */
#define  RCC_CFG_PLLMULT33                      ((uint32_t)0x40000000)        /*!< PLL input clock*33 */
#define  RCC_CFG_PLLMULT34                      ((uint32_t)0x40040000)        /*!< PLL input clock*34 */
#define  RCC_CFG_PLLMULT35                      ((uint32_t)0x40080000)        /*!< PLL input clock*35 */
#define  RCC_CFG_PLLMULT36                      ((uint32_t)0x400C0000)        /*!< PLL input clock*36 */
#define  RCC_CFG_PLLMULT37                      ((uint32_t)0x40100000)        /*!< PLL input clock*37 */
#define  RCC_CFG_PLLMULT38                      ((uint32_t)0x40140000)        /*!< PLL input clock*38 */
#define  RCC_CFG_PLLMULT39                      ((uint32_t)0x40180000)        /*!< PLL input clock*39 */
#define  RCC_CFG_PLLMULT40                      ((uint32_t)0x401C0000)        /*!< PLL input clock*40 */
#define  RCC_CFG_PLLMULT41                      ((uint32_t)0x40200000)        /*!< PLL input clock*41 */
#define  RCC_CFG_PLLMULT42                      ((uint32_t)0x40240000)        /*!< PLL input clock*42 */
#define  RCC_CFG_PLLMULT43                      ((uint32_t)0x40280000)        /*!< PLL input clock*43 */
#define  RCC_CFG_PLLMULT44                      ((uint32_t)0x402C0000)        /*!< PLL input clock*44 */
#define  RCC_CFG_PLLMULT45                      ((uint32_t)0x40300000)        /*!< PLL input clock*45 */
#define  RCC_CFG_PLLMULT46                      ((uint32_t)0x40340000)        /*!< PLL input clock*46 */
#define  RCC_CFG_PLLMULT47                      ((uint32_t)0x40380000)        /*!< PLL input clock*47 */
#define  RCC_CFG_PLLMULT48                      ((uint32_t)0x403C0000)        /*!< PLL input clock*48 */
#define  RCC_CFG_PLLMULT49                      ((uint32_t)0x60000000)        /*!< PLL input clock*49 */
#define  RCC_CFG_PLLMULT50                      ((uint32_t)0x60040000)        /*!< PLL input clock*50 */
#define  RCC_CFG_PLLMULT51                      ((uint32_t)0x60080000)        /*!< PLL input clock*51 */
#define  RCC_CFG_PLLMULT52                      ((uint32_t)0x600C0000)        /*!< PLL input clock*52 */
#define  RCC_CFG_PLLMULT53                      ((uint32_t)0x60100000)        /*!< PLL input clock*53 */
#define  RCC_CFG_PLLMULT54                      ((uint32_t)0x60140000)        /*!< PLL input clock*54 */
#define  RCC_CFG_PLLMULT55                      ((uint32_t)0x60180000)        /*!< PLL input clock*55 */
#define  RCC_CFG_PLLMULT56                      ((uint32_t)0x601C0000)        /*!< PLL input clock*56 */
#define  RCC_CFG_PLLMULT57                      ((uint32_t)0x60200000)        /*!< PLL input clock*57 */
#define  RCC_CFG_PLLMULT58                      ((uint32_t)0x60240000)        /*!< PLL input clock*58 */
#define  RCC_CFG_PLLMULT59                      ((uint32_t)0x60280000)        /*!< PLL input clock*59 */
#define  RCC_CFG_PLLMULT60                      ((uint32_t)0x602C0000)        /*!< PLL input clock*60 */
#define  RCC_CFG_PLLMULT61                      ((uint32_t)0x60300000)        /*!< PLL input clock*61 */
#define  RCC_CFG_PLLMULT62                      ((uint32_t)0x60340000)        /*!< PLL input clock*62 */
#define  RCC_CFG_PLLMULT63                      ((uint32_t)0x60380000)        /*!< PLL input clock*63 */
#define  RCC_CFG_PLLMULT64                      ((uint32_t)0x603C0000)        /*!< PLL input clock*64 */

#define  RCC_CFG_USBPSC                     ((uint32_t)0x08C00000)        /*!< USB Device prescaler */
#define  RCC_CFG_USBPSC_0                   ((uint32_t)0x00400000)        /*!< Bit 0 */
#define  RCC_CFG_USBPSC_1                   ((uint32_t)0x00800000)        /*!< Bit 1 */
#define  RCC_CFG_USBPSC_2                   ((uint32_t)0x08000000)        /*!< Bit 2 */
#define  RCC_CFG_USBPSC_DIV1_5              ((uint32_t)0x00000000)        /*!< SYSCLK divided by 1.5 selected as USB clock source */
#define  RCC_CFG_USBPSC_DIV1                ((uint32_t)0x00400000)        /*!< SYSCLK selected as USB clock source */
#define  RCC_CFG_USBPSC_DIV2_5              ((uint32_t)0x00800000)        /*!< SYSCLK divided by 2.5 selected as USB clock source */
#define  RCC_CFG_USBPSC_DIV2                ((uint32_t)0x00C00000)        /*!< SYSCLK divided by 2 selected as USB clock source */
#define  RCC_CFG_USBPSC_DIV3_5              ((uint32_t)0x08000000)        /*!< SYSCLK divided by 3.5 selected as USB clock source */
#define  RCC_CFG_USBPSC_DIV3                ((uint32_t)0x08400000)        /*!< SYSCLK divided by 3 selected as USB clock source */
#define  RCC_CFG_USBPSC_DIV4                ((uint32_t)0x08800000)        /*!< SYSCLK divided by 4 selected as USB clock source */
#define  RCC_CFG_USBPSC_DIV4_0              ((uint32_t)0x08C00000)        /*!< SYSCLK divided by 4 selected as USB clock source */



/** CLKOUT configuration */
#define  RCC_CFG_CLKOUT                         ((uint32_t)0x07000000)        /*!< CLKOUT[2:0] bits (Microcontroller Clock Output) */
#define  RCC_CFG_CLKOUT_0                       ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  RCC_CFG_CLKOUT_1                       ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  RCC_CFG_CLKOUT_2                       ((uint32_t)0x04000000)        /*!< Bit 2 */

#define  RCC_CFG_CLKOUT_NOCLK                   ((uint32_t)0x00000000)        /*!< No clock */
#define  RCC_CFG_CLKOUT_SYSCLK                  ((uint32_t)0x04000000)        /*!< System clock selected as CLKOUT source */
#define  RCC_CFG_CLKOUT_HSI                     ((uint32_t)0x05000000)        /*!< HSI clock selected as CLKOUT source */
#define  RCC_CFG_CLKOUT_HSE                     ((uint32_t)0x06000000)        /*!< HSE clock selected as CLKOUT source  */
#define  RCC_CFG_CLKOUT_PLL_DIV2                ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as CLKOUT source */

/********************  Bit definition for RCC_CLKINT register  ********************/
#define  RCC_CLKINT_LSISTBLF                    ((uint32_t)0x00000001)        /*!< LSI Ready Interrupt flag */
#define  RCC_CLKINT_LSESTBLF                    ((uint32_t)0x00000002)        /*!< LSE Ready Interrupt flag */
#define  RCC_CLKINT_HSISTBLF                    ((uint32_t)0x00000004)        /*!< HSI Ready Interrupt flag */
#define  RCC_CLKINT_HSESTBLF                    ((uint32_t)0x00000008)        /*!< HSE Ready Interrupt flag */
#define  RCC_CLKINT_PLLSTBLF                    ((uint32_t)0x00000010)        /*!< PLL Ready Interrupt flag */
#define  RCC_CLKINT_HSECFDF                     ((uint32_t)0x00000080)        /*!< Clock Security System Interrupt flag */
#define  RCC_CIR_CSSF							RCC_CLKINT_HSECFDF
#define  RCC_CLKINT_LSISTBLIE                   ((uint32_t)0x00000100)        /*!< LSI Ready Interrupt Enable */
#define  RCC_CLKINT_LSESTBLIE                   ((uint32_t)0x00000200)        /*!< LSE Ready Interrupt Enable */
#define  RCC_CLKINT_HSISTBLIE                   ((uint32_t)0x00000400)        /*!< HSI Ready Interrupt Enable */
#define  RCC_CLKINT_HSESTBLIE                   ((uint32_t)0x00000800)        /*!< HSE Ready Interrupt Enable */
#define  RCC_CLKINT_PLLSTBLIE                   ((uint32_t)0x00001000)        /*!< PLL Ready Interrupt Enable */
#define  RCC_CLKINT_LSISTBLFC                   ((uint32_t)0x00010000)        /*!< LSI Ready Interrupt Clear */
#define  RCC_CLKINT_LSESTBLFC                   ((uint32_t)0x00020000)        /*!< LSE Ready Interrupt Clear */
#define  RCC_CLKINT_HSISTBLFC                   ((uint32_t)0x00040000)        /*!< HSI Ready Interrupt Clear */
#define  RCC_CLKINT_HSESTBLFC                   ((uint32_t)0x00080000)        /*!< HSE Ready Interrupt Clear */
#define  RCC_CLKINT_PLLSTBLFC                   ((uint32_t)0x00100000)        /*!< PLL Ready Interrupt Clear */
#define  RCC_CLKINT_HSECFDFC                    ((uint32_t)0x00800000)        /*!< Clock Security System Interrupt Clear */

/*****************  Bit definition for RCC_APB2RST register  *****************/
#define  RCC_APB2RST_AFIORST                    ((uint32_t)0x00000001)        /*!< Alternate Function I/O reset */
#define  RCC_APB2RST_GPIOARST                   ((uint32_t)0x00000004)        /*!< I/O port A reset */
#define  RCC_APB2RST_GPIOBRST                   ((uint32_t)0x00000008)        /*!< I/O port B reset */
#define  RCC_APB2RST_GPIOCRST                   ((uint32_t)0x00000010)        /*!< I/O port C reset */
#define  RCC_APB2RST_GPIODRST                   ((uint32_t)0x00000020)        /*!< I/O port D reset */
#define  RCC_APB2RST_GPIOERST                   ((uint32_t)0x00000040)        /*!< I/O port E reset */
#define  RCC_APB2RST_ADC1RST                    ((uint32_t)0x00000200)        /*!< ADC 1 interface reset */
#define  RCC_APB2RST_ADC2RST                    ((uint32_t)0x00000400)        /*!< ADC 2 interface reset */
#define  RCC_APB2RST_TMR1RST                    ((uint32_t)0x00000800)        /*!< TMR1 Timer reset */
#define  RCC_APB2RST_SPI1RST                    ((uint32_t)0x00001000)        /*!< SPI 1 reset */
#define  RCC_APB2RST_USART1RST                  ((uint32_t)0x00004000)        /*!< USART1 reset */

#define  RCC_APB2RST_TMR8RST                    ((uint32_t)0x00002000)        /*!< TMR8 Timer reset */
#define  RCC_APB2RST_ADC3RST                    ((uint32_t)0x00008000)        /*!< ADC3 interface reset */

#define  RCC_APB2RST_GPIOFRST                   ((uint32_t)0x00000080)        /*!< I/O port F reset */
#define  RCC_APB2RST_GPIOGRST                   ((uint32_t)0x00000100)        /*!< I/O port G reset */
#define  RCC_APB2RST_TMR15RST                   ((uint32_t)0x00010000)         /*!< TMR15 Timer reset */

#define  RCC_APB2RST_TMR9RST                    ((uint32_t)0x00080000)         /*!< TMR9 Timer reset */
#define  RCC_APB2RST_TMR10RST                   ((uint32_t)0x00100000)         /*!< TMR10 Timer reset */
#define  RCC_APB2RST_TMR11RST                   ((uint32_t)0x00200000)         /*!< TMR11 Timer reset */

/*****************  Bit definition for RCC_APB1RST register  *****************/
#define  RCC_APB1RST_TMR2RST                    ((uint32_t)0x00000001)        /*!< Timer 2 reset */
#define  RCC_APB1RST_TMR3RST                    ((uint32_t)0x00000002)        /*!< Timer 3 reset */
#define  RCC_APB1RST_WWDGRST                    ((uint32_t)0x00000800)        /*!< Window Watchdog reset */
#define  RCC_APB1RST_USART2RST                  ((uint32_t)0x00020000)        /*!< USART 2 reset */
#define  RCC_APB1RST_I2C1RST                    ((uint32_t)0x00200000)        /*!< I2C 1 reset */
#define  RCC_APB1RST_CAN1RST                    ((uint32_t)0x02000000)        /*!< CAN1 reset */
#define  RCC_APB1RST_BKPRST                     ((uint32_t)0x08000000)        /*!< Backup interface reset */
#define  RCC_APB1RST_PWRRST                     ((uint32_t)0x10000000)        /*!< Power interface reset */
#define  RCC_APB1RST_TMR4RST                    ((uint32_t)0x00000004)        /*!< Timer 4 reset */
#define  RCC_APB1RST_SPI2RST                    ((uint32_t)0x00004000)        /*!< SPI 2 reset */
#define  RCC_APB1RST_USART3RST                  ((uint32_t)0x00040000)        /*!< USART 3 reset */
#define  RCC_APB1RST_I2C2RST                    ((uint32_t)0x00400000)        /*!< I2C 2 reset */
#define  RCC_APB1RST_USBRST                     ((uint32_t)0x00800000)        /*!< USB Device reset */

#define  RCC_APB1RST_TMR5RST                    ((uint32_t)0x00000008)        /*!< Timer 5 reset */
#define  RCC_APB1RST_TMR6RST                    ((uint32_t)0x00000010)        /*!< Timer 6 reset */
#define  RCC_APB1RST_TMR7RST                    ((uint32_t)0x00000020)        /*!< Timer 7 reset */
#define  RCC_APB1RST_SPI3RST                    ((uint32_t)0x00008000)        /*!< SPI 3 reset */
#define  RCC_APB1RST_UART4RST                   ((uint32_t)0x00080000)        /*!< UART 4 reset */
#define  RCC_APB1RST_UART5RST                   ((uint32_t)0x00100000)        /*!< UART 5 reset */
#define  RCC_APB1RST_I2C3RST                    ((uint32_t)0x04000000)        /*!< I2C 3 reset */
#define  RCC_APB1RST_DACRST                     ((uint32_t)0x20000000)        /*!< DAC interface reset */

#if defined (AT32F403Vx_HD) || defined (AT32F403Zx_HD) || \
    defined (AT32F403Vx_XL) || defined (AT32F403Zx_XL)
#define  RCC_APB1RST_SPI4RST                    ((uint32_t)0x00010000)        /*!< SPI 4 reset */
#endif /* AT32F403Vx_HD || AT32F403Zx_HD || AT32F403Vx_XL || AT32F403Zx_XL */

#define  RCC_APB1RST_TMR12RST                   ((uint32_t)0x00000040)         /*!< TMR12 Timer reset */
#define  RCC_APB1RST_TMR13RST                   ((uint32_t)0x00000080)         /*!< TMR13 Timer reset */
#define  RCC_APB1RST_TMR14RST                   ((uint32_t)0x00000100)         /*!< TMR14 Timer reset */

/******************  Bit definition for RCC_AHBEN register  ******************/
#define  RCC_AHBEN_DMA1EN                       ((uint16_t)0x0001)            /*!< DMA1 clock enable */
#define  RCC_AHBENR_DMA1EN						RCC_AHBEN_DMA1EN
#define  RCC_AHBEN_DMA2EN                       ((uint16_t)0x0002)            /*!< DMA2 clock enable */
#define  RCC_AHBENR_DMA2EN						RCC_AHBEN_DMA2EN
#define  RCC_AHBEN_SRAMEN                       ((uint16_t)0x0004)            /*!< SRAM interface clock enable */
#define  RCC_AHBEN_FLASHEN                      ((uint16_t)0x0010)            /*!< FLITF clock enable */
#define  RCC_AHBEN_CRCEN                        ((uint16_t)0x0040)            /*!< CRC clock enable */

#define  RCC_AHBEN_SDIO1EN                      ((uint16_t)0x0400)            /*!< SDIO clock enable */
#define  RCC_AHBEN_SDIO2EN                      ((uint16_t)0x0800)            /*!< SDIO2 clock enable */

#if defined (AT32F403Vx_HD) || defined (AT32F403Zx_HD) || \
    defined (AT32F403Vx_XL) || defined (AT32F403Zx_XL)
#define  RCC_AHBEN_XMCEN                        ((uint16_t)0x0100)            /*!< XMC clock enable */
#endif /* AT32F403Vx_HD || AT32F403Zx_HD || AT32F403Vx_XL || AT32F403Zx_XL */


/******************  Bit definition for RCC_APB2EN register  *****************/
#define  RCC_APB2EN_AFIOEN                      ((uint32_t)0x00000001)         /*!< Alternate Function I/O clock enable */
#define  RCC_APB2EN_GPIOAEN                     ((uint32_t)0x00000004)         /*!< I/O port A clock enable */
#define  RCC_APB2EN_GPIOBEN                     ((uint32_t)0x00000008)         /*!< I/O port B clock enable */
#define  RCC_APB2EN_GPIOCEN                     ((uint32_t)0x00000010)         /*!< I/O port C clock enable */
#define  RCC_APB2EN_GPIODEN                     ((uint32_t)0x00000020)         /*!< I/O port D clock enable */
#define  RCC_APB2EN_GPIOEEN                     ((uint32_t)0x00000040)         /*!< I/O port E clock enable */
#define  RCC_APB2EN_ADC1EN                      ((uint32_t)0x00000200)         /*!< ADC 1 interface clock enable */
#define  RCC_APB2EN_ADC2EN                      ((uint32_t)0x00000400)         /*!< ADC 2 interface clock enable */
#define  RCC_APB2EN_TMR1EN                      ((uint32_t)0x00000800)         /*!< TMR1 Timer clock enable */
#define  RCC_APB2EN_SPI1EN                      ((uint32_t)0x00001000)         /*!< SPI 1 clock enable */
#define  RCC_APB2EN_USART1EN                    ((uint32_t)0x00004000)         /*!< USART1 clock enable */

#define  RCC_APB2EN_TMR8EN                      ((uint32_t)0x00002000)         /*!< TMR8 Timer clock enable */
#define  RCC_APB2EN_ADC3EN                      ((uint32_t)0x00008000)         /*!< DMA1 clock enable */

#define  RCC_APB2EN_GPIOFEN                     ((uint32_t)0x00000080)         /*!< I/O port F clock enable */
#define  RCC_APB2EN_GPIOGEN                     ((uint32_t)0x00000100)         /*!< I/O port G clock enable */
#define  RCC_APB2EN_TMR15EN                     ((uint32_t)0x00010000)         /*!< TMR15 Timer clock enable */

#define  RCC_APB2EN_TMR9EN                      ((uint32_t)0x00080000)         /*!< TMR9 Timer clock enable  */
#define  RCC_APB2EN_TMR10EN                     ((uint32_t)0x00100000)         /*!< TMR10 Timer clock enable  */
#define  RCC_APB2EN_TMR11EN                     ((uint32_t)0x00200000)         /*!< TMR11 Timer clock enable */


/*****************  Bit definition for RCC_APB1EN register  ******************/
#define  RCC_APB1EN_TMR2EN                      ((uint32_t)0x00000001)        /*!< Timer 2 clock enabled*/
#define  RCC_APB1ENR_TIM2EN 					RCC_APB1EN_TMR2EN
#define  RCC_APB1EN_TMR3EN                      ((uint32_t)0x00000002)        /*!< Timer 3 clock enable */
#define  RCC_APB1ENR_TIM3EN						RCC_APB1EN_TMR3EN
#define  RCC_APB1EN_WWDGEN                      ((uint32_t)0x00000800)        /*!< Window Watchdog clock enable */
#define  RCC_APB1EN_USART2EN                    ((uint32_t)0x00020000)        /*!< USART 2 clock enable */
#define  RCC_APB1EN_I2C1EN                      ((uint32_t)0x00200000)        /*!< I2C 1 clock enable */
#define  RCC_APB1ENR_I2C2EN                   	RCC_APB1EN_I2C1EN
#define  RCC_APB1EN_CAN1EN                      ((uint32_t)0x02000000)        /*!< CAN1 clock enable */
#define  RCC_APB1EN_BKPEN                       ((uint32_t)0x08000000)        /*!< Backup interface clock enable */
#define  RCC_APB1EN_PWREN                       ((uint32_t)0x10000000)        /*!< Power interface clock enable */
#define  RCC_APB1EN_TMR4EN                      ((uint32_t)0x00000004)        /*!< Timer 4 clock enable */
#define  RCC_APB1EN_SPI2EN                      ((uint32_t)0x00004000)        /*!< SPI 2 clock enable */
#define  RCC_APB1EN_USART3EN                    ((uint32_t)0x00040000)        /*!< USART 3 clock enable */
#define  RCC_APB1ENR_USART3EN					RCC_APB1EN_USART3EN
#define  RCC_APB1EN_I2C2EN                      ((uint32_t)0x00400000)        /*!< I2C 2 clock enable */
#define  RCC_APB1EN_USBEN                       ((uint32_t)0x00800000)        /*!< USB Device clock enable */

#define  RCC_APB1EN_TMR5EN                      ((uint32_t)0x00000008)        /*!< Timer 5 clock enable */
#define  RCC_APB1EN_TMR6EN                      ((uint32_t)0x00000010)        /*!< Timer 6 clock enable */
#define  RCC_APB1EN_TMR7EN                      ((uint32_t)0x00000020)        /*!< Timer 7 clock enable */
#define  RCC_APB1EN_SPI3EN                      ((uint32_t)0x00008000)        /*!< SPI 3 clock enable */
#define  RCC_APB1EN_I2C3EN                      ((uint32_t)0x04000000)        /*!< I2C 3 clock enable */
#define  RCC_APB1EN_UART4EN                     ((uint32_t)0x00080000)        /*!< UART 4 clock enable */
#define  RCC_APB1EN_UART5EN                     ((uint32_t)0x00100000)        /*!< UART 5 clock enable */
#define  RCC_APB1EN_DACEN                       ((uint32_t)0x20000000)        /*!< DAC interface clock enable */

#if defined (AT32F403Vx_HD) || defined (AT32F403Zx_HD) || \
    defined (AT32F403Vx_XL) || defined (AT32F403Zx_XL)
#define  RCC_APB1EN_SPI4EN                      ((uint32_t)0x00010000)        /*!< SPI 4 clock enable */
#endif /* AT32F403Vx_HD || AT32F403Zx_HD || AT32F403Vx_XL || AT32F403Zx_XL */

#define  RCC_APB1EN_TMR12EN                     ((uint32_t)0x00000040)         /*!< TMR12 Timer clock enable  */
#define  RCC_APB1EN_TMR13EN                     ((uint32_t)0x00000080)         /*!< TMR13 Timer clock enable  */
#define  RCC_APB1EN_TMR14EN                     ((uint32_t)0x00000100)         /*!< TMR14 Timer clock enable */

/*******************  Bit definition for RCC_BDC register  *******************/
#define  RCC_BDC_LSEEN                          ((uint32_t)0x00000001)        /*!< External Low Speed oscillator enable */
#define  RCC_BDCR_LSEON							RCC_BDC_LSEEN                          
#define  RCC_BDC_LSESTBL                        ((uint32_t)0x00000002)        /*!< External Low Speed oscillator Ready */
#define  RCC_BDC_LSEBYPS                        ((uint32_t)0x00000004)        /*!< External Low Speed oscillator Bypass */
#define  RCC_BDCR_LSEBYP						RCC_BDC_LSEBYPS                        

#define  RCC_BDC_RTCSEL                         ((uint32_t)0x00000300)        /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define  RCC_BDC_RTCSEL_0                       ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  RCC_BDC_RTCSEL_1                       ((uint32_t)0x00000200)        /*!< Bit 1 */

/** RTC congiguration */
#define  RCC_BDC_RTCSEL_NOCLK                   ((uint32_t)0x00000000)        /*!< No clock */
#define  RCC_BDCR_RTCSEL_NOCLOCK				RCC_BDC_RTCSEL_NOCLK
#define  RCC_BDC_RTCSEL_LSE                     ((uint32_t)0x00000100)        /*!< LSE oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_LSE					RCC_BDC_RTCSEL_LSE
#define  RCC_BDC_RTCSEL_LSI                     ((uint32_t)0x00000200)        /*!< LSI oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_LSI      				RCC_BDC_RTCSEL_LSI            
#define  RCC_BDC_RTCSEL_HSE_DIV128              ((uint32_t)0x00000300)        /*!< HSE oscillator clock divided by 128 used as RTC clock */
#define  RCC_BDCR_RTCSEL_HSE                  	RCC_BDC_RTCSEL_HSE_DIV128

//Copied from stm file
#define RCC_BDCR_RTCEN_Pos                   (15U)                             
#define RCC_BDCR_RTCEN_Msk                   (0x1U << RCC_BDCR_RTCEN_Pos)      /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                       RCC_BDCR_RTCEN_Msk                /*!< RTC clock enable */
#define RCC_BDCR_BDRST_Pos                   (16U)                             
#define RCC_BDCR_BDRST_Msk                   (0x1U << RCC_BDCR_BDRST_Pos)      /*!< 0x00010000 */
#define RCC_BDCR_BDRST                       RCC_BDCR_BDRST_Msk                /*!< Backup domain software reset  */


#define  RCC_BDC_RTCEN                          ((uint32_t)0x00008000)        /*!< RTC clock enable */
#define  RCC_BDC_BDRST                          ((uint32_t)0x00010000)        /*!< Backup domain software reset  */

/*******************  Bit definition for RCC_CTRLSTS register  ********************/
#define  RCC_CTRLSTS_LSIEN                      ((uint32_t)0x00000001)        /*!< Internal Low Speed oscillator enable */
#define  RCC_CSR_LSION							RCC_CTRLSTS_LSIEN
#define  RCC_CTRLSTS_LSISTBL                    ((uint32_t)0x00000002)        /*!< Internal Low Speed oscillator Ready */
#define  RCC_CTRLSTS_RSTFC                      ((uint32_t)0x01000000)        /*!< Remove reset flag */
#define  RCC_CTRLSTS_PINRSTF                    ((uint32_t)0x04000000)        /*!< PIN reset flag */
#define  RCC_CTRLSTS_PORSTF                     ((uint32_t)0x08000000)        /*!< POR/PDR reset flag */
#define  RCC_CTRLSTS_SWRSTF                     ((uint32_t)0x10000000)        /*!< Software Reset flag */
#define  RCC_CTRLSTS_IWDGRSTF                   ((uint32_t)0x20000000)        /*!< Independent Watchdog reset flag */
#define  RCC_CTRLSTS_WWDGRSTF                   ((uint32_t)0x40000000)        /*!< Window watchdog reset flag */
#define  RCC_CTRLSTS_LPRSTF                     ((uint32_t)0x80000000)        /*!< Low-Power reset flag */

/** MISC configuration */
#define  RCC_MISC_HSICAL_KEY                    ((uint32_t)0x000000FF)        /*!< HSICAL KEY */
#define  RCC_MISC_CLKOUT_3                      ((uint32_t)0x00010000)        /*!< CLKOUT Bit 3 */
#define  RCC_MISC_USB768B                       ((uint32_t)0x01000000)        /*!< USB SRAM size */



/******************************************************************************/
/*                                                                            */
/*                General Purpose and Alternate Function I/O                  */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for GPIO_CTRLL register  *******************/
#define  GPIO_CTRLL_MDE                         ((uint32_t)0x33333333)        /*!< Port x mode bits */

#define  GPIO_CTRLL_MDE0                        ((uint32_t)0x00000003)        /*!< MODE0[1:0] bits (Port x mode bits, pin 0) */
//#define GPIO_CRL_MODE0 GPIO_CTRLL_MDE0
#define  GPIO_CTRLL_MDE0_0                      ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  GPIO_CTRLL_MDE0_1                      ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  GPIO_CTRLL_MDE1                        ((uint32_t)0x00000030)        /*!< MODE1[1:0] bits (Port x mode bits, pin 1) */
#define  GPIO_CTRLL_MDE1_0                      ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  GPIO_CTRLL_MDE1_1                      ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  GPIO_CTRLL_MDE2                        ((uint32_t)0x00000300)        /*!< MODE2[1:0] bits (Port x mode bits, pin 2) */
#define  GPIO_CTRLL_MDE2_0                      ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  GPIO_CTRLL_MDE2_1                      ((uint32_t)0x00000200)        /*!< Bit 1 */

#define  GPIO_CTRLL_MDE3                        ((uint32_t)0x00003000)        /*!< MODE3[1:0] bits (Port x mode bits, pin 3) */
#define  GPIO_CTRLL_MDE3_0                      ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  GPIO_CTRLL_MDE3_1                      ((uint32_t)0x00002000)        /*!< Bit 1 */

#define  GPIO_CTRLL_MDE4                        ((uint32_t)0x00030000)        /*!< MODE4[1:0] bits (Port x mode bits, pin 4) */
#define  GPIO_CTRLL_MDE4_0                      ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  GPIO_CTRLL_MDE4_1                      ((uint32_t)0x00020000)        /*!< Bit 1 */

#define  GPIO_CTRLL_MDE5                        ((uint32_t)0x00300000)        /*!< MODE5[1:0] bits (Port x mode bits, pin 5) */
#define  GPIO_CTRLL_MDE5_0                      ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  GPIO_CTRLL_MDE5_1                      ((uint32_t)0x00200000)        /*!< Bit 1 */

#define  GPIO_CTRLL_MDE6                        ((uint32_t)0x03000000)        /*!< MODE6[1:0] bits (Port x mode bits, pin 6) */
#define  GPIO_CTRLL_MDE6_0                      ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  GPIO_CTRLL_MDE6_1                      ((uint32_t)0x02000000)        /*!< Bit 1 */

#define  GPIO_CTRLL_MDE7                        ((uint32_t)0x30000000)        /*!< MODE7[1:0] bits (Port x mode bits, pin 7) */
#define  GPIO_CTRLL_MDE7_0                      ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  GPIO_CTRLL_MDE7_1                      ((uint32_t)0x20000000)        /*!< Bit 1 */

#define  GPIO_CTRLL_CONF                        ((uint32_t)0xCCCCCCCC)        /*!< Port x configuration bits */

#define  GPIO_CTRLL_CONF0                       ((uint32_t)0x0000000C)        /*!< CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define  GPIO_CTRLL_CONF0_0                     ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  GPIO_CTRLL_CONF0_1                     ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  GPIO_CTRLL_CONF1                       ((uint32_t)0x000000C0)        /*!< CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define  GPIO_CTRLL_CONF1_0                     ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  GPIO_CTRLL_CONF1_1                     ((uint32_t)0x00000080)        /*!< Bit 1 */

#define  GPIO_CTRLL_CONF2                       ((uint32_t)0x00000C00)        /*!< CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define  GPIO_CTRLL_CONF2_0                     ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  GPIO_CTRLL_CONF2_1                     ((uint32_t)0x00000800)        /*!< Bit 1 */

#define  GPIO_CTRLL_CONF3                       ((uint32_t)0x0000C000)        /*!< CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define  GPIO_CTRLL_CONF3_0                     ((uint32_t)0x00004000)        /*!< Bit 0 */
#define  GPIO_CTRLL_CONF3_1                     ((uint32_t)0x00008000)        /*!< Bit 1 */

#define  GPIO_CTRLL_CONF4                       ((uint32_t)0x000C0000)        /*!< CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define  GPIO_CTRLL_CONF4_0                     ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  GPIO_CTRLL_CONF4_1                     ((uint32_t)0x00080000)        /*!< Bit 1 */

#define  GPIO_CTRLL_CONF5                       ((uint32_t)0x00C00000)        /*!< CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define  GPIO_CTRLL_CONF5_0                     ((uint32_t)0x00400000)        /*!< Bit 0 */
#define  GPIO_CTRLL_CONF5_1                     ((uint32_t)0x00800000)        /*!< Bit 1 */

#define  GPIO_CTRLL_CONF6                       ((uint32_t)0x0C000000)        /*!< CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define  GPIO_CTRLL_CONF6_0                     ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  GPIO_CTRLL_CONF6_1                     ((uint32_t)0x08000000)        /*!< Bit 1 */

#define  GPIO_CTRLL_CONF7                       ((uint32_t)0xC0000000)        /*!< CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define  GPIO_CTRLL_CONF7_0                     ((uint32_t)0x40000000)        /*!< Bit 0 */
#define  GPIO_CTRLL_CONF7_1                     ((uint32_t)0x80000000)        /*!< Bit 1 */

/////////////////////////////////////////////////////////////////////////////////
//Copied from stm file
/*******************  Bit definition for GPIO_CRL register  *******************/
#define GPIO_CRL_MODE_Pos                    (0U)                              
#define GPIO_CRL_MODE_Msk                    (0x33333333U << GPIO_CRL_MODE_Pos) /*!< 0x33333333 */
#define GPIO_CRL_MODE                        GPIO_CRL_MODE_Msk                 /*!< Port x mode bits */

#define GPIO_CRL_MODE0_Pos                   (0U)                              
#define GPIO_CRL_MODE0_Msk                   (0x3U << GPIO_CRL_MODE0_Pos)      /*!< 0x00000003 */
#define GPIO_CRL_MODE0                       GPIO_CRL_MODE0_Msk                /*!< MODE0[1:0] bits (Port x mode bits, pin 0) */
#define GPIO_CRL_MODE0_0                     (0x1U << GPIO_CRL_MODE0_Pos)      /*!< 0x00000001 */
#define GPIO_CRL_MODE0_1                     (0x2U << GPIO_CRL_MODE0_Pos)      /*!< 0x00000002 */

#define GPIO_CRL_MODE1_Pos                   (4U)                              
#define GPIO_CRL_MODE1_Msk                   (0x3U << GPIO_CRL_MODE1_Pos)      /*!< 0x00000030 */
#define GPIO_CRL_MODE1                       GPIO_CRL_MODE1_Msk                /*!< MODE1[1:0] bits (Port x mode bits, pin 1) */
#define GPIO_CRL_MODE1_0                     (0x1U << GPIO_CRL_MODE1_Pos)      /*!< 0x00000010 */
#define GPIO_CRL_MODE1_1                     (0x2U << GPIO_CRL_MODE1_Pos)      /*!< 0x00000020 */

#define GPIO_CRL_MODE2_Pos                   (8U)                              
#define GPIO_CRL_MODE2_Msk                   (0x3U << GPIO_CRL_MODE2_Pos)      /*!< 0x00000300 */
#define GPIO_CRL_MODE2                       GPIO_CRL_MODE2_Msk                /*!< MODE2[1:0] bits (Port x mode bits, pin 2) */
#define GPIO_CRL_MODE2_0                     (0x1U << GPIO_CRL_MODE2_Pos)      /*!< 0x00000100 */
#define GPIO_CRL_MODE2_1                     (0x2U << GPIO_CRL_MODE2_Pos)      /*!< 0x00000200 */

#define GPIO_CRL_MODE3_Pos                   (12U)                             
#define GPIO_CRL_MODE3_Msk                   (0x3U << GPIO_CRL_MODE3_Pos)      /*!< 0x00003000 */
#define GPIO_CRL_MODE3                       GPIO_CRL_MODE3_Msk                /*!< MODE3[1:0] bits (Port x mode bits, pin 3) */
#define GPIO_CRL_MODE3_0                     (0x1U << GPIO_CRL_MODE3_Pos)      /*!< 0x00001000 */
#define GPIO_CRL_MODE3_1                     (0x2U << GPIO_CRL_MODE3_Pos)      /*!< 0x00002000 */

#define GPIO_CRL_MODE4_Pos                   (16U)                             
#define GPIO_CRL_MODE4_Msk                   (0x3U << GPIO_CRL_MODE4_Pos)      /*!< 0x00030000 */
#define GPIO_CRL_MODE4                       GPIO_CRL_MODE4_Msk                /*!< MODE4[1:0] bits (Port x mode bits, pin 4) */
#define GPIO_CRL_MODE4_0                     (0x1U << GPIO_CRL_MODE4_Pos)      /*!< 0x00010000 */
#define GPIO_CRL_MODE4_1                     (0x2U << GPIO_CRL_MODE4_Pos)      /*!< 0x00020000 */

#define GPIO_CRL_MODE5_Pos                   (20U)                             
#define GPIO_CRL_MODE5_Msk                   (0x3U << GPIO_CRL_MODE5_Pos)      /*!< 0x00300000 */
#define GPIO_CRL_MODE5                       GPIO_CRL_MODE5_Msk                /*!< MODE5[1:0] bits (Port x mode bits, pin 5) */
#define GPIO_CRL_MODE5_0                     (0x1U << GPIO_CRL_MODE5_Pos)      /*!< 0x00100000 */
#define GPIO_CRL_MODE5_1                     (0x2U << GPIO_CRL_MODE5_Pos)      /*!< 0x00200000 */

#define GPIO_CRL_MODE6_Pos                   (24U)                             
#define GPIO_CRL_MODE6_Msk                   (0x3U << GPIO_CRL_MODE6_Pos)      /*!< 0x03000000 */
#define GPIO_CRL_MODE6                       GPIO_CRL_MODE6_Msk                /*!< MODE6[1:0] bits (Port x mode bits, pin 6) */
#define GPIO_CRL_MODE6_0                     (0x1U << GPIO_CRL_MODE6_Pos)      /*!< 0x01000000 */
#define GPIO_CRL_MODE6_1                     (0x2U << GPIO_CRL_MODE6_Pos)      /*!< 0x02000000 */

#define GPIO_CRL_MODE7_Pos                   (28U)                             
#define GPIO_CRL_MODE7_Msk                   (0x3U << GPIO_CRL_MODE7_Pos)      /*!< 0x30000000 */
#define GPIO_CRL_MODE7                       GPIO_CRL_MODE7_Msk                /*!< MODE7[1:0] bits (Port x mode bits, pin 7) */
#define GPIO_CRL_MODE7_0                     (0x1U << GPIO_CRL_MODE7_Pos)      /*!< 0x10000000 */
#define GPIO_CRL_MODE7_1                     (0x2U << GPIO_CRL_MODE7_Pos)      /*!< 0x20000000 */

#define GPIO_CRL_CNF_Pos                     (2U)                              
#define GPIO_CRL_CNF_Msk                     (0x33333333U << GPIO_CRL_CNF_Pos) /*!< 0xCCCCCCCC */
#define GPIO_CRL_CNF                         GPIO_CRL_CNF_Msk                  /*!< Port x configuration bits */

#define GPIO_CRL_CNF0_Pos                    (2U)                              
#define GPIO_CRL_CNF0_Msk                    (0x3U << GPIO_CRL_CNF0_Pos)       /*!< 0x0000000C */
#define GPIO_CRL_CNF0                        GPIO_CRL_CNF0_Msk                 /*!< CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define GPIO_CRL_CNF0_0                      (0x1U << GPIO_CRL_CNF0_Pos)       /*!< 0x00000004 */
#define GPIO_CRL_CNF0_1                      (0x2U << GPIO_CRL_CNF0_Pos)       /*!< 0x00000008 */

#define GPIO_CRL_CNF1_Pos                    (6U)                              
#define GPIO_CRL_CNF1_Msk                    (0x3U << GPIO_CRL_CNF1_Pos)       /*!< 0x000000C0 */
#define GPIO_CRL_CNF1                        GPIO_CRL_CNF1_Msk                 /*!< CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define GPIO_CRL_CNF1_0                      (0x1U << GPIO_CRL_CNF1_Pos)       /*!< 0x00000040 */
#define GPIO_CRL_CNF1_1                      (0x2U << GPIO_CRL_CNF1_Pos)       /*!< 0x00000080 */

#define GPIO_CRL_CNF2_Pos                    (10U)                             
#define GPIO_CRL_CNF2_Msk                    (0x3U << GPIO_CRL_CNF2_Pos)       /*!< 0x00000C00 */
#define GPIO_CRL_CNF2                        GPIO_CRL_CNF2_Msk                 /*!< CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define GPIO_CRL_CNF2_0                      (0x1U << GPIO_CRL_CNF2_Pos)       /*!< 0x00000400 */
#define GPIO_CRL_CNF2_1                      (0x2U << GPIO_CRL_CNF2_Pos)       /*!< 0x00000800 */

#define GPIO_CRL_CNF3_Pos                    (14U)                             
#define GPIO_CRL_CNF3_Msk                    (0x3U << GPIO_CRL_CNF3_Pos)       /*!< 0x0000C000 */
#define GPIO_CRL_CNF3                        GPIO_CRL_CNF3_Msk                 /*!< CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define GPIO_CRL_CNF3_0                      (0x1U << GPIO_CRL_CNF3_Pos)       /*!< 0x00004000 */
#define GPIO_CRL_CNF3_1                      (0x2U << GPIO_CRL_CNF3_Pos)       /*!< 0x00008000 */

#define GPIO_CRL_CNF4_Pos                    (18U)                             
#define GPIO_CRL_CNF4_Msk                    (0x3U << GPIO_CRL_CNF4_Pos)       /*!< 0x000C0000 */
#define GPIO_CRL_CNF4                        GPIO_CRL_CNF4_Msk                 /*!< CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define GPIO_CRL_CNF4_0                      (0x1U << GPIO_CRL_CNF4_Pos)       /*!< 0x00040000 */
#define GPIO_CRL_CNF4_1                      (0x2U << GPIO_CRL_CNF4_Pos)       /*!< 0x00080000 */

#define GPIO_CRL_CNF5_Pos                    (22U)                             
#define GPIO_CRL_CNF5_Msk                    (0x3U << GPIO_CRL_CNF5_Pos)       /*!< 0x00C00000 */
#define GPIO_CRL_CNF5                        GPIO_CRL_CNF5_Msk                 /*!< CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define GPIO_CRL_CNF5_0                      (0x1U << GPIO_CRL_CNF5_Pos)       /*!< 0x00400000 */
#define GPIO_CRL_CNF5_1                      (0x2U << GPIO_CRL_CNF5_Pos)       /*!< 0x00800000 */

#define GPIO_CRL_CNF6_Pos                    (26U)                             
#define GPIO_CRL_CNF6_Msk                    (0x3U << GPIO_CRL_CNF6_Pos)       /*!< 0x0C000000 */
#define GPIO_CRL_CNF6                        GPIO_CRL_CNF6_Msk                 /*!< CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define GPIO_CRL_CNF6_0                      (0x1U << GPIO_CRL_CNF6_Pos)       /*!< 0x04000000 */
#define GPIO_CRL_CNF6_1                      (0x2U << GPIO_CRL_CNF6_Pos)       /*!< 0x08000000 */

#define GPIO_CRL_CNF7_Pos                    (30U)                             
#define GPIO_CRL_CNF7_Msk                    (0x3U << GPIO_CRL_CNF7_Pos)       /*!< 0xC0000000 */
#define GPIO_CRL_CNF7                        GPIO_CRL_CNF7_Msk                 /*!< CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define GPIO_CRL_CNF7_0                      (0x1U << GPIO_CRL_CNF7_Pos)       /*!< 0x40000000 */
#define GPIO_CRL_CNF7_1                      (0x2U << GPIO_CRL_CNF7_Pos)       /*!< 0x80000000 */

/*******************  Bit definition for GPIO_CTRLH register  *******************/
#define  GPIO_CTRLH_MDE                         ((uint32_t)0x33333333)        /*!< Port x mode bits */

#define  GPIO_CTRLH_MDE8                        ((uint32_t)0x00000003)        /*!< MODE8[1:0] bits (Port x mode bits, pin 8) */
#define  GPIO_CTRLH_MDE8_0                      ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  GPIO_CTRLH_MDE8_1                      ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  GPIO_CTRLH_MDE9                        ((uint32_t)0x00000030)        /*!< MODE9[1:0] bits (Port x mode bits, pin 9) */
#define  GPIO_CTRLH_MDE9_0                      ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  GPIO_CTRLH_MDE9_1                      ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  GPIO_CTRLH_MDE10                       ((uint32_t)0x00000300)        /*!< MODE10[1:0] bits (Port x mode bits, pin 10) */
#define  GPIO_CTRLH_MDE10_0                     ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  GPIO_CTRLH_MDE10_1                     ((uint32_t)0x00000200)        /*!< Bit 1 */

#define  GPIO_CTRLH_MDE11                       ((uint32_t)0x00003000)        /*!< MODE11[1:0] bits (Port x mode bits, pin 11) */
#define  GPIO_CTRLH_MDE11_0                     ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  GPIO_CTRLH_MDE11_1                     ((uint32_t)0x00002000)        /*!< Bit 1 */

#define  GPIO_CTRLH_MDE12                       ((uint32_t)0x00030000)        /*!< MODE12[1:0] bits (Port x mode bits, pin 12) */
#define  GPIO_CTRLH_MDE12_0                     ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  GPIO_CTRLH_MDE12_1                     ((uint32_t)0x00020000)        /*!< Bit 1 */

#define  GPIO_CTRLH_MDE13                       ((uint32_t)0x00300000)        /*!< MODE13[1:0] bits (Port x mode bits, pin 13) */
#define  GPIO_CTRLH_MDE13_0                     ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  GPIO_CTRLH_MDE13_1                     ((uint32_t)0x00200000)        /*!< Bit 1 */

#define  GPIO_CTRLH_MDE14                       ((uint32_t)0x03000000)        /*!< MODE14[1:0] bits (Port x mode bits, pin 14) */
#define  GPIO_CTRLH_MDE14_0                     ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  GPIO_CTRLH_MDE14_1                     ((uint32_t)0x02000000)        /*!< Bit 1 */

#define  GPIO_CTRLH_MDE15                       ((uint32_t)0x30000000)        /*!< MODE15[1:0] bits (Port x mode bits, pin 15) */
#define  GPIO_CTRLH_MDE15_0                     ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  GPIO_CTRLH_MDE15_1                     ((uint32_t)0x20000000)        /*!< Bit 1 */

#define  GPIO_CTRLH_CONF                        ((uint32_t)0xCCCCCCCC)        /*!< Port x configuration bits */

#define  GPIO_CTRLH_CONF8                       ((uint32_t)0x0000000C)        /*!< CNF8[1:0] bits (Port x configuration bits, pin 8) */
#define  GPIO_CTRLH_CONF8_0                     ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  GPIO_CTRLH_CONF8_1                     ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  GPIO_CTRLH_CONF9                       ((uint32_t)0x000000C0)        /*!< CNF9[1:0] bits (Port x configuration bits, pin 9) */
#define  GPIO_CTRLH_CONF9_0                     ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  GPIO_CTRLH_CONF9_1                     ((uint32_t)0x00000080)        /*!< Bit 1 */

#define  GPIO_CTRLH_CONF10                      ((uint32_t)0x00000C00)        /*!< CNF10[1:0] bits (Port x configuration bits, pin 10) */
#define  GPIO_CTRLH_CONF10_0                    ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  GPIO_CTRLH_CONF10_1                    ((uint32_t)0x00000800)        /*!< Bit 1 */

#define  GPIO_CTRLH_CONF11                      ((uint32_t)0x0000C000)        /*!< CNF11[1:0] bits (Port x configuration bits, pin 11) */
#define  GPIO_CTRLH_CONF11_0                    ((uint32_t)0x00004000)        /*!< Bit 0 */
#define  GPIO_CTRLH_CONF11_1                    ((uint32_t)0x00008000)        /*!< Bit 1 */

#define  GPIO_CTRLH_CONF12                      ((uint32_t)0x000C0000)        /*!< CNF12[1:0] bits (Port x configuration bits, pin 12) */
#define  GPIO_CTRLH_CONF12_0                    ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  GPIO_CTRLH_CONF12_1                    ((uint32_t)0x00080000)        /*!< Bit 1 */

#define  GPIO_CTRLH_CONF13                      ((uint32_t)0x00C00000)        /*!< CNF13[1:0] bits (Port x configuration bits, pin 13) */
#define  GPIO_CTRLH_CONF13_0                    ((uint32_t)0x00400000)        /*!< Bit 0 */
#define  GPIO_CTRLH_CONF13_1                    ((uint32_t)0x00800000)        /*!< Bit 1 */

#define  GPIO_CTRLH_CONF14                      ((uint32_t)0x0C000000)        /*!< CNF14[1:0] bits (Port x configuration bits, pin 14) */
#define  GPIO_CTRLH_CONF14_0                    ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  GPIO_CTRLH_CONF14_1                    ((uint32_t)0x08000000)        /*!< Bit 1 */

#define  GPIO_CTRLH_CONF15                      ((uint32_t)0xC0000000)        /*!< CNF15[1:0] bits (Port x configuration bits, pin 15) */
#define  GPIO_CTRLH_CONF15_0                    ((uint32_t)0x40000000)        /*!< Bit 0 */
#define  GPIO_CTRLH_CONF15_1                    ((uint32_t)0x80000000)        /*!< Bit 1 */

/********************  Bit definition for GPIO_IPTDT register  *******************/
#define GPIO_IPTDT_IPTDT0                       ((uint16_t)0x0001)            /*!< Port input data, bit 0 */
#define GPIO_IPTDT_IPTDT1                       ((uint16_t)0x0002)            /*!< Port input data, bit 1 */
#define GPIO_IPTDT_IPTDT2                       ((uint16_t)0x0004)            /*!< Port input data, bit 2 */
#define GPIO_IPTDT_IPTDT3                       ((uint16_t)0x0008)            /*!< Port input data, bit 3 */
#define GPIO_IPTDT_IPTDT4                       ((uint16_t)0x0010)            /*!< Port input data, bit 4 */
#define GPIO_IPTDT_IPTDT5                       ((uint16_t)0x0020)            /*!< Port input data, bit 5 */
#define GPIO_IPTDT_IPTDT6                       ((uint16_t)0x0040)            /*!< Port input data, bit 6 */
#define GPIO_IPTDT_IPTDT7                       ((uint16_t)0x0080)            /*!< Port input data, bit 7 */
#define GPIO_IPTDT_IPTDT8                       ((uint16_t)0x0100)            /*!< Port input data, bit 8 */
#define GPIO_IPTDT_IPTDT9                       ((uint16_t)0x0200)            /*!< Port input data, bit 9 */
#define GPIO_IPTDT_IPTDT10                      ((uint16_t)0x0400)            /*!< Port input data, bit 10 */
#define GPIO_IPTDT_IPTDT11                      ((uint16_t)0x0800)            /*!< Port input data, bit 11 */
#define GPIO_IPTDT_IPTDT12                      ((uint16_t)0x1000)            /*!< Port input data, bit 12 */
#define GPIO_IPTDT_IPTDT13                      ((uint16_t)0x2000)            /*!< Port input data, bit 13 */
#define GPIO_IPTDT_IPTDT14                      ((uint16_t)0x4000)            /*!< Port input data, bit 14 */
#define GPIO_IPTDT_IPTDT15                      ((uint16_t)0x8000)            /*!< Port input data, bit 15 */

/*******************  Bit definition for GPIO_OPTDT register  *******************/
#define GPIO_OPTDT_OPTDT0                       ((uint16_t)0x0001)            /*!< Port output data, bit 0 */
#define GPIO_OPTDT_OPTDT1                       ((uint16_t)0x0002)            /*!< Port output data, bit 1 */
#define GPIO_OPTDT_OPTDT2                       ((uint16_t)0x0004)            /*!< Port output data, bit 2 */
#define GPIO_OPTDT_OPTDT3                       ((uint16_t)0x0008)            /*!< Port output data, bit 3 */
#define GPIO_OPTDT_OPTDT4                       ((uint16_t)0x0010)            /*!< Port output data, bit 4 */
#define GPIO_OPTDT_OPTDT5                       ((uint16_t)0x0020)            /*!< Port output data, bit 5 */
#define GPIO_OPTDT_OPTDT6                       ((uint16_t)0x0040)            /*!< Port output data, bit 6 */
#define GPIO_OPTDT_OPTDT7                       ((uint16_t)0x0080)            /*!< Port output data, bit 7 */
#define GPIO_OPTDT_OPTDT8                       ((uint16_t)0x0100)            /*!< Port output data, bit 8 */
#define GPIO_OPTDT_OPTDT9                       ((uint16_t)0x0200)            /*!< Port output data, bit 9 */
#define GPIO_OPTDT_OPTDT10                      ((uint16_t)0x0400)            /*!< Port output data, bit 10 */
#define GPIO_OPTDT_OPTDT11                      ((uint16_t)0x0800)            /*!< Port output data, bit 11 */
#define GPIO_OPTDT_OPTDT12                      ((uint16_t)0x1000)            /*!< Port output data, bit 12 */
#define GPIO_OPTDT_OPTDT13                      ((uint16_t)0x2000)            /*!< Port output data, bit 13 */
#define GPIO_OPTDT_OPTDT14                      ((uint16_t)0x4000)            /*!< Port output data, bit 14 */
#define GPIO_OPTDT_OPTDT15                      ((uint16_t)0x8000)            /*!< Port output data, bit 15 */

/******************  Bit definition for GPIO_BSRE register  *******************/
#define GPIO_BSRE_BST0                          ((uint32_t)0x00000001)        /*!< Port x Set bit 0 */
#define GPIO_BSRE_BST1                          ((uint32_t)0x00000002)        /*!< Port x Set bit 1 */
#define GPIO_BSRE_BST2                          ((uint32_t)0x00000004)        /*!< Port x Set bit 2 */
#define GPIO_BSRE_BST3                          ((uint32_t)0x00000008)        /*!< Port x Set bit 3 */
#define GPIO_BSRE_BST4                          ((uint32_t)0x00000010)        /*!< Port x Set bit 4 */
#define GPIO_BSRE_BST5                          ((uint32_t)0x00000020)        /*!< Port x Set bit 5 */
#define GPIO_BSRE_BST6                          ((uint32_t)0x00000040)        /*!< Port x Set bit 6 */
#define GPIO_BSRE_BST7                          ((uint32_t)0x00000080)        /*!< Port x Set bit 7 */
#define GPIO_BSRE_BST8                          ((uint32_t)0x00000100)        /*!< Port x Set bit 8 */
#define GPIO_BSRE_BST9                          ((uint32_t)0x00000200)        /*!< Port x Set bit 9 */
#define GPIO_BSRE_BST10                         ((uint32_t)0x00000400)        /*!< Port x Set bit 10 */
#define GPIO_BSRE_BST11                         ((uint32_t)0x00000800)        /*!< Port x Set bit 11 */
#define GPIO_BSRE_BST12                         ((uint32_t)0x00001000)        /*!< Port x Set bit 12 */
#define GPIO_BSRE_BST13                         ((uint32_t)0x00002000)        /*!< Port x Set bit 13 */
#define GPIO_BSRE_BST14                         ((uint32_t)0x00004000)        /*!< Port x Set bit 14 */
#define GPIO_BSRE_BST15                         ((uint32_t)0x00008000)        /*!< Port x Set bit 15 */

#define GPIO_BSRE_BRE0                          ((uint32_t)0x00010000)        /*!< Port x Reset bit 0 */
#define GPIO_BSRE_BRE1                          ((uint32_t)0x00020000)        /*!< Port x Reset bit 1 */
#define GPIO_BSRE_BRE2                          ((uint32_t)0x00040000)        /*!< Port x Reset bit 2 */
#define GPIO_BSRE_BRE3                          ((uint32_t)0x00080000)        /*!< Port x Reset bit 3 */
#define GPIO_BSRE_BRE4                          ((uint32_t)0x00100000)        /*!< Port x Reset bit 4 */
#define GPIO_BSRE_BRE5                          ((uint32_t)0x00200000)        /*!< Port x Reset bit 5 */
#define GPIO_BSRE_BRE6                          ((uint32_t)0x00400000)        /*!< Port x Reset bit 6 */
#define GPIO_BSRE_BRE7                          ((uint32_t)0x00800000)        /*!< Port x Reset bit 7 */
#define GPIO_BSRE_BRE8                          ((uint32_t)0x01000000)        /*!< Port x Reset bit 8 */
#define GPIO_BSRE_BRE9                          ((uint32_t)0x02000000)        /*!< Port x Reset bit 9 */
#define GPIO_BSRE_BRE10                         ((uint32_t)0x04000000)        /*!< Port x Reset bit 10 */
#define GPIO_BSRE_BRE11                         ((uint32_t)0x08000000)        /*!< Port x Reset bit 11 */
#define GPIO_BSRE_BRE12                         ((uint32_t)0x10000000)        /*!< Port x Reset bit 12 */
#define GPIO_BSRE_BRE13                         ((uint32_t)0x20000000)        /*!< Port x Reset bit 13 */
#define GPIO_BSRE_BRE14                         ((uint32_t)0x40000000)        /*!< Port x Reset bit 14 */
#define GPIO_BSRE_BRE15                         ((uint32_t)0x80000000)        /*!< Port x Reset bit 15 */

/*******************  Bit definition for GPIO_BRE register  *******************/
#define GPIO_BRE_BRE0                           ((uint16_t)0x0001)            /*!< Port x Reset bit 0 */
#define GPIO_BRE_BRE1                           ((uint16_t)0x0002)            /*!< Port x Reset bit 1 */
#define GPIO_BRE_BRE2                           ((uint16_t)0x0004)            /*!< Port x Reset bit 2 */
#define GPIO_BRE_BRE3                           ((uint16_t)0x0008)            /*!< Port x Reset bit 3 */
#define GPIO_BRE_BRE4                           ((uint16_t)0x0010)            /*!< Port x Reset bit 4 */
#define GPIO_BRE_BRE5                           ((uint16_t)0x0020)            /*!< Port x Reset bit 5 */
#define GPIO_BRE_BRE6                           ((uint16_t)0x0040)            /*!< Port x Reset bit 6 */
#define GPIO_BRE_BRE7                           ((uint16_t)0x0080)            /*!< Port x Reset bit 7 */
#define GPIO_BRE_BRE8                           ((uint16_t)0x0100)            /*!< Port x Reset bit 8 */
#define GPIO_BRE_BRE9                           ((uint16_t)0x0200)            /*!< Port x Reset bit 9 */
#define GPIO_BRE_BRE10                          ((uint16_t)0x0400)            /*!< Port x Reset bit 10 */
#define GPIO_BRE_BRE11                          ((uint16_t)0x0800)            /*!< Port x Reset bit 11 */
#define GPIO_BRE_BRE12                          ((uint16_t)0x1000)            /*!< Port x Reset bit 12 */
#define GPIO_BRE_BRE13                          ((uint16_t)0x2000)            /*!< Port x Reset bit 13 */
#define GPIO_BRE_BRE14                          ((uint16_t)0x4000)            /*!< Port x Reset bit 14 */
#define GPIO_BRE_BRE15                          ((uint16_t)0x8000)            /*!< Port x Reset bit 15 */

/******************  Bit definition for GPIO_LOCK register  *******************/
#define GPIO_LOCK_LOCK0                         ((uint32_t)0x00000001)        /*!< Port x Lock bit 0 */
#define GPIO_LOCK_LOCK1                         ((uint32_t)0x00000002)        /*!< Port x Lock bit 1 */
#define GPIO_LOCK_LOCK2                         ((uint32_t)0x00000004)        /*!< Port x Lock bit 2 */
#define GPIO_LOCK_LOCK3                         ((uint32_t)0x00000008)        /*!< Port x Lock bit 3 */
#define GPIO_LOCK_LOCK4                         ((uint32_t)0x00000010)        /*!< Port x Lock bit 4 */
#define GPIO_LOCK_LOCK5                         ((uint32_t)0x00000020)        /*!< Port x Lock bit 5 */
#define GPIO_LOCK_LOCK6                         ((uint32_t)0x00000040)        /*!< Port x Lock bit 6 */
#define GPIO_LOCK_LOCK7                         ((uint32_t)0x00000080)        /*!< Port x Lock bit 7 */
#define GPIO_LOCK_LOCK8                         ((uint32_t)0x00000100)        /*!< Port x Lock bit 8 */
#define GPIO_LOCK_LOCK9                         ((uint32_t)0x00000200)        /*!< Port x Lock bit 9 */
#define GPIO_LOCK_LOCK10                        ((uint32_t)0x00000400)        /*!< Port x Lock bit 10 */
#define GPIO_LOCK_LOCK11                        ((uint32_t)0x00000800)        /*!< Port x Lock bit 11 */
#define GPIO_LOCK_LOCK12                        ((uint32_t)0x00001000)        /*!< Port x Lock bit 12 */
#define GPIO_LOCK_LOCK13                        ((uint32_t)0x00002000)        /*!< Port x Lock bit 13 */
#define GPIO_LOCK_LOCK14                        ((uint32_t)0x00004000)        /*!< Port x Lock bit 14 */
#define GPIO_LOCK_LOCK15                        ((uint32_t)0x00008000)        /*!< Port x Lock bit 15 */
#define GPIO_LOCK_LOCKK                         ((uint32_t)0x00010000)        /*!< Lock key */
#define GPIO_LCKR_LCKK							GPIO_LOCK_LOCKK

/*----------------------------------------------------------------------------*/

/******************  Bit definition for AFIO_EVCTRL register  *******************/
#define AFIO_EVCTRL_PIN                         ((uint8_t)0x0F)               /*!< PIN[3:0] bits (Pin selection) */
#define AFIO_EVCR_PIN							AFIO_EVCTRL_PIN
#define AFIO_EVCTRL_PIN_B0                      ((uint8_t)0x01)               /*!< Bit 0 */
#define AFIO_EVCTRL_PIN_B1                      ((uint8_t)0x02)               /*!< Bit 1 */
#define AFIO_EVCTRL_PIN_B2                      ((uint8_t)0x04)               /*!< Bit 2 */
#define AFIO_EVCTRL_PIN_B3                      ((uint8_t)0x08)               /*!< Bit 3 */

/** PIN configuration */
#define AFIO_EVCTRL_PIN_PN0                     ((uint8_t)0x00)               /*!< Pin 0 selected */
#define AFIO_EVCTRL_PIN_PN1                     ((uint8_t)0x01)               /*!< Pin 1 selected */
#define AFIO_EVCTRL_PIN_PN2                     ((uint8_t)0x02)               /*!< Pin 2 selected */
#define AFIO_EVCTRL_PIN_PN3                     ((uint8_t)0x03)               /*!< Pin 3 selected */
#define AFIO_EVCTRL_PIN_PN4                     ((uint8_t)0x04)               /*!< Pin 4 selected */
#define AFIO_EVCTRL_PIN_PN5                     ((uint8_t)0x05)               /*!< Pin 5 selected */
#define AFIO_EVCTRL_PIN_PN6                     ((uint8_t)0x06)               /*!< Pin 6 selected */
#define AFIO_EVCTRL_PIN_PN7                     ((uint8_t)0x07)               /*!< Pin 7 selected */
#define AFIO_EVCTRL_PIN_PN8                     ((uint8_t)0x08)               /*!< Pin 8 selected */
#define AFIO_EVCTRL_PIN_PN9                     ((uint8_t)0x09)               /*!< Pin 9 selected */
#define AFIO_EVCTRL_PIN_PN10                    ((uint8_t)0x0A)               /*!< Pin 10 selected */
#define AFIO_EVCTRL_PIN_PN11                    ((uint8_t)0x0B)               /*!< Pin 11 selected */
#define AFIO_EVCTRL_PIN_PN12                    ((uint8_t)0x0C)               /*!< Pin 12 selected */
#define AFIO_EVCTRL_PIN_PN13                    ((uint8_t)0x0D)               /*!< Pin 13 selected */
#define AFIO_EVCTRL_PIN_PN14                    ((uint8_t)0x0E)               /*!< Pin 14 selected */
#define AFIO_EVCTRL_PIN_PN15                    ((uint8_t)0x0F)               /*!< Pin 15 selected */

#define AFIO_EVCTRL_PORT                        ((uint8_t)0x70)               /*!< PORT[2:0] bits (Port selection) */
#define AFIO_EVCR_PORT							AFIO_EVCTRL_PORT
#define AFIO_EVCTRL_PORT_B0                     ((uint8_t)0x10)               /*!< Bit 0 */
#define AFIO_EVCTRL_PORT_B1                     ((uint8_t)0x20)               /*!< Bit 1 */
#define AFIO_EVCTRL_PORT_B2                     ((uint8_t)0x40)               /*!< Bit 2 */

/** PORT configuration */
#define AFIO_EVCTRL_PORT_PTA                    ((uint8_t)0x00)               /*!< Port A selected */
#define AFIO_EVCTRL_PORT_PTB                    ((uint8_t)0x10)               /*!< Port B selected */
#define AFIO_EVCTRL_PORT_PTC                    ((uint8_t)0x20)               /*!< Port C selected */
#define AFIO_EVCTRL_PORT_PTD                    ((uint8_t)0x30)               /*!< Port D selected */
#define AFIO_EVCTRL_PORT_PTE                    ((uint8_t)0x40)               /*!< Port E selected */

#define AFIO_EVCTRL_EVOEN                       ((uint8_t)0x80)               /*!< Event Output Enable */
#define AFIO_EVCR_EVOE							AFIO_EVCTRL_EVOEN
/******************  Bit definition for AFIO_MAP register  *******************/
#define AFIO_MAP_SPI1_REMAP                     ((uint32_t)0x00000001)        /*!< SPI1 remapping */
#define AFIO_MAP_I2C1_REMAP                     ((uint32_t)0x00000002)        /*!< I2C1 remapping */
#define AFIO_MAP_USART1_REMAP                   ((uint32_t)0x00000004)        /*!< USART1 remapping */
#define AFIO_MAP_USART2_REMAP                   ((uint32_t)0x00000008)        /*!< USART2 remapping */

#define AFIO_MAP_USART3_REMAP                   ((uint32_t)0x00000030)        /*!< USART3_REMAP[1:0] bits (USART3 remapping) */
#define AFIO_MAP_USART3_REMAP_B0                ((uint32_t)0x00000010)        /*!< Bit 0 */
#define AFIO_MAP_USART3_REMAP_B1                ((uint32_t)0x00000020)        /*!< Bit 1 */

/** USART3_REMAP configuration */
#define AFIO_MAP_USART3_REMAP_NONEREMAP         ((uint32_t)0x00000000)        /*!< No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14) */
#define AFIO_MAP_USART3_REMAP_PARTIALREMAP      ((uint32_t)0x00000010)        /*!< Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14) */
#define AFIO_MAP_USART3_REMAP_FULLREMAP         ((uint32_t)0x00000030)        /*!< Full remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12) */

#define AFIO_MAP_TMR1_REMAP                     ((uint32_t)0x000000C0)        /*!< TMR1_REMAP[1:0] bits (TMR1 remapping) */
#define AFIO_MAP_TMR1_REMAP_B0                  ((uint32_t)0x00000040)        /*!< Bit 0 */
#define AFIO_MAP_TMR1_REMAP_B1                  ((uint32_t)0x00000080)        /*!< Bit 1 */

/** TMR1_REMAP configuration */
#define AFIO_MAP_TMR1_REMAP_NONEREMAP           ((uint32_t)0x00000000)        /*!< No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15) */
#define AFIO_MAP_TMR1_REMAP_PARTIALREMAP        ((uint32_t)0x00000040)        /*!< Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1) */
#define AFIO_MAP_TMR1_REMAP_FULLREMAP           ((uint32_t)0x000000C0)        /*!< Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12) */

#define AFIO_MAP_TMR2_REMAP                     ((uint32_t)0x00000300)        /*!< TMR2_REMAP[1:0] bits (TMR2 remapping) */
#define AFIO_MAP_TMR2_REMAP_B0                  ((uint32_t)0x00000100)        /*!< Bit 0 */
#define AFIO_MAP_TMR2_REMAP_B1                  ((uint32_t)0x00000200)        /*!< Bit 1 */

/** TMR2_REMAP configuration */
#define AFIO_MAP_TMR2_REMAP_NONEREMAP           ((uint32_t)0x00000000)        /*!< No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3) */
#define AFIO_MAP_TMR2_REMAP_PARTIALREMAP1       ((uint32_t)0x00000100)        /*!< Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3) */
#define AFIO_MAP_TMR2_REMAP_PARTIALREMAP2       ((uint32_t)0x00000200)        /*!< Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11) */
#define AFIO_MAP_TMR2_REMAP_FULLREMAP           ((uint32_t)0x00000300)        /*!< Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11) */

#define AFIO_MAP_TMR3_REMAP                     ((uint32_t)0x00000C00)        /*!< TMR3_REMAP[1:0] bits (TMR3 remapping) */
#define AFIO_MAP_TMR3_REMAP_B0                  ((uint32_t)0x00000400)        /*!< Bit 0 */
#define AFIO_MAP_TMR3_REMAP_B1                  ((uint32_t)0x00000800)        /*!< Bit 1 */

/** TMR3_REMAP configuration */
#define AFIO_MAP_TMR3_REMAP_NONEREMAP           ((uint32_t)0x00000000)        /*!< No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1) */
#define AFIO_MAP_TMR3_REMAP_PARTIALREMAP        ((uint32_t)0x00000800)        /*!< Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) */
#define AFIO_MAP_TMR3_REMAP_FULLREMAP           ((uint32_t)0x00000C00)        /*!< Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9) */

#define AFIO_MAP_TMR4_REMAP                     ((uint32_t)0x00001000)        /*!< TMR4_REMAP bit (TMR4 remapping) */

#define AFIO_MAP_CAN_REMAP                      ((uint32_t)0x00006000)        /*!< CAN_REMAP[1:0] bits (CAN Alternate function remapping) */
#define AFIO_MAP_CAN_REMAP_B0                   ((uint32_t)0x00002000)        /*!< Bit 0 */
#define AFIO_MAP_CAN_REMAP_B1                   ((uint32_t)0x00004000)        /*!< Bit 1 */

/** CAN_REMAP configuration */
#define AFIO_MAP_CAN_REMAP_RE1                  ((uint32_t)0x00000000)        /*!< CANRX mapped to PA11, CANTX mapped to PA12 */
#define AFIO_MAP_CAN_REMAP_RE2                  ((uint32_t)0x00004000)        /*!< CANRX mapped to PB8, CANTX mapped to PB9 */
#define AFIO_MAP_CAN_REMAP_RE3                  ((uint32_t)0x00006000)        /*!< CANRX mapped to PD0, CANTX mapped to PD1 */

#define AFIO_MAP_PTD01_REMAP                    ((uint32_t)0x00008000)        /*!< Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_MAP_TMR5CH4_INTLRE                 ((uint32_t)0x00010000)        /*!< TMR5 Channel4 Internal Remap */
#define AFIO_MAP_ADC1_EXTRGINJ_REMAP            ((uint32_t)0x00020000)        /*!< ADC 1 External Trigger Injected Conversion remapping */
#define AFIO_MAP_ADC1_EXTRGREG_REMAP            ((uint32_t)0x00040000)        /*!< ADC 1 External Trigger Regular Conversion remapping */
#define AFIO_MAPR_ADC1_ETRGREG_REMAP 			AFIO_MAP_ADC1_EXTRGREG_REMAP
#define AFIO_MAP_ADC2_EXTRGINJ_REMAP            ((uint32_t)0x00080000)        /*!< ADC 2 External Trigger Injected Conversion remapping */
#define AFIO_MAP_ADC2_EXTRGREG_REMAP            ((uint32_t)0x00100000)        /*!< ADC 2 External Trigger Regular Conversion remapping */

/** SWJ_CFG configuration */
#define AFIO_MAP_SWJTAG_CONF                    ((uint32_t)0x07000000)        /*!< SWJ_CFG[2:0] bits (Serial Wire JTAG configuration) */
#define AFIO_MAP_SWJTAG_CONF_B0                 ((uint32_t)0x01000000)        /*!< Bit 0 */
#define AFIO_MAP_SWJTAG_CONF_B1                 ((uint32_t)0x02000000)        /*!< Bit 1 */
#define AFIO_MAP_SWJTAG_CONF_B2                 ((uint32_t)0x04000000)        /*!< Bit 2 */

#define AFIO_MAP_SWJTAG_CONF_FULL               ((uint32_t)0x00000000)        /*!< Full SWJ (JTAG-DP + SW-DP) : Reset State */
#define AFIO_MAP_SWJTAG_CONF_NOJNTRST           ((uint32_t)0x01000000)        /*!< Full SWJ (JTAG-DP + SW-DP) but without JNTRST */
#define AFIO_MAP_SWJTAG_CONF_JTAGDISABLE        ((uint32_t)0x02000000)        /*!< JTAG-DP Disabled and SW-DP Enabled */
#define AFIO_MAP_SWJTAG_CONF_ALLDISABLE         ((uint32_t)0x04000000)        /*!< JTAG-DP Disabled and SW-DP Disabled */

/*****************  Bit definition for AFIO_EXTIC1 register  *****************/
#define AFIO_EXTIC1_EXTINT0                     ((uint16_t)0x000F)            /*!< EXTI 0 configuration */
#define AFIO_EXTIC1_EXTINT1                     ((uint16_t)0x00F0)            /*!< EXTI 1 configuration */
#define AFIO_EXTIC1_EXTINT2                     ((uint16_t)0x0F00)            /*!< EXTI 2 configuration */
#define AFIO_EXTIC1_EXTINT3                     ((uint16_t)0xF000)            /*!< EXTI 3 configuration */

/** EXTI0 configuration */
#define AFIO_EXTIC1_EXTINT0_PTA                 ((uint16_t)0x0000)            /*!< PA[0] pin */
#define AFIO_EXTIC1_EXTINT0_PTB                 ((uint16_t)0x0001)            /*!< PB[0] pin */
#define AFIO_EXTIC1_EXTINT0_PTC                 ((uint16_t)0x0002)            /*!< PC[0] pin */
#define AFIO_EXTIC1_EXTINT0_PTD                 ((uint16_t)0x0003)            /*!< PD[0] pin */
#define AFIO_EXTIC1_EXTINT0_PTE                 ((uint16_t)0x0004)            /*!< PE[0] pin */
#define AFIO_EXTIC1_EXTINT0_PTF                 ((uint16_t)0x0005)            /*!< PF[0] pin */
#define AFIO_EXTIC1_EXTINT0_PTG                 ((uint16_t)0x0006)            /*!< PG[0] pin */

/** EXTI1 configuration */
#define AFIO_EXTIC1_EXTINT1_PTA                 ((uint16_t)0x0000)            /*!< PA[1] pin */
#define AFIO_EXTIC1_EXTINT1_PTB                 ((uint16_t)0x0010)            /*!< PB[1] pin */
#define AFIO_EXTIC1_EXTINT1_PTC                 ((uint16_t)0x0020)            /*!< PC[1] pin */
#define AFIO_EXTIC1_EXTINT1_PTD                 ((uint16_t)0x0030)            /*!< PD[1] pin */
#define AFIO_EXTIC1_EXTINT1_PTE                 ((uint16_t)0x0040)            /*!< PE[1] pin */
#define AFIO_EXTIC1_EXTINT1_PTF                 ((uint16_t)0x0050)            /*!< PF[1] pin */
#define AFIO_EXTIC1_EXTINT1_PTG                 ((uint16_t)0x0060)            /*!< PG[1] pin */

/** EXTI2 configuration */
#define AFIO_EXTIC1_EXTINT2_PTA                 ((uint16_t)0x0000)            /*!< PA[2] pin */
#define AFIO_EXTIC1_EXTINT2_PTB                 ((uint16_t)0x0100)            /*!< PB[2] pin */
#define AFIO_EXTIC1_EXTINT2_PTC                 ((uint16_t)0x0200)            /*!< PC[2] pin */
#define AFIO_EXTIC1_EXTINT2_PTD                 ((uint16_t)0x0300)            /*!< PD[2] pin */
#define AFIO_EXTIC1_EXTINT2_PTE                 ((uint16_t)0x0400)            /*!< PE[2] pin */
#define AFIO_EXTIC1_EXTINT2_PTF                 ((uint16_t)0x0500)            /*!< PF[2] pin */
#define AFIO_EXTIC1_EXTINT2_PTG                 ((uint16_t)0x0600)            /*!< PG[2] pin */

/** EXTI3 configuration */
#define AFIO_EXTIC1_EXTINT3_PTA                 ((uint16_t)0x0000)            /*!< PA[3] pin */
#define AFIO_EXTIC1_EXTINT3_PTB                 ((uint16_t)0x1000)            /*!< PB[3] pin */
#define AFIO_EXTIC1_EXTINT3_PTC                 ((uint16_t)0x2000)            /*!< PC[3] pin */
#define AFIO_EXTIC1_EXTINT3_PTD                 ((uint16_t)0x3000)            /*!< PD[3] pin */
#define AFIO_EXTIC1_EXTINT3_PTE                 ((uint16_t)0x4000)            /*!< PE[3] pin */
#define AFIO_EXTIC1_EXTINT3_PTF                 ((uint16_t)0x5000)            /*!< PF[3] pin */
#define AFIO_EXTIC1_EXTINT3_PTG                 ((uint16_t)0x6000)            /*!< PG[3] pin */

/*****************  Bit definition for AFIO_EXTIC2 register  *****************/
#define AFIO_EXTIC2_EXTINT4                     ((uint16_t)0x000F)            /*!< EXTI 4 configuration */
#define AFIO_EXTIC2_EXTINT5                     ((uint16_t)0x00F0)            /*!< EXTI 5 configuration */
#define AFIO_EXTIC2_EXTINT6                     ((uint16_t)0x0F00)            /*!< EXTI 6 configuration */
#define AFIO_EXTIC2_EXTINT7                     ((uint16_t)0xF000)            /*!< EXTI 7 configuration */

/** EXTI4 configuration */
#define AFIO_EXTIC2_EXTINT4_PTA                 ((uint16_t)0x0000)            /*!< PA[4] pin */
#define AFIO_EXTIC2_EXTINT4_PTB                 ((uint16_t)0x0001)            /*!< PB[4] pin */
#define AFIO_EXTIC2_EXTINT4_PTC                 ((uint16_t)0x0002)            /*!< PC[4] pin */
#define AFIO_EXTIC2_EXTINT4_PTD                 ((uint16_t)0x0003)            /*!< PD[4] pin */
#define AFIO_EXTIC2_EXTINT4_PTE                 ((uint16_t)0x0004)            /*!< PE[4] pin */
#define AFIO_EXTIC2_EXTINT4_PTF                 ((uint16_t)0x0005)            /*!< PF[4] pin */
#define AFIO_EXTIC2_EXTINT4_PTG                 ((uint16_t)0x0006)            /*!< PG[4] pin */

/* EXTI5 configuration */
#define AFIO_EXTIC2_EXTINT5_PTA                 ((uint16_t)0x0000)            /*!< PA[5] pin */
#define AFIO_EXTIC2_EXTINT5_PTB                 ((uint16_t)0x0010)            /*!< PB[5] pin */
#define AFIO_EXTIC2_EXTINT5_PTC                 ((uint16_t)0x0020)            /*!< PC[5] pin */
#define AFIO_EXTIC2_EXTINT5_PTD                 ((uint16_t)0x0030)            /*!< PD[5] pin */
#define AFIO_EXTIC2_EXTINT5_PTE                 ((uint16_t)0x0040)            /*!< PE[5] pin */
#define AFIO_EXTIC2_EXTINT5_PTF                 ((uint16_t)0x0050)            /*!< PF[5] pin */
#define AFIO_EXTIC2_EXTINT5_PTG                 ((uint16_t)0x0060)            /*!< PG[5] pin */

/** EXTI6 configuration */
#define AFIO_EXTIC2_EXTINT6_PTA                 ((uint16_t)0x0000)            /*!< PA[6] pin */
#define AFIO_EXTIC2_EXTINT6_PTB                 ((uint16_t)0x0100)            /*!< PB[6] pin */
#define AFIO_EXTIC2_EXTINT6_PTC                 ((uint16_t)0x0200)            /*!< PC[6] pin */
#define AFIO_EXTIC2_EXTINT6_PTD                 ((uint16_t)0x0300)            /*!< PD[6] pin */
#define AFIO_EXTIC2_EXTINT6_PTE                 ((uint16_t)0x0400)            /*!< PE[6] pin */
#define AFIO_EXTIC2_EXTINT6_PTF                 ((uint16_t)0x0500)            /*!< PF[6] pin */
#define AFIO_EXTIC2_EXTINT6_PTG                 ((uint16_t)0x0600)            /*!< PG[6] pin */

/** EXTI7 configuration */
#define AFIO_EXTIC2_EXTINT7_PTA                 ((uint16_t)0x0000)            /*!< PA[7] pin */
#define AFIO_EXTIC2_EXTINT7_PTB                 ((uint16_t)0x1000)            /*!< PB[7] pin */
#define AFIO_EXTIC2_EXTINT7_PTC                 ((uint16_t)0x2000)            /*!< PC[7] pin */
#define AFIO_EXTIC2_EXTINT7_PTD                 ((uint16_t)0x3000)            /*!< PD[7] pin */
#define AFIO_EXTIC2_EXTINT7_PTE                 ((uint16_t)0x4000)            /*!< PE[7] pin */
#define AFIO_EXTIC2_EXTINT7_PTF                 ((uint16_t)0x5000)            /*!< PF[7] pin */
#define AFIO_EXTIC2_EXTINT7_PTG                 ((uint16_t)0x6000)            /*!< PG[7] pin */

/*****************  Bit definition for AFIO_EXTIC3 register  *****************/
#define AFIO_EXTIC3_EXTINT8                     ((uint16_t)0x000F)            /*!< EXTI 8 configuration */
#define AFIO_EXTIC3_EXTINT9                     ((uint16_t)0x00F0)            /*!< EXTI 9 configuration */
#define AFIO_EXTIC3_EXTINT10                    ((uint16_t)0x0F00)            /*!< EXTI 10 configuration */
#define AFIO_EXTIC3_EXTINT11                    ((uint16_t)0xF000)            /*!< EXTI 11 configuration */

/** EXTI8 configuration */
#define AFIO_EXTIC3_EXTINT8_PTA                 ((uint16_t)0x0000)            /*!< PA[8] pin */
#define AFIO_EXTIC3_EXTINT8_PTB                 ((uint16_t)0x0001)            /*!< PB[8] pin */
#define AFIO_EXTIC3_EXTINT8_PTC                 ((uint16_t)0x0002)            /*!< PC[8] pin */
#define AFIO_EXTIC3_EXTINT8_PTD                 ((uint16_t)0x0003)            /*!< PD[8] pin */
#define AFIO_EXTIC3_EXTINT8_PTE                 ((uint16_t)0x0004)            /*!< PE[8] pin */
#define AFIO_EXTIC3_EXTINT8_PTF                 ((uint16_t)0x0005)            /*!< PF[8] pin */
#define AFIO_EXTIC3_EXTINT8_PTG                 ((uint16_t)0x0006)            /*!< PG[8] pin */

/** EXTI9 configuration */
#define AFIO_EXTIC3_EXTINT9_PTA                 ((uint16_t)0x0000)            /*!< PA[9] pin */
#define AFIO_EXTIC3_EXTINT9_PTB                 ((uint16_t)0x0010)            /*!< PB[9] pin */
#define AFIO_EXTIC3_EXTINT9_PTC                 ((uint16_t)0x0020)            /*!< PC[9] pin */
#define AFIO_EXTIC3_EXTINT9_PTD                 ((uint16_t)0x0030)            /*!< PD[9] pin */
#define AFIO_EXTIC3_EXTINT9_PTE                 ((uint16_t)0x0040)            /*!< PE[9] pin */
#define AFIO_EXTIC3_EXTINT9_PTF                 ((uint16_t)0x0050)            /*!< PF[9] pin */
#define AFIO_EXTIC3_EXTINT9_PTG                 ((uint16_t)0x0060)            /*!< PG[9] pin */

/** EXTI10 configuration */
#define AFIO_EXTIC3_EXTINT10_PTA                ((uint16_t)0x0000)            /*!< PA[10] pin */
#define AFIO_EXTIC3_EXTINT10_PTB                ((uint16_t)0x0100)            /*!< PB[10] pin */
#define AFIO_EXTIC3_EXTINT10_PTC                ((uint16_t)0x0200)            /*!< PC[10] pin */
#define AFIO_EXTIC3_EXTINT10_PTD                ((uint16_t)0x0300)            /*!< PD[10] pin */
#define AFIO_EXTIC3_EXTINT10_PTE                ((uint16_t)0x0400)            /*!< PE[10] pin */
#define AFIO_EXTIC3_EXTINT10_PTF                ((uint16_t)0x0500)            /*!< PF[10] pin */
#define AFIO_EXTIC3_EXTINT10_PTG                ((uint16_t)0x0600)            /*!< PG[10] pin */

/** EXTI11 configuration */
#define AFIO_EXTIC3_EXTINT11_PTA                ((uint16_t)0x0000)            /*!< PA[11] pin */
#define AFIO_EXTIC3_EXTINT11_PTB                ((uint16_t)0x1000)            /*!< PB[11] pin */
#define AFIO_EXTIC3_EXTINT11_PTC                ((uint16_t)0x2000)            /*!< PC[11] pin */
#define AFIO_EXTIC3_EXTINT11_PTD                ((uint16_t)0x3000)            /*!< PD[11] pin */
#define AFIO_EXTIC3_EXTINT11_PTE                ((uint16_t)0x4000)            /*!< PE[11] pin */
#define AFIO_EXTIC3_EXTINT11_PTF                ((uint16_t)0x5000)            /*!< PF[11] pin */
#define AFIO_EXTIC3_EXTINT11_PTG                ((uint16_t)0x6000)            /*!< PG[11] pin */

/*****************  Bit definition for AFIO_EXTIC4 register  *****************/
#define AFIO_EXTIC4_EXTINT12                    ((uint16_t)0x000F)            /*!< EXTI 12 configuration */
#define AFIO_EXTIC4_EXTINT13                    ((uint16_t)0x00F0)            /*!< EXTI 13 configuration */
#define AFIO_EXTIC4_EXTINT14                    ((uint16_t)0x0F00)            /*!< EXTI 14 configuration */
#define AFIO_EXTIC4_EXTINT15                    ((uint16_t)0xF000)            /*!< EXTI 15 configuration */

/** EXTI12 configuration */
#define AFIO_EXTIC4_EXTINT12_PTA                ((uint16_t)0x0000)            /*!< PA[12] pin */
#define AFIO_EXTIC4_EXTINT12_PTB                ((uint16_t)0x0001)            /*!< PB[12] pin */
#define AFIO_EXTIC4_EXTINT12_PTC                ((uint16_t)0x0002)            /*!< PC[12] pin */
#define AFIO_EXTIC4_EXTINT12_PTD                ((uint16_t)0x0003)            /*!< PD[12] pin */
#define AFIO_EXTIC4_EXTINT12_PTE                ((uint16_t)0x0004)            /*!< PE[12] pin */
#define AFIO_EXTIC4_EXTINT12_PTF                ((uint16_t)0x0005)            /*!< PF[12] pin */
#define AFIO_EXTIC4_EXTINT12_PTG                ((uint16_t)0x0006)            /*!< PG[12] pin */

/** EXTI13 configuration */
#define AFIO_EXTIC4_EXTINT13_PTA                ((uint16_t)0x0000)            /*!< PA[13] pin */
#define AFIO_EXTIC4_EXTINT13_PTB                ((uint16_t)0x0010)            /*!< PB[13] pin */
#define AFIO_EXTIC4_EXTINT13_PTC                ((uint16_t)0x0020)            /*!< PC[13] pin */
#define AFIO_EXTIC4_EXTINT13_PTD                ((uint16_t)0x0030)            /*!< PD[13] pin */
#define AFIO_EXTIC4_EXTINT13_PTE                ((uint16_t)0x0040)            /*!< PE[13] pin */
#define AFIO_EXTIC4_EXTINT13_PTF                ((uint16_t)0x0050)            /*!< PF[13] pin */
#define AFIO_EXTIC4_EXTINT13_PTG                ((uint16_t)0x0060)            /*!< PG[13] pin */

/** EXTI14 configuration */
#define AFIO_EXTIC4_EXTINT14_PTA                ((uint16_t)0x0000)            /*!< PA[14] pin */
#define AFIO_EXTIC4_EXTINT14_PTB                ((uint16_t)0x0100)            /*!< PB[14] pin */
#define AFIO_EXTIC4_EXTINT14_PTC                ((uint16_t)0x0200)            /*!< PC[14] pin */
#define AFIO_EXTIC4_EXTINT14_PTD                ((uint16_t)0x0300)            /*!< PD[14] pin */
#define AFIO_EXTIC4_EXTINT14_PTE                ((uint16_t)0x0400)            /*!< PE[14] pin */
#define AFIO_EXTIC4_EXTINT14_PTF                ((uint16_t)0x0500)            /*!< PF[14] pin */
#define AFIO_EXTIC4_EXTINT14_PTG                ((uint16_t)0x0600)            /*!< PG[14] pin */

/** EXTI15 configuration */
#define AFIO_EXTIC4_EXTINT15_PTA                ((uint16_t)0x0000)            /*!< PA[15] pin */
#define AFIO_EXTIC4_EXTINT15_PTB                ((uint16_t)0x1000)            /*!< PB[15] pin */
#define AFIO_EXTIC4_EXTINT15_PTC                ((uint16_t)0x2000)            /*!< PC[15] pin */
#define AFIO_EXTIC4_EXTINT15_PTD                ((uint16_t)0x3000)            /*!< PD[15] pin */
#define AFIO_EXTIC4_EXTINT15_PTE                ((uint16_t)0x4000)            /*!< PE[15] pin */
#define AFIO_EXTIC4_EXTINT15_PTF                ((uint16_t)0x5000)            /*!< PF[15] pin */
#define AFIO_EXTIC4_EXTINT15_PTG                ((uint16_t)0x6000)            /*!< PG[15] pin */


/******************  Bit definition for AFIO_MAP2 register  ******************/
#define AFIO_MAP2_TMR9_REMAP                    ((uint32_t)0x00000020)        /*!< TMR9 remapping */
#define AFIO_MAP2_TMR10_REMAP                   ((uint32_t)0x00000040)        /*!< TMR10 remapping */
#define AFIO_MAP2_TMR11_REMAP                   ((uint32_t)0x00000080)        /*!< TMR11 remapping */
#define AFIO_MAP2_TMR13_REMAP                   ((uint32_t)0x00000100)        /*!< TMR13 remapping */
#define AFIO_MAP2_TMR14_REMAP                   ((uint32_t)0x00000200)        /*!< TMR14 remapping */
#define AFIO_MAP2_XMC_NADV_REMAP                ((uint32_t)0x00000400)        /*!< XMC NADV remapping */
#define AFIO_MAP2_TMR15_REMAP                   ((uint32_t)0x00000001)        /*!< TMR15 remapping */
#define AFIO_MAP2_SPI4_REMAP                    ((uint32_t)0x00020000)        /*!< SPI4 remapping */
#define AFIO_MAP2_I2C3_REMAP                    ((uint32_t)0x00040000)        /*!< I2C3 remapping */
#define AFIO_MAP2_SDIO2_REMAP_01                ((uint32_t)0x00080000)        /*!< SDIO2 remapping01 */
#define AFIO_MAP2_SDIO2_REMAP_10                ((uint32_t)0x00100000)        /*!< SDIO2 remapping10 */
#define AFIO_MAP2_SDIO2_REMAP_11                ((uint32_t)0x00180000)        /*!< SDIO2 remapping11 */
#define AFIO_MAP2_EXT_FLASH_REMAP               ((uint32_t)0x00200000)        /*!< External FLASH remapping */


/******************************************************************************/
/*                                                                            */
/*                               SystemTick                                   */
/*                                                                            */
/******************************************************************************/

/*****************  Bit definition for SysTick_CTRL register  *****************/
#define  SysTick_CTRL_ENABLE                    ((uint32_t)0x00000001)        /*!< Counter enable */
#define  SysTick_CTRL_TICKINT                   ((uint32_t)0x00000002)        /*!< Counting down to 0 pends the SysTick handler */
#define  SysTick_CTRL_CLKSOURCE                 ((uint32_t)0x00000004)        /*!< Clock source */
#define  SysTick_CTRL_COUNTFLAG                 ((uint32_t)0x00010000)        /*!< Count Flag */

/*****************  Bit definition for SysTick_LOAD register  *****************/
#define  SysTick_LOAD_RELOAD                    ((uint32_t)0x00FFFFFF)        /*!< Value to load into the SysTick Current Value Register when the counter reaches 0 */

/*****************  Bit definition for SysTick_VAL register  ******************/
#define  SysTick_VAL_CURRENT                    ((uint32_t)0x00FFFFFF)        /*!< Current value at the time the register is accessed */

/*****************  Bit definition for SysTick_CALIB register  ****************/
#define  SysTick_CALIB_TENMS                    ((uint32_t)0x00FFFFFF)        /*!< Reload value to use for 10ms timing */
#define  SysTick_CALIB_SKEW                     ((uint32_t)0x40000000)        /*!< Calibration value is not exactly 10 ms */
#define  SysTick_CALIB_NOREF                    ((uint32_t)0x80000000)        /*!< The reference clock is not provided */

/******************************************************************************/
/*                                                                            */
/*                  Nested Vectored Interrupt Controller                      */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for NVIC_ISER register  *******************/
#define  NVIC_ISER_SETENA                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt set enable bits */
#define  NVIC_ISER_SETENA_0                 ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ISER_SETENA_1                 ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ISER_SETENA_2                 ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ISER_SETENA_3                 ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ISER_SETENA_4                 ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ISER_SETENA_5                 ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ISER_SETENA_6                 ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ISER_SETENA_7                 ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ISER_SETENA_8                 ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ISER_SETENA_9                 ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ISER_SETENA_10                ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ISER_SETENA_11                ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ISER_SETENA_12                ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ISER_SETENA_13                ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ISER_SETENA_14                ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ISER_SETENA_15                ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ISER_SETENA_16                ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ISER_SETENA_17                ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ISER_SETENA_18                ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ISER_SETENA_19                ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ISER_SETENA_20                ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ISER_SETENA_21                ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ISER_SETENA_22                ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ISER_SETENA_23                ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ISER_SETENA_24                ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ISER_SETENA_25                ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ISER_SETENA_26                ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ISER_SETENA_27                ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ISER_SETENA_28                ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ISER_SETENA_29                ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ISER_SETENA_30                ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ISER_SETENA_31                ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ICER register  *******************/
#define  NVIC_ICER_CLRENA                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt clear-enable bits */
#define  NVIC_ICER_CLRENA_0                 ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ICER_CLRENA_1                 ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ICER_CLRENA_2                 ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ICER_CLRENA_3                 ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ICER_CLRENA_4                 ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ICER_CLRENA_5                 ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ICER_CLRENA_6                 ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ICER_CLRENA_7                 ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ICER_CLRENA_8                 ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ICER_CLRENA_9                 ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ICER_CLRENA_10                ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ICER_CLRENA_11                ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ICER_CLRENA_12                ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ICER_CLRENA_13                ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ICER_CLRENA_14                ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ICER_CLRENA_15                ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ICER_CLRENA_16                ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ICER_CLRENA_17                ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ICER_CLRENA_18                ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ICER_CLRENA_19                ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ICER_CLRENA_20                ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ICER_CLRENA_21                ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ICER_CLRENA_22                ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ICER_CLRENA_23                ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ICER_CLRENA_24                ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ICER_CLRENA_25                ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ICER_CLRENA_26                ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ICER_CLRENA_27                ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ICER_CLRENA_28                ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ICER_CLRENA_29                ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ICER_CLRENA_30                ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ICER_CLRENA_31                ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ISPR register  *******************/
#define  NVIC_ISPR_SETPEND                  ((uint32_t)0xFFFFFFFF)        /*!< Interrupt set-pending bits */
#define  NVIC_ISPR_SETPEND_0                ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ISPR_SETPEND_1                ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ISPR_SETPEND_2                ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ISPR_SETPEND_3                ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ISPR_SETPEND_4                ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ISPR_SETPEND_5                ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ISPR_SETPEND_6                ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ISPR_SETPEND_7                ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ISPR_SETPEND_8                ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ISPR_SETPEND_9                ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ISPR_SETPEND_10               ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ISPR_SETPEND_11               ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ISPR_SETPEND_12               ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ISPR_SETPEND_13               ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ISPR_SETPEND_14               ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ISPR_SETPEND_15               ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ISPR_SETPEND_16               ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ISPR_SETPEND_17               ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ISPR_SETPEND_18               ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ISPR_SETPEND_19               ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ISPR_SETPEND_20               ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ISPR_SETPEND_21               ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ISPR_SETPEND_22               ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ISPR_SETPEND_23               ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ISPR_SETPEND_24               ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ISPR_SETPEND_25               ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ISPR_SETPEND_26               ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ISPR_SETPEND_27               ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ISPR_SETPEND_28               ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ISPR_SETPEND_29               ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ISPR_SETPEND_30               ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ISPR_SETPEND_31               ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ICPR register  *******************/
#define  NVIC_ICPR_CLRPEND                  ((uint32_t)0xFFFFFFFF)        /*!< Interrupt clear-pending bits */
#define  NVIC_ICPR_CLRPEND_0                ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ICPR_CLRPEND_1                ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ICPR_CLRPEND_2                ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ICPR_CLRPEND_3                ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ICPR_CLRPEND_4                ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ICPR_CLRPEND_5                ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ICPR_CLRPEND_6                ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ICPR_CLRPEND_7                ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ICPR_CLRPEND_8                ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ICPR_CLRPEND_9                ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ICPR_CLRPEND_10               ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ICPR_CLRPEND_11               ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ICPR_CLRPEND_12               ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ICPR_CLRPEND_13               ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ICPR_CLRPEND_14               ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ICPR_CLRPEND_15               ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ICPR_CLRPEND_16               ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ICPR_CLRPEND_17               ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ICPR_CLRPEND_18               ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ICPR_CLRPEND_19               ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ICPR_CLRPEND_20               ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ICPR_CLRPEND_21               ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ICPR_CLRPEND_22               ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ICPR_CLRPEND_23               ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ICPR_CLRPEND_24               ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ICPR_CLRPEND_25               ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ICPR_CLRPEND_26               ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ICPR_CLRPEND_27               ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ICPR_CLRPEND_28               ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ICPR_CLRPEND_29               ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ICPR_CLRPEND_30               ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ICPR_CLRPEND_31               ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_IABR register  *******************/
#define  NVIC_IABR_ACTIVE                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt active flags */
#define  NVIC_IABR_ACTIVE_0                 ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_IABR_ACTIVE_1                 ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_IABR_ACTIVE_2                 ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_IABR_ACTIVE_3                 ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_IABR_ACTIVE_4                 ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_IABR_ACTIVE_5                 ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_IABR_ACTIVE_6                 ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_IABR_ACTIVE_7                 ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_IABR_ACTIVE_8                 ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_IABR_ACTIVE_9                 ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_IABR_ACTIVE_10                ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_IABR_ACTIVE_11                ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_IABR_ACTIVE_12                ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_IABR_ACTIVE_13                ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_IABR_ACTIVE_14                ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_IABR_ACTIVE_15                ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_IABR_ACTIVE_16                ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_IABR_ACTIVE_17                ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_IABR_ACTIVE_18                ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_IABR_ACTIVE_19                ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_IABR_ACTIVE_20                ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_IABR_ACTIVE_21                ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_IABR_ACTIVE_22                ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_IABR_ACTIVE_23                ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_IABR_ACTIVE_24                ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_IABR_ACTIVE_25                ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_IABR_ACTIVE_26                ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_IABR_ACTIVE_27                ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_IABR_ACTIVE_28                ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_IABR_ACTIVE_29                ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_IABR_ACTIVE_30                ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_IABR_ACTIVE_31                ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_PRI0 register  *******************/
#define  NVIC_IPR0_PRI_0                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 0 */
#define  NVIC_IPR0_PRI_1                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 1 */
#define  NVIC_IPR0_PRI_2                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 2 */
#define  NVIC_IPR0_PRI_3                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 3 */

/******************  Bit definition for NVIC_PRI1 register  *******************/
#define  NVIC_IPR1_PRI_4                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 4 */
#define  NVIC_IPR1_PRI_5                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 5 */
#define  NVIC_IPR1_PRI_6                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 6 */
#define  NVIC_IPR1_PRI_7                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 7 */

/******************  Bit definition for NVIC_PRI2 register  *******************/
#define  NVIC_IPR2_PRI_8                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 8 */
#define  NVIC_IPR2_PRI_9                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 9 */
#define  NVIC_IPR2_PRI_10                   ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 10 */
#define  NVIC_IPR2_PRI_11                   ((uint32_t)0xFF000000)        /*!< Priority of interrupt 11 */

/******************  Bit definition for NVIC_PRI3 register  *******************/
#define  NVIC_IPR3_PRI_12                   ((uint32_t)0x000000FF)        /*!< Priority of interrupt 12 */
#define  NVIC_IPR3_PRI_13                   ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 13 */
#define  NVIC_IPR3_PRI_14                   ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 14 */
#define  NVIC_IPR3_PRI_15                   ((uint32_t)0xFF000000)        /*!< Priority of interrupt 15 */

/******************  Bit definition for NVIC_PRI4 register  *******************/
#define  NVIC_IPR4_PRI_16                   ((uint32_t)0x000000FF)        /*!< Priority of interrupt 16 */
#define  NVIC_IPR4_PRI_17                   ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 17 */
#define  NVIC_IPR4_PRI_18                   ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 18 */
#define  NVIC_IPR4_PRI_19                   ((uint32_t)0xFF000000)        /*!< Priority of interrupt 19 */

/******************  Bit definition for NVIC_PRI5 register  *******************/
#define  NVIC_IPR5_PRI_20                   ((uint32_t)0x000000FF)        /*!< Priority of interrupt 20 */
#define  NVIC_IPR5_PRI_21                   ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 21 */
#define  NVIC_IPR5_PRI_22                   ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 22 */
#define  NVIC_IPR5_PRI_23                   ((uint32_t)0xFF000000)        /*!< Priority of interrupt 23 */

/******************  Bit definition for NVIC_PRI6 register  *******************/
#define  NVIC_IPR6_PRI_24                   ((uint32_t)0x000000FF)        /*!< Priority of interrupt 24 */
#define  NVIC_IPR6_PRI_25                   ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 25 */
#define  NVIC_IPR6_PRI_26                   ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 26 */
#define  NVIC_IPR6_PRI_27                   ((uint32_t)0xFF000000)        /*!< Priority of interrupt 27 */

/******************  Bit definition for NVIC_PRI7 register  *******************/
#define  NVIC_IPR7_PRI_28                   ((uint32_t)0x000000FF)        /*!< Priority of interrupt 28 */
#define  NVIC_IPR7_PRI_29                   ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 29 */
#define  NVIC_IPR7_PRI_30                   ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 30 */
#define  NVIC_IPR7_PRI_31                   ((uint32_t)0xFF000000)        /*!< Priority of interrupt 31 */

/******************  Bit definition for SCB_CPUID register  *******************/
#define  SCB_CPUID_REVISION                 ((uint32_t)0x0000000F)        /*!< Implementation defined revision number */
#define  SCB_CPUID_PARTNO                   ((uint32_t)0x0000FFF0)        /*!< Number of processor within family */
#define  SCB_CPUID_Constant                 ((uint32_t)0x000F0000)        /*!< Reads as 0x0F */
#define  SCB_CPUID_VARIANT                  ((uint32_t)0x00F00000)        /*!< Implementation defined variant number */
#define  SCB_CPUID_IMPLEMENTER              ((uint32_t)0xFF000000)        /*!< Implementer code. ARM is 0x41 */

/*******************  Bit definition for SCB_ICSR register  *******************/
#define  SCB_ICSR_VECTACTIVE                ((uint32_t)0x000001FF)        /*!< Active ISR number field */
#define  SCB_ICSR_RETTOBASE                 ((uint32_t)0x00000800)        /*!< All active exceptions minus the IPSR_current_exception yields the empty set */
#define  SCB_ICSR_VECTPENDING               ((uint32_t)0x003FF000)        /*!< Pending ISR number field */
#define  SCB_ICSR_ISRPENDING                ((uint32_t)0x00400000)        /*!< Interrupt pending flag */
#define  SCB_ICSR_ISRPREEMPT                ((uint32_t)0x00800000)        /*!< It indicates that a pending interrupt becomes active in the next running cycle */
#define  SCB_ICSR_PENDSTCLR                 ((uint32_t)0x02000000)        /*!< Clear pending SysTick bit */
#define  SCB_ICSR_PENDSTSET                 ((uint32_t)0x04000000)        /*!< Set pending SysTick bit */
#define  SCB_ICSR_PENDSVCLR                 ((uint32_t)0x08000000)        /*!< Clear pending pendSV bit */
#define  SCB_ICSR_PENDSVSET                 ((uint32_t)0x10000000)        /*!< Set pending pendSV bit */
#define  SCB_ICSR_NMIPENDSET                ((uint32_t)0x80000000)        /*!< Set pending NMI bit */

/*******************  Bit definition for SCB_VTOR register  *******************/
#define  SCB_VTOR_TBLOFF                    ((uint32_t)0x1FFFFF80)        /*!< Vector table base offset field */
#define  SCB_VTOR_TBLBASE                   ((uint32_t)0x20000000)        /*!< Table base in code(0) or RAM(1) */

/*******************  Bit definition for SCB_AIRCR register  *******************/
#define  SCB_AIRCR_VECTRESET                ((uint32_t)0x00000001)        /*!< System Reset bit */
#define  SCB_AIRCR_VECTCLRACTIVE            ((uint32_t)0x00000002)        /*!< Clear active vector bit */
#define  SCB_AIRCR_SYSRESETREQ              ((uint32_t)0x00000004)        /*!< Requests chip control logic to generate a reset */

#define  SCB_AIRCR_PRIGROUP                 ((uint32_t)0x00000700)        /*!< PRIGROUP[2:0] bits (Priority group) */
#define  SCB_AIRCR_PRIGROUP_0               ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  SCB_AIRCR_PRIGROUP_1               ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  SCB_AIRCR_PRIGROUP_2               ((uint32_t)0x00000400)        /*!< Bit 2  */

/* prority group configuration */
#define  SCB_AIRCR_PRIGROUP0                ((uint32_t)0x00000000)        /*!< Priority group=0 (7 bits of pre-emption priority, 1 bit of subpriority) */
#define  SCB_AIRCR_PRIGROUP1                ((uint32_t)0x00000100)        /*!< Priority group=1 (6 bits of pre-emption priority, 2 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP2                ((uint32_t)0x00000200)        /*!< Priority group=2 (5 bits of pre-emption priority, 3 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP3                ((uint32_t)0x00000300)        /*!< Priority group=3 (4 bits of pre-emption priority, 4 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP4                ((uint32_t)0x00000400)        /*!< Priority group=4 (3 bits of pre-emption priority, 5 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP5                ((uint32_t)0x00000500)        /*!< Priority group=5 (2 bits of pre-emption priority, 6 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP6                ((uint32_t)0x00000600)        /*!< Priority group=6 (1 bit of pre-emption priority, 7 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP7                ((uint32_t)0x00000700)        /*!< Priority group=7 (no pre-emption priority, 8 bits of subpriority) */

#define  SCB_AIRCR_ENDIANESS                ((uint32_t)0x00008000)        /*!< Data endianness bit */
#define  SCB_AIRCR_VECTKEY                  ((uint32_t)0xFFFF0000)        /*!< Register key (VECTKEY) - Reads as 0xFA05 (VECTKEYSTAT) */

/*******************  Bit definition for SCB_SCR register  ********************/
#define  SCB_SCR_SLEEPONEXIT                ((uint8_t)0x02)               /*!< Sleep on exit bit */
#define  SCB_SCR_SLEEPDEEP                  ((uint8_t)0x04)               /*!< Sleep deep bit */
#define  SCB_SCR_SEVONPEND                  ((uint8_t)0x10)               /*!< Wake up from WFE */

/********************  Bit definition for SCB_CCR register  *******************/
#define  SCB_CCR_NONBASETHRDENA             ((uint16_t)0x0001)            /*!< Thread mode can be entered from any level in Handler mode by controlled return value */
#define  SCB_CCR_USERSETMPEND               ((uint16_t)0x0002)            /*!< Enables user code to write the Software Trigger Interrupt register to trigger (pend) a Main exception */
#define  SCB_CCR_UNALIGN_TRP                ((uint16_t)0x0008)            /*!< Trap for unaligned access */
#define  SCB_CCR_DIV_0_TRP                  ((uint16_t)0x0010)            /*!< Trap on Divide by 0 */
#define  SCB_CCR_BFHFNMIGN                  ((uint16_t)0x0100)            /*!< Handlers running at priority -1 and -2 */
#define  SCB_CCR_STKALIGN                   ((uint16_t)0x0200)            /*!< On exception entry, the SP used prior to the exception is adjusted to be 8-byte aligned */

/*******************  Bit definition for SCB_SHPR register ********************/
#define  SCB_SHPR_PRI_N                     ((uint32_t)0x000000FF)        /*!< Priority of system handler 4,8, and 12. Mem Manage, reserved and Debug Monitor */
#define  SCB_SHPR_PRI_N1                    ((uint32_t)0x0000FF00)        /*!< Priority of system handler 5,9, and 13. Bus Fault, reserved and reserved */
#define  SCB_SHPR_PRI_N2                    ((uint32_t)0x00FF0000)        /*!< Priority of system handler 6,10, and 14. Usage Fault, reserved and PendSV */
#define  SCB_SHPR_PRI_N3                    ((uint32_t)0xFF000000)        /*!< Priority of system handler 7,11, and 15. Reserved, SVCall and SysTick */

/******************  Bit definition for SCB_SHCSR register  *******************/
#define  SCB_SHCSR_MEMFAULTACT              ((uint32_t)0x00000001)        /*!< MemManage is active */
#define  SCB_SHCSR_BUSFAULTACT              ((uint32_t)0x00000002)        /*!< BusFault is active */
#define  SCB_SHCSR_USGFAULTACT              ((uint32_t)0x00000008)        /*!< UsageFault is active */
#define  SCB_SHCSR_SVCALLACT                ((uint32_t)0x00000080)        /*!< SVCall is active */
#define  SCB_SHCSR_MONITORACT               ((uint32_t)0x00000100)        /*!< Monitor is active */
#define  SCB_SHCSR_PENDSVACT                ((uint32_t)0x00000400)        /*!< PendSV is active */
#define  SCB_SHCSR_SYSTICKACT               ((uint32_t)0x00000800)        /*!< SysTick is active */
#define  SCB_SHCSR_USGFAULTPENDED           ((uint32_t)0x00001000)        /*!< Usage Fault is pended */
#define  SCB_SHCSR_MEMFAULTPENDED           ((uint32_t)0x00002000)        /*!< MemManage is pended */
#define  SCB_SHCSR_BUSFAULTPENDED           ((uint32_t)0x00004000)        /*!< Bus Fault is pended */
#define  SCB_SHCSR_SVCALLPENDED             ((uint32_t)0x00008000)        /*!< SVCall is pended */
#define  SCB_SHCSR_MEMFAULTENA              ((uint32_t)0x00010000)        /*!< MemManage enable */
#define  SCB_SHCSR_BUSFAULTENA              ((uint32_t)0x00020000)        /*!< Bus Fault enable */
#define  SCB_SHCSR_USGFAULTENA              ((uint32_t)0x00040000)        /*!< UsageFault enable */

/*******************  Bit definition for SCB_CFSR register  *******************/
/** MFSR */
#define  SCB_CFSR_IACCVIOL                  ((uint32_t)0x00000001)        /*!< Instruction access violation */
#define  SCB_CFSR_DACCVIOL                  ((uint32_t)0x00000002)        /*!< Data access violation */
#define  SCB_CFSR_MUNSTKERR                 ((uint32_t)0x00000008)        /*!< Unstacking error */
#define  SCB_CFSR_MSTKERR                   ((uint32_t)0x00000010)        /*!< Stacking error */
#define  SCB_CFSR_MMARVALID                 ((uint32_t)0x00000080)        /*!< Memory Manage Address Register address valid flag */
/** BFSR */
#define  SCB_CFSR_IBUSERR                   ((uint32_t)0x00000100)        /*!< Instruction bus error flag */
#define  SCB_CFSR_PRECISERR                 ((uint32_t)0x00000200)        /*!< Precise data bus error */
#define  SCB_CFSR_IMPRECISERR               ((uint32_t)0x00000400)        /*!< Imprecise data bus error */
#define  SCB_CFSR_UNSTKERR                  ((uint32_t)0x00000800)        /*!< Unstacking error */
#define  SCB_CFSR_STKERR                    ((uint32_t)0x00001000)        /*!< Stacking error */
#define  SCB_CFSR_BFARVALID                 ((uint32_t)0x00008000)        /*!< Bus Fault Address Register address valid flag */
/** UFSR */
#define  SCB_CFSR_UNDEFINSTR                ((uint32_t)0x00010000)        /*!< The processor attempt to execute an undefined instruction */
#define  SCB_CFSR_INVSTATE                  ((uint32_t)0x00020000)        /*!< Invalid combination of EPSR and instruction */
#define  SCB_CFSR_INVPC                     ((uint32_t)0x00040000)        /*!< Attempt to load EXC_RETURN into pc illegally */
#define  SCB_CFSR_NOCP                      ((uint32_t)0x00080000)        /*!< Attempt to use a coprocessor instruction */
#define  SCB_CFSR_UNALIGNED                 ((uint32_t)0x01000000)        /*!< Fault occurs when there is an attempt to make an unaligned memory access */
#define  SCB_CFSR_DIVBYZERO                 ((uint32_t)0x02000000)        /*!< Fault occurs when SDIV or DIV instruction is used with a divisor of 0 */

/*******************  Bit definition for SCB_HFSR register  *******************/
#define  SCB_HFSR_VECTTBL                   ((uint32_t)0x00000002)        /*!< Fault occurs because of vector table read on exception processing */
#define  SCB_HFSR_FORCED                    ((uint32_t)0x40000000)        /*!< Hard Fault activated when a configurable Fault was received and cannot activate */
#define  SCB_HFSR_DEBUGEVT                  ((uint32_t)0x80000000)        /*!< Fault related to debug */

/*******************  Bit definition for SCB_DFSR register  *******************/
#define  SCB_DFSR_HALTED                    ((uint8_t)0x01)               /*!< Halt request flag */
#define  SCB_DFSR_BKPT                      ((uint8_t)0x02)               /*!< BKPT flag */
#define  SCB_DFSR_DWTTRAP                   ((uint8_t)0x04)               /*!< Data Watchpoint and Trace (DWT) flag */
#define  SCB_DFSR_VCATCH                    ((uint8_t)0x08)               /*!< Vector catch flag */
#define  SCB_DFSR_EXTERNAL                  ((uint8_t)0x10)               /*!< External debug request flag */

/*******************  Bit definition for SCB_MMFAR register  ******************/
#define  SCB_MMFAR_ADDRESS                  ((uint32_t)0xFFFFFFFF)        /*!< Mem Manage fault address field */

/*******************  Bit definition for SCB_BFAR register  *******************/
#define  SCB_BFAR_ADDRESS                   ((uint32_t)0xFFFFFFFF)        /*!< Bus fault address field */

/*******************  Bit definition for SCB_AFSR register  *******************/
#define  SCB_AFSR_IMPDEF                    ((uint32_t)0xFFFFFFFF)        /*!< Implementation defined */

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for EXTI_INTEN register  *******************/
#define  EXTI_INTEN_LN0                     ((uint32_t)0x00000001)        /*!< Interrupt Mask on line 0 */
#define  EXTI_INTEN_LN1                     ((uint32_t)0x00000002)        /*!< Interrupt Mask on line 1 */
#define  EXTI_INTEN_LN2                     ((uint32_t)0x00000004)        /*!< Interrupt Mask on line 2 */
#define  EXTI_INTEN_LN3                     ((uint32_t)0x00000008)        /*!< Interrupt Mask on line 3 */
#define  EXTI_INTEN_LN4                     ((uint32_t)0x00000010)        /*!< Interrupt Mask on line 4 */
#define  EXTI_INTEN_LN5                     ((uint32_t)0x00000020)        /*!< Interrupt Mask on line 5 */
#define  EXTI_INTEN_LN6                     ((uint32_t)0x00000040)        /*!< Interrupt Mask on line 6 */
#define  EXTI_INTEN_LN7                     ((uint32_t)0x00000080)        /*!< Interrupt Mask on line 7 */
#define  EXTI_INTEN_LN8                     ((uint32_t)0x00000100)        /*!< Interrupt Mask on line 8 */
#define  EXTI_INTEN_LN9                     ((uint32_t)0x00000200)        /*!< Interrupt Mask on line 9 */
#define  EXTI_INTEN_LN10                    ((uint32_t)0x00000400)        /*!< Interrupt Mask on line 10 */
#define  EXTI_INTEN_LN11                    ((uint32_t)0x00000800)        /*!< Interrupt Mask on line 11 */
#define  EXTI_INTEN_LN12                    ((uint32_t)0x00001000)        /*!< Interrupt Mask on line 12 */
#define  EXTI_INTEN_LN13                    ((uint32_t)0x00002000)        /*!< Interrupt Mask on line 13 */
#define  EXTI_INTEN_LN14                    ((uint32_t)0x00004000)        /*!< Interrupt Mask on line 14 */
#define  EXTI_INTEN_LN15                    ((uint32_t)0x00008000)        /*!< Interrupt Mask on line 15 */
#define  EXTI_INTEN_LN16                    ((uint32_t)0x00010000)        /*!< Interrupt Mask on line 16 */
#define  EXTI_INTEN_LN17                    ((uint32_t)0x00020000)        /*!< Interrupt Mask on line 17 */
#define  EXTI_INTEN_LN18                    ((uint32_t)0x00040000)        /*!< Interrupt Mask on line 18 */
#define  EXTI_INTEN_LN19                    ((uint32_t)0x00080000)        /*!< Interrupt Mask on line 19 */

/*******************  Bit definition for EXTI_EVTEN register  *******************/
#define  EXTI_EVTEN_LN0                     ((uint32_t)0x00000001)        /*!< Event Mask on line 0 */
#define  EXTI_EVTEN_LN1                     ((uint32_t)0x00000002)        /*!< Event Mask on line 1 */
#define  EXTI_EVTEN_LN2                     ((uint32_t)0x00000004)        /*!< Event Mask on line 2 */
#define  EXTI_EVTEN_LN3                     ((uint32_t)0x00000008)        /*!< Event Mask on line 3 */
#define  EXTI_EVTEN_LN4                     ((uint32_t)0x00000010)        /*!< Event Mask on line 4 */
#define  EXTI_EVTEN_LN5                     ((uint32_t)0x00000020)        /*!< Event Mask on line 5 */
#define  EXTI_EVTEN_LN6                     ((uint32_t)0x00000040)        /*!< Event Mask on line 6 */
#define  EXTI_EVTEN_LN7                     ((uint32_t)0x00000080)        /*!< Event Mask on line 7 */
#define  EXTI_EVTEN_LN8                     ((uint32_t)0x00000100)        /*!< Event Mask on line 8 */
#define  EXTI_EVTEN_LN9                     ((uint32_t)0x00000200)        /*!< Event Mask on line 9 */
#define  EXTI_EVTEN_LN10                    ((uint32_t)0x00000400)        /*!< Event Mask on line 10 */
#define  EXTI_EVTEN_LN11                    ((uint32_t)0x00000800)        /*!< Event Mask on line 11 */
#define  EXTI_EVTEN_LN12                    ((uint32_t)0x00001000)        /*!< Event Mask on line 12 */
#define  EXTI_EVTEN_LN13                    ((uint32_t)0x00002000)        /*!< Event Mask on line 13 */
#define  EXTI_EVTEN_LN14                    ((uint32_t)0x00004000)        /*!< Event Mask on line 14 */
#define  EXTI_EVTEN_LN15                    ((uint32_t)0x00008000)        /*!< Event Mask on line 15 */
#define  EXTI_EVTEN_LN16                    ((uint32_t)0x00010000)        /*!< Event Mask on line 16 */
#define  EXTI_EVTEN_LN17                    ((uint32_t)0x00020000)        /*!< Event Mask on line 17 */
#define  EXTI_EVTEN_LN18                    ((uint32_t)0x00040000)        /*!< Event Mask on line 18 */
#define  EXTI_EVTEN_LN19                    ((uint32_t)0x00080000)        /*!< Event Mask on line 19 */

/******************  Bit definition for EXTI_RTRSEL register  *******************/
#define  EXTI_RTRSEL_LN0                    ((uint32_t)0x00000001)        /*!< Rising trigger event configuration bit of line 0 */
#define  EXTI_RTRSEL_LN1                    ((uint32_t)0x00000002)        /*!< Rising trigger event configuration bit of line 1 */
#define  EXTI_RTRSEL_LN2                    ((uint32_t)0x00000004)        /*!< Rising trigger event configuration bit of line 2 */
#define  EXTI_RTRSEL_LN3                    ((uint32_t)0x00000008)        /*!< Rising trigger event configuration bit of line 3 */
#define  EXTI_RTRSEL_LN4                    ((uint32_t)0x00000010)        /*!< Rising trigger event configuration bit of line 4 */
#define  EXTI_RTRSEL_LN5                    ((uint32_t)0x00000020)        /*!< Rising trigger event configuration bit of line 5 */
#define  EXTI_RTRSEL_LN6                    ((uint32_t)0x00000040)        /*!< Rising trigger event configuration bit of line 6 */
#define  EXTI_RTRSEL_LN7                    ((uint32_t)0x00000080)        /*!< Rising trigger event configuration bit of line 7 */
#define  EXTI_RTRSEL_LN8                    ((uint32_t)0x00000100)        /*!< Rising trigger event configuration bit of line 8 */
#define  EXTI_RTRSEL_LN9                    ((uint32_t)0x00000200)        /*!< Rising trigger event configuration bit of line 9 */
#define  EXTI_RTRSEL_LN10                   ((uint32_t)0x00000400)        /*!< Rising trigger event configuration bit of line 10 */
#define  EXTI_RTRSEL_LN11                   ((uint32_t)0x00000800)        /*!< Rising trigger event configuration bit of line 11 */
#define  EXTI_RTRSEL_LN12                   ((uint32_t)0x00001000)        /*!< Rising trigger event configuration bit of line 12 */
#define  EXTI_RTRSEL_LN13                   ((uint32_t)0x00002000)        /*!< Rising trigger event configuration bit of line 13 */
#define  EXTI_RTRSEL_LN14                   ((uint32_t)0x00004000)        /*!< Rising trigger event configuration bit of line 14 */
#define  EXTI_RTRSEL_LN15                   ((uint32_t)0x00008000)        /*!< Rising trigger event configuration bit of line 15 */
#define  EXTI_RTRSEL_LN16                   ((uint32_t)0x00010000)        /*!< Rising trigger event configuration bit of line 16 */
#define  EXTI_RTRSEL_LN17                   ((uint32_t)0x00020000)        /*!< Rising trigger event configuration bit of line 17 */
#define  EXTI_RTRSEL_LN18                   ((uint32_t)0x00040000)        /*!< Rising trigger event configuration bit of line 18 */
#define  EXTI_RTRSEL_LN19                   ((uint32_t)0x00080000)        /*!< Rising trigger event configuration bit of line 19 */

/******************  Bit definition for EXTI_FTRSEL register  *******************/
#define  EXTI_FTRSEL_LN0                    ((uint32_t)0x00000001)        /*!< Falling trigger event configuration bit of line 0 */
#define  EXTI_FTRSEL_LN1                    ((uint32_t)0x00000002)        /*!< Falling trigger event configuration bit of line 1 */
#define  EXTI_FTRSEL_LN2                    ((uint32_t)0x00000004)        /*!< Falling trigger event configuration bit of line 2 */
#define  EXTI_FTRSEL_LN3                    ((uint32_t)0x00000008)        /*!< Falling trigger event configuration bit of line 3 */
#define  EXTI_FTRSEL_LN4                    ((uint32_t)0x00000010)        /*!< Falling trigger event configuration bit of line 4 */
#define  EXTI_FTRSEL_LN5                    ((uint32_t)0x00000020)        /*!< Falling trigger event configuration bit of line 5 */
#define  EXTI_FTRSEL_LN6                    ((uint32_t)0x00000040)        /*!< Falling trigger event configuration bit of line 6 */
#define  EXTI_FTRSEL_LN7                    ((uint32_t)0x00000080)        /*!< Falling trigger event configuration bit of line 7 */
#define  EXTI_FTRSEL_LN8                    ((uint32_t)0x00000100)        /*!< Falling trigger event configuration bit of line 8 */
#define  EXTI_FTRSEL_LN9                    ((uint32_t)0x00000200)        /*!< Falling trigger event configuration bit of line 9 */
#define  EXTI_FTRSEL_LN10                   ((uint32_t)0x00000400)        /*!< Falling trigger event configuration bit of line 10 */
#define  EXTI_FTRSEL_LN11                   ((uint32_t)0x00000800)        /*!< Falling trigger event configuration bit of line 11 */
#define  EXTI_FTRSEL_LN12                   ((uint32_t)0x00001000)        /*!< Falling trigger event configuration bit of line 12 */
#define  EXTI_FTRSEL_LN13                   ((uint32_t)0x00002000)        /*!< Falling trigger event configuration bit of line 13 */
#define  EXTI_FTRSEL_LN14                   ((uint32_t)0x00004000)        /*!< Falling trigger event configuration bit of line 14 */
#define  EXTI_FTRSEL_LN15                   ((uint32_t)0x00008000)        /*!< Falling trigger event configuration bit of line 15 */
#define  EXTI_FTRSEL_LN16                   ((uint32_t)0x00010000)        /*!< Falling trigger event configuration bit of line 16 */
#define  EXTI_FTRSEL_LN17                   ((uint32_t)0x00020000)        /*!< Falling trigger event configuration bit of line 17 */
#define  EXTI_FTRSEL_LN18                   ((uint32_t)0x00040000)        /*!< Falling trigger event configuration bit of line 18 */
#define  EXTI_FTRSEL_LN19                   ((uint32_t)0x00080000)        /*!< Falling trigger event configuration bit of line 19 */

/******************  Bit definition for EXTI_SWIE register  ******************/
#define  EXTI_SWIE_LN0                      ((uint32_t)0x00000001)        /*!< Software Interrupt on line 0 */
#define  EXTI_SWIE_LN1                      ((uint32_t)0x00000002)        /*!< Software Interrupt on line 1 */
#define  EXTI_SWIE_LN2                      ((uint32_t)0x00000004)        /*!< Software Interrupt on line 2 */
#define  EXTI_SWIE_LN3                      ((uint32_t)0x00000008)        /*!< Software Interrupt on line 3 */
#define  EXTI_SWIE_LN4                      ((uint32_t)0x00000010)        /*!< Software Interrupt on line 4 */
#define  EXTI_SWIE_LN5                      ((uint32_t)0x00000020)        /*!< Software Interrupt on line 5 */
#define  EXTI_SWIE_LN6                      ((uint32_t)0x00000040)        /*!< Software Interrupt on line 6 */
#define  EXTI_SWIE_LN7                      ((uint32_t)0x00000080)        /*!< Software Interrupt on line 7 */
#define  EXTI_SWIE_LN8                      ((uint32_t)0x00000100)        /*!< Software Interrupt on line 8 */
#define  EXTI_SWIE_LN9                      ((uint32_t)0x00000200)        /*!< Software Interrupt on line 9 */
#define  EXTI_SWIE_LN10                     ((uint32_t)0x00000400)        /*!< Software Interrupt on line 10 */
#define  EXTI_SWIE_LN11                     ((uint32_t)0x00000800)        /*!< Software Interrupt on line 11 */
#define  EXTI_SWIE_LN12                     ((uint32_t)0x00001000)        /*!< Software Interrupt on line 12 */
#define  EXTI_SWIE_LN13                     ((uint32_t)0x00002000)        /*!< Software Interrupt on line 13 */
#define  EXTI_SWIE_LN14                     ((uint32_t)0x00004000)        /*!< Software Interrupt on line 14 */
#define  EXTI_SWIE_LN15                     ((uint32_t)0x00008000)        /*!< Software Interrupt on line 15 */
#define  EXTI_SWIE_LN16                     ((uint32_t)0x00010000)        /*!< Software Interrupt on line 16 */
#define  EXTI_SWIE_LN17                     ((uint32_t)0x00020000)        /*!< Software Interrupt on line 17 */
#define  EXTI_SWIE_LN18                     ((uint32_t)0x00040000)        /*!< Software Interrupt on line 18 */
#define  EXTI_SWIE_LN19                     ((uint32_t)0x00080000)        /*!< Software Interrupt on line 19 */

/*******************  Bit definition for EXTI_PND register  ********************/
#define  EXTI_PND_LN0                       ((uint32_t)0x00000001)        /*!< Pending bit for line 0 */
#define  EXTI_PND_LN1                       ((uint32_t)0x00000002)        /*!< Pending bit for line 1 */
#define  EXTI_PND_LN2                       ((uint32_t)0x00000004)        /*!< Pending bit for line 2 */
#define  EXTI_PND_LN3                       ((uint32_t)0x00000008)        /*!< Pending bit for line 3 */
#define  EXTI_PND_LN4                       ((uint32_t)0x00000010)        /*!< Pending bit for line 4 */
#define  EXTI_PND_LN5                       ((uint32_t)0x00000020)        /*!< Pending bit for line 5 */
#define  EXTI_PND_LN6                       ((uint32_t)0x00000040)        /*!< Pending bit for line 6 */
#define  EXTI_PND_LN7                       ((uint32_t)0x00000080)        /*!< Pending bit for line 7 */
#define  EXTI_PND_LN8                       ((uint32_t)0x00000100)        /*!< Pending bit for line 8 */
#define  EXTI_PND_LN9                       ((uint32_t)0x00000200)        /*!< Pending bit for line 9 */
#define  EXTI_PND_LN10                      ((uint32_t)0x00000400)        /*!< Pending bit for line 10 */
#define  EXTI_PND_LN11                      ((uint32_t)0x00000800)        /*!< Pending bit for line 11 */
#define  EXTI_PND_LN12                      ((uint32_t)0x00001000)        /*!< Pending bit for line 12 */
#define  EXTI_PND_LN13                      ((uint32_t)0x00002000)        /*!< Pending bit for line 13 */
#define  EXTI_PND_LN14                      ((uint32_t)0x00004000)        /*!< Pending bit for line 14 */
#define  EXTI_PND_LN15                      ((uint32_t)0x00008000)        /*!< Pending bit for line 15 */
#define  EXTI_PND_LN16                      ((uint32_t)0x00010000)        /*!< Pending bit for line 16 */
#define  EXTI_PND_LN17                      ((uint32_t)0x00020000)        /*!< Pending bit for line 17 */
#define  EXTI_PND_LN18                      ((uint32_t)0x00040000)        /*!< Pending bit for line 18 */
#define  EXTI_PND_LN19                      ((uint32_t)0x00080000)        /*!< Pending bit for line 19 */

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for DMA_ISTS register  ********************/
#define  DMA_ISTS_GIF1                      ((uint32_t)0x00000001)        /*!< Channel 1 Global interrupt flag */
#define  DMA_ISTS_TCIF1                     ((uint32_t)0x00000002)        /*!< Channel 1 Transfer Complete flag */
#define  DMA_ISTS_HTIF1                     ((uint32_t)0x00000004)        /*!< Channel 1 Half Transfer flag */
#define  DMA_ISTS_ERRIF1                    ((uint32_t)0x00000008)        /*!< Channel 1 Transfer Error flag */
#define  DMA_ISTS_GIF2                      ((uint32_t)0x00000010)        /*!< Channel 2 Global interrupt flag */
#define  DMA_ISTS_TCIF2                     ((uint32_t)0x00000020)        /*!< Channel 2 Transfer Complete flag */
#define  DMA_ISTS_HTIF2                     ((uint32_t)0x00000040)        /*!< Channel 2 Half Transfer flag */
#define  DMA_ISTS_ERRIF2                    ((uint32_t)0x00000080)        /*!< Channel 2 Transfer Error flag */
#define  DMA_ISTS_GIF3                      ((uint32_t)0x00000100)        /*!< Channel 3 Global interrupt flag */
#define  DMA_ISTS_TCIF3                     ((uint32_t)0x00000200)        /*!< Channel 3 Transfer Complete flag */
#define  DMA_ISTS_HTIF3                     ((uint32_t)0x00000400)        /*!< Channel 3 Half Transfer flag */
#define  DMA_ISTS_ERRIF3                    ((uint32_t)0x00000800)        /*!< Channel 3 Transfer Error flag */
#define  DMA_ISTS_GIF4                      ((uint32_t)0x00001000)        /*!< Channel 4 Global interrupt flag */
#define  DMA_ISTS_TCIF4                     ((uint32_t)0x00002000)        /*!< Channel 4 Transfer Complete flag */
#define  DMA_ISTS_HTIF4                     ((uint32_t)0x00004000)        /*!< Channel 4 Half Transfer flag */
#define  DMA_ISTS_ERRIF4                    ((uint32_t)0x00008000)        /*!< Channel 4 Transfer Error flag */
#define  DMA_ISTS_GIF5                      ((uint32_t)0x00010000)        /*!< Channel 5 Global interrupt flag */
#define  DMA_ISTS_TCIF5                     ((uint32_t)0x00020000)        /*!< Channel 5 Transfer Complete flag */
#define  DMA_ISTS_HTIF5                     ((uint32_t)0x00040000)        /*!< Channel 5 Half Transfer flag */
#define  DMA_ISTS_ERRIF5                    ((uint32_t)0x00080000)        /*!< Channel 5 Transfer Error flag */
#define  DMA_ISTS_GIF6                      ((uint32_t)0x00100000)        /*!< Channel 6 Global interrupt flag */
#define  DMA_ISTS_TCIF6                     ((uint32_t)0x00200000)        /*!< Channel 6 Transfer Complete flag */
#define  DMA_ISTS_HTIF6                     ((uint32_t)0x00400000)        /*!< Channel 6 Half Transfer flag */
#define  DMA_ISTS_ERRIF6                    ((uint32_t)0x00800000)        /*!< Channel 6 Transfer Error flag */
#define  DMA_ISTS_GIF7                      ((uint32_t)0x01000000)        /*!< Channel 7 Global interrupt flag */
#define  DMA_ISTS_TCIF7                     ((uint32_t)0x02000000)        /*!< Channel 7 Transfer Complete flag */
#define  DMA_ISTS_HTIF7                     ((uint32_t)0x04000000)        /*!< Channel 7 Half Transfer flag */
#define  DMA_ISTS_ERRIF7                    ((uint32_t)0x08000000)        /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_ICLR register  *******************/
#define  DMA_ICLR_CGIF1                      ((uint32_t)0x00000001)        /*!< Channel 1 Global interrupt clear */
#define  DMA_ICLR_CTCIF1                     ((uint32_t)0x00000002)        /*!< Channel 1 Transfer Complete clear */
#define  DMA_ICLR_CHTIF1                     ((uint32_t)0x00000004)        /*!< Channel 1 Half Transfer clear */
#define  DMA_ICLR_CERRIF1                    ((uint32_t)0x00000008)        /*!< Channel 1 Transfer Error clear */
#define  DMA_ICLR_CGIF2                      ((uint32_t)0x00000010)        /*!< Channel 2 Global interrupt clear */
#define  DMA_ICLR_CTCIF2                     ((uint32_t)0x00000020)        /*!< Channel 2 Transfer Complete clear */
#define  DMA_ICLR_CHTIF2                     ((uint32_t)0x00000040)        /*!< Channel 2 Half Transfer clear */
#define  DMA_ICLR_CERRIF2                    ((uint32_t)0x00000080)        /*!< Channel 2 Transfer Error clear */
#define  DMA_ICLR_CGIF3                      ((uint32_t)0x00000100)        /*!< Channel 3 Global interrupt clear */
#define  DMA_ICLR_CTCIF3                     ((uint32_t)0x00000200)        /*!< Channel 3 Transfer Complete clear */
#define  DMA_ICLR_CHTIF3                     ((uint32_t)0x00000400)        /*!< Channel 3 Half Transfer clear */
#define  DMA_ICLR_CERRIF3                    ((uint32_t)0x00000800)        /*!< Channel 3 Transfer Error clear */
#define  DMA_ICLR_CGIF4                      ((uint32_t)0x00001000)        /*!< Channel 4 Global interrupt clear */
#define  DMA_ICLR_CTCIF4                     ((uint32_t)0x00002000)        /*!< Channel 4 Transfer Complete clear */
#define  DMA_ICLR_CHTIF4                     ((uint32_t)0x00004000)        /*!< Channel 4 Half Transfer clear */
#define  DMA_ICLR_CERRIF4                    ((uint32_t)0x00008000)        /*!< Channel 4 Transfer Error clear */
#define  DMA_ICLR_CGIF5                      ((uint32_t)0x00010000)        /*!< Channel 5 Global interrupt clear */
#define  DMA_ICLR_CTCIF5                     ((uint32_t)0x00020000)        /*!< Channel 5 Transfer Complete clear */
#define  DMA_ICLR_CHTIF5                     ((uint32_t)0x00040000)        /*!< Channel 5 Half Transfer clear */
#define  DMA_ICLR_CERRIF5                    ((uint32_t)0x00080000)        /*!< Channel 5 Transfer Error clear */
#define  DMA_ICLR_CGIF6                      ((uint32_t)0x00100000)        /*!< Channel 6 Global interrupt clear */
#define  DMA_ICLR_CTCIF6                     ((uint32_t)0x00200000)        /*!< Channel 6 Transfer Complete clear */
#define  DMA_ICLR_CHTIF6                     ((uint32_t)0x00400000)        /*!< Channel 6 Half Transfer clear */
#define  DMA_ICLR_CERRIF6                    ((uint32_t)0x00800000)        /*!< Channel 6 Transfer Error clear */
#define  DMA_ICLR_CGIF7                      ((uint32_t)0x01000000)        /*!< Channel 7 Global interrupt clear */
#define  DMA_ICLR_CTCIF7                     ((uint32_t)0x02000000)        /*!< Channel 7 Transfer Complete clear */
#define  DMA_ICLR_CHTIF7                     ((uint32_t)0x04000000)        /*!< Channel 7 Half Transfer clear */
#define  DMA_ICLR_CERRIF7                    ((uint32_t)0x08000000)        /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CHCTRL1 register  *******************/
#define  DMA_CHCTRL1_CHEN                   ((uint16_t)0x0001)            /*!< Channel enable*/
#define  DMA_CHCTRL1_TCIE                   ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CHCTRL1_HTIE                   ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CHCTRL1_ERRIE                  ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CHCTRL1_DIR                    ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CHCTRL1_CIRM                   ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CHCTRL1_PINC                   ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CHCTRL1_MINC                   ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CHCTRL1_PWIDTH                 ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CHCTRL1_PWIDTH_0               ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CHCTRL1_PWIDTH_1               ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CHCTRL1_MWIDTH                 ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CHCTRL1_MWIDTH_0               ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CHCTRL1_MWIDTH_1               ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CHCTRL1_CHPL                   ((uint16_t)0x3000)            /*!< PL[1:0] bits(Channel Priority level) */
#define  DMA_CHCTRL1_CHPL_0                 ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CHCTRL1_CHPL_1                 ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CHCTRL1_MEMTOMEM               ((uint16_t)0x4000)            /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CHCTRL2 register  *******************/
#define  DMA_CHCTRL2_CHEN                   ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CHCTRL2_TCIE                   ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CHCTRL2_HTIE                   ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CHCTRL2_ERRIE                  ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CHCTRL2_DIR                    ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CHCTRL2_CIRM                   ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CHCTRL2_PINC                   ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CHCTRL2_MINC                   ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CHCTRL2_PWIDTH                 ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CHCTRL2_PWIDTH_0               ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CHCTRL2_PWIDTH_1               ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CHCTRL2_MWIDTH                 ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CHCTRL2_MWIDTH_0               ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CHCTRL2_MWIDTH_1               ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CHCTRL2_CHPL                   ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CHCTRL2_CHPL_0                 ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CHCTRL2_CHPL_1                 ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CHCTRL2_MEMTOMEM               ((uint16_t)0x4000)            /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CHCTRL3 register  *******************/
#define  DMA_CHCTRL3_CHEN                   ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CHCTRL3_TCIE                   ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CHCTRL3_HTIE                   ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CHCTRL3_ERRIE                  ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CHCTRL3_DIR                    ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CHCTRL3_CIRM                   ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CHCTRL3_PINC                   ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CHCTRL3_MINC                   ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CHCTRL3_PWIDTH                 ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CHCTRL3_PWIDTH_0               ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CHCTRL3_PWIDTH_1               ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CHCTRL3_MWIDTH                 ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CHCTRL3_MWIDTH_0               ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CHCTRL3_MWIDTH_1               ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CHCTRL3_CHPL                   ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CHCTRL3_CHPL_0                 ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CHCTRL3_CHPL_1                 ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CHCTRL3_MEMTOMEM               ((uint16_t)0x4000)            /*!< Memory to memory mode */

/********************  Bit definition for DMA_CHCTRL4 register  *******************/
#define  DMA_CHCTRL4_CHEN                   ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CHCTRL4_TCIE                   ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CHCTRL4_HTIE                   ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CHCTRL4_ERRIE                  ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CHCTRL4_DIR                    ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CHCTRL4_CIRM                   ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CHCTRL4_PINC                   ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CHCTRL4_MINC                   ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CHCTRL4_PWIDTH                 ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CHCTRL4_PWIDTH_0               ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CHCTRL4_PWIDTH_1               ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CHCTRL4_MWIDTH                 ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CHCTRL4_MWIDTH_0               ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CHCTRL4_MWIDTH_1               ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CHCTRL4_CHPL                   ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CHCTRL4_CHPL_0                 ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CHCTRL4_CHPL_1                 ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CHCTRL4_MEMTOMEM               ((uint16_t)0x4000)            /*!< Memory to memory mode */

/******************  Bit definition for DMA_CHCTRL5 register  *******************/
#define  DMA_CHCTRL5_CHEN                   ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CHCTRL5_TCIE                   ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CHCTRL5_HTIE                   ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CHCTRL5_ERRIE                  ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CHCTRL5_DIR                    ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CHCTRL5_CIRM                   ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CHCTRL5_PINC                   ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CHCTRL5_MINC                   ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CHCTRL5_PWIDTH                 ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CHCTRL5_PWIDTH_0               ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CHCTRL5_PWIDTH_1               ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CHCTRL5_MWIDTH                 ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CHCTRL5_MWIDTH_0               ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CHCTRL5_MWIDTH_1               ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CHCTRL5_CHPL                   ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CHCTRL5_CHPL_0                 ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CHCTRL5_CHPL_1                 ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CHCTRL5_MEMTOMEM               ((uint16_t)0x4000)            /*!< Memory to memory mode enable */

/*******************  Bit definition for DMA_CHCTRL6 register  *******************/
#define  DMA_CHCTRL6_CHEN                   ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CHCTRL6_TCIE                   ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CHCTRL6_HTIE                   ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CHCTRL6_ERRIE                  ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CHCTRL6_DIR                    ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CHCTRL6_CIRM                   ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CHCTRL6_PINC                   ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CHCTRL6_MINC                   ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CHCTRL6_PWIDTH                 ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CHCTRL6_PWIDTH_0               ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CHCTRL6_PWIDTH_1               ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CHCTRL6_MWIDTH                 ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CHCTRL6_MWIDTH_0               ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CHCTRL6_MWIDTH_1               ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CHCTRL6_CHPL                   ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CHCTRL6_CHPL_0                 ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CHCTRL6_CHPL_1                 ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CHCTRL6_MEMTOMEM               ((uint16_t)0x4000)            /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CHCTRL7 register  *******************/
#define  DMA_CHCTRL7_CHEN                   ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CHCTRL7_TCIE                   ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CHCTRL7_HTIE                   ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CHCTRL7_ERRIE                  ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CHCTRL7_DIR                    ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CHCTRL7_CIRM                   ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CHCTRL7_PINC                   ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CHCTRL7_MINC                   ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CHCTRL7_PWIDTH            ,    ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CHCTRL7_PWIDTH_0               ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CHCTRL7_PWIDTH_1               ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CHCTRL7_MWIDTH                 ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CHCTRL7_MWIDTH_0               ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CHCTRL7_MWIDTH_1               ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CHCTRL7_CHPL                   ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CHCTRL7_CHPL_0                 ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CHCTRL7_CHPL_1                 ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CHCTRL7_MEMTOMEM               ((uint16_t)0x4000)            /*!< Memory to memory mode enable */

///////////////////////////////////////////////////////////////////////////////
// copied from stm file
#define DMA_ISR_GIF1_Pos                    (0U)                               
#define DMA_ISR_GIF1_Msk                    (0x1U << DMA_ISR_GIF1_Pos)         /*!< 0x00000001 */
#define DMA_ISR_GIF1                        DMA_ISR_GIF1_Msk                   /*!< Channel 1 Global interrupt flag */
#define DMA_ISR_TCIF1_Pos                   (1U)                               
#define DMA_ISR_TCIF1_Msk                   (0x1U << DMA_ISR_TCIF1_Pos)        /*!< 0x00000002 */
#define DMA_ISR_TCIF1                       DMA_ISR_TCIF1_Msk                  /*!< Channel 1 Transfer Complete flag */
#define DMA_ISR_HTIF1_Pos                   (2U)                               
#define DMA_ISR_HTIF1_Msk                   (0x1U << DMA_ISR_HTIF1_Pos)        /*!< 0x00000004 */
#define DMA_ISR_HTIF1                       DMA_ISR_HTIF1_Msk                  /*!< Channel 1 Half Transfer flag */
#define DMA_ISR_TEIF1_Pos                   (3U)                               
#define DMA_ISR_TEIF1_Msk                   (0x1U << DMA_ISR_TEIF1_Pos)        /*!< 0x00000008 */
#define DMA_ISR_TEIF1                       DMA_ISR_TEIF1_Msk                  /*!< Channel 1 Transfer Error flag */
#define DMA_ISR_GIF2_Pos                    (4U)                               
#define DMA_ISR_GIF2_Msk                    (0x1U << DMA_ISR_GIF2_Pos)         /*!< 0x00000010 */
#define DMA_ISR_GIF2                        DMA_ISR_GIF2_Msk                   /*!< Channel 2 Global interrupt flag */
#define DMA_ISR_TCIF2_Pos                   (5U)                               
#define DMA_ISR_TCIF2_Msk                   (0x1U << DMA_ISR_TCIF2_Pos)        /*!< 0x00000020 */
#define DMA_ISR_TCIF2                       DMA_ISR_TCIF2_Msk                  /*!< Channel 2 Transfer Complete flag */
#define DMA_ISR_HTIF2_Pos                   (6U)                               
#define DMA_ISR_HTIF2_Msk                   (0x1U << DMA_ISR_HTIF2_Pos)        /*!< 0x00000040 */
#define DMA_ISR_HTIF2                       DMA_ISR_HTIF2_Msk                  /*!< Channel 2 Half Transfer flag */
#define DMA_ISR_TEIF2_Pos                   (7U)                               
#define DMA_ISR_TEIF2_Msk                   (0x1U << DMA_ISR_TEIF2_Pos)        /*!< 0x00000080 */
#define DMA_ISR_TEIF2                       DMA_ISR_TEIF2_Msk                  /*!< Channel 2 Transfer Error flag */
#define DMA_ISR_GIF3_Pos                    (8U)                               
#define DMA_ISR_GIF3_Msk                    (0x1U << DMA_ISR_GIF3_Pos)         /*!< 0x00000100 */
#define DMA_ISR_GIF3                        DMA_ISR_GIF3_Msk                   /*!< Channel 3 Global interrupt flag */
#define DMA_ISR_TCIF3_Pos                   (9U)                               
#define DMA_ISR_TCIF3_Msk                   (0x1U << DMA_ISR_TCIF3_Pos)        /*!< 0x00000200 */
#define DMA_ISR_TCIF3                       DMA_ISR_TCIF3_Msk                  /*!< Channel 3 Transfer Complete flag */
#define DMA_ISR_HTIF3_Pos                   (10U)                              
#define DMA_ISR_HTIF3_Msk                   (0x1U << DMA_ISR_HTIF3_Pos)        /*!< 0x00000400 */
#define DMA_ISR_HTIF3                       DMA_ISR_HTIF3_Msk                  /*!< Channel 3 Half Transfer flag */
#define DMA_ISR_TEIF3_Pos                   (11U)                              
#define DMA_ISR_TEIF3_Msk                   (0x1U << DMA_ISR_TEIF3_Pos)        /*!< 0x00000800 */
#define DMA_ISR_TEIF3                       DMA_ISR_TEIF3_Msk                  /*!< Channel 3 Transfer Error flag */
#define DMA_ISR_GIF4_Pos                    (12U)                              
#define DMA_ISR_GIF4_Msk                    (0x1U << DMA_ISR_GIF4_Pos)         /*!< 0x00001000 */
#define DMA_ISR_GIF4                        DMA_ISR_GIF4_Msk                   /*!< Channel 4 Global interrupt flag */
#define DMA_ISR_TCIF4_Pos                   (13U)                              
#define DMA_ISR_TCIF4_Msk                   (0x1U << DMA_ISR_TCIF4_Pos)        /*!< 0x00002000 */
#define DMA_ISR_TCIF4                       DMA_ISR_TCIF4_Msk                  /*!< Channel 4 Transfer Complete flag */
#define DMA_ISR_HTIF4_Pos                   (14U)                              
#define DMA_ISR_HTIF4_Msk                   (0x1U << DMA_ISR_HTIF4_Pos)        /*!< 0x00004000 */
#define DMA_ISR_HTIF4                       DMA_ISR_HTIF4_Msk                  /*!< Channel 4 Half Transfer flag */
#define DMA_ISR_TEIF4_Pos                   (15U)                              
#define DMA_ISR_TEIF4_Msk                   (0x1U << DMA_ISR_TEIF4_Pos)        /*!< 0x00008000 */
#define DMA_ISR_TEIF4                       DMA_ISR_TEIF4_Msk                  /*!< Channel 4 Transfer Error flag */
#define DMA_ISR_GIF5_Pos                    (16U)                              
#define DMA_ISR_GIF5_Msk                    (0x1U << DMA_ISR_GIF5_Pos)         /*!< 0x00010000 */
#define DMA_ISR_GIF5                        DMA_ISR_GIF5_Msk                   /*!< Channel 5 Global interrupt flag */
#define DMA_ISR_TCIF5_Pos                   (17U)                              
#define DMA_ISR_TCIF5_Msk                   (0x1U << DMA_ISR_TCIF5_Pos)        /*!< 0x00020000 */
#define DMA_ISR_TCIF5                       DMA_ISR_TCIF5_Msk                  /*!< Channel 5 Transfer Complete flag */
#define DMA_ISR_HTIF5_Pos                   (18U)                              
#define DMA_ISR_HTIF5_Msk                   (0x1U << DMA_ISR_HTIF5_Pos)        /*!< 0x00040000 */
#define DMA_ISR_HTIF5                       DMA_ISR_HTIF5_Msk                  /*!< Channel 5 Half Transfer flag */
#define DMA_ISR_TEIF5_Pos                   (19U)                              
#define DMA_ISR_TEIF5_Msk                   (0x1U << DMA_ISR_TEIF5_Pos)        /*!< 0x00080000 */
#define DMA_ISR_TEIF5                       DMA_ISR_TEIF5_Msk                  /*!< Channel 5 Transfer Error flag */
#define DMA_ISR_GIF6_Pos                    (20U)                              
#define DMA_ISR_GIF6_Msk                    (0x1U << DMA_ISR_GIF6_Pos)         /*!< 0x00100000 */
#define DMA_ISR_GIF6                        DMA_ISR_GIF6_Msk                   /*!< Channel 6 Global interrupt flag */
#define DMA_ISR_TCIF6_Pos                   (21U)                              
#define DMA_ISR_TCIF6_Msk                   (0x1U << DMA_ISR_TCIF6_Pos)        /*!< 0x00200000 */
#define DMA_ISR_TCIF6                       DMA_ISR_TCIF6_Msk                  /*!< Channel 6 Transfer Complete flag */
#define DMA_ISR_HTIF6_Pos                   (22U)                              
#define DMA_ISR_HTIF6_Msk                   (0x1U << DMA_ISR_HTIF6_Pos)        /*!< 0x00400000 */
#define DMA_ISR_HTIF6                       DMA_ISR_HTIF6_Msk                  /*!< Channel 6 Half Transfer flag */
#define DMA_ISR_TEIF6_Pos                   (23U)                              
#define DMA_ISR_TEIF6_Msk                   (0x1U << DMA_ISR_TEIF6_Pos)        /*!< 0x00800000 */
#define DMA_ISR_TEIF6                       DMA_ISR_TEIF6_Msk                  /*!< Channel 6 Transfer Error flag */
#define DMA_ISR_GIF7_Pos                    (24U)                              
#define DMA_ISR_GIF7_Msk                    (0x1U << DMA_ISR_GIF7_Pos)         /*!< 0x01000000 */
#define DMA_ISR_GIF7                        DMA_ISR_GIF7_Msk                   /*!< Channel 7 Global interrupt flag */
#define DMA_ISR_TCIF7_Pos                   (25U)                              
#define DMA_ISR_TCIF7_Msk                   (0x1U << DMA_ISR_TCIF7_Pos)        /*!< 0x02000000 */
#define DMA_ISR_TCIF7                       DMA_ISR_TCIF7_Msk                  /*!< Channel 7 Transfer Complete flag */
#define DMA_ISR_HTIF7_Pos                   (26U)                              
#define DMA_ISR_HTIF7_Msk                   (0x1U << DMA_ISR_HTIF7_Pos)        /*!< 0x04000000 */
#define DMA_ISR_HTIF7                       DMA_ISR_HTIF7_Msk                  /*!< Channel 7 Half Transfer flag */
#define DMA_ISR_TEIF7_Pos                   (27U)                              
#define DMA_ISR_TEIF7_Msk                   (0x1U << DMA_ISR_TEIF7_Pos)        /*!< 0x08000000 */
#define DMA_ISR_TEIF7                       DMA_ISR_TEIF7_Msk                  /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define DMA_IFCR_CGIF1_Pos                  (0U)                               
#define DMA_IFCR_CGIF1_Msk                  (0x1U << DMA_IFCR_CGIF1_Pos)       /*!< 0x00000001 */
#define DMA_IFCR_CGIF1                      DMA_IFCR_CGIF1_Msk                 /*!< Channel 1 Global interrupt clear */
#define DMA_IFCR_CTCIF1_Pos                 (1U)                               
#define DMA_IFCR_CTCIF1_Msk                 (0x1U << DMA_IFCR_CTCIF1_Pos)      /*!< 0x00000002 */
#define DMA_IFCR_CTCIF1                     DMA_IFCR_CTCIF1_Msk                /*!< Channel 1 Transfer Complete clear */
#define DMA_IFCR_CHTIF1_Pos                 (2U)                               
#define DMA_IFCR_CHTIF1_Msk                 (0x1U << DMA_IFCR_CHTIF1_Pos)      /*!< 0x00000004 */
#define DMA_IFCR_CHTIF1                     DMA_IFCR_CHTIF1_Msk                /*!< Channel 1 Half Transfer clear */
#define DMA_IFCR_CTEIF1_Pos                 (3U)                               
#define DMA_IFCR_CTEIF1_Msk                 (0x1U << DMA_IFCR_CTEIF1_Pos)      /*!< 0x00000008 */
#define DMA_IFCR_CTEIF1                     DMA_IFCR_CTEIF1_Msk                /*!< Channel 1 Transfer Error clear */
#define DMA_IFCR_CGIF2_Pos                  (4U)                               
#define DMA_IFCR_CGIF2_Msk                  (0x1U << DMA_IFCR_CGIF2_Pos)       /*!< 0x00000010 */
#define DMA_IFCR_CGIF2                      DMA_IFCR_CGIF2_Msk                 /*!< Channel 2 Global interrupt clear */
#define DMA_IFCR_CTCIF2_Pos                 (5U)                               
#define DMA_IFCR_CTCIF2_Msk                 (0x1U << DMA_IFCR_CTCIF2_Pos)      /*!< 0x00000020 */
#define DMA_IFCR_CTCIF2                     DMA_IFCR_CTCIF2_Msk                /*!< Channel 2 Transfer Complete clear */
#define DMA_IFCR_CHTIF2_Pos                 (6U)                               
#define DMA_IFCR_CHTIF2_Msk                 (0x1U << DMA_IFCR_CHTIF2_Pos)      /*!< 0x00000040 */
#define DMA_IFCR_CHTIF2                     DMA_IFCR_CHTIF2_Msk                /*!< Channel 2 Half Transfer clear */
#define DMA_IFCR_CTEIF2_Pos                 (7U)                               
#define DMA_IFCR_CTEIF2_Msk                 (0x1U << DMA_IFCR_CTEIF2_Pos)      /*!< 0x00000080 */
#define DMA_IFCR_CTEIF2                     DMA_IFCR_CTEIF2_Msk                /*!< Channel 2 Transfer Error clear */
#define DMA_IFCR_CGIF3_Pos                  (8U)                               
#define DMA_IFCR_CGIF3_Msk                  (0x1U << DMA_IFCR_CGIF3_Pos)       /*!< 0x00000100 */
#define DMA_IFCR_CGIF3                      DMA_IFCR_CGIF3_Msk                 /*!< Channel 3 Global interrupt clear */
#define DMA_IFCR_CTCIF3_Pos                 (9U)                               
#define DMA_IFCR_CTCIF3_Msk                 (0x1U << DMA_IFCR_CTCIF3_Pos)      /*!< 0x00000200 */
#define DMA_IFCR_CTCIF3                     DMA_IFCR_CTCIF3_Msk                /*!< Channel 3 Transfer Complete clear */
#define DMA_IFCR_CHTIF3_Pos                 (10U)                              
#define DMA_IFCR_CHTIF3_Msk                 (0x1U << DMA_IFCR_CHTIF3_Pos)      /*!< 0x00000400 */
#define DMA_IFCR_CHTIF3                     DMA_IFCR_CHTIF3_Msk                /*!< Channel 3 Half Transfer clear */
#define DMA_IFCR_CTEIF3_Pos                 (11U)                              
#define DMA_IFCR_CTEIF3_Msk                 (0x1U << DMA_IFCR_CTEIF3_Pos)      /*!< 0x00000800 */
#define DMA_IFCR_CTEIF3                     DMA_IFCR_CTEIF3_Msk                /*!< Channel 3 Transfer Error clear */
#define DMA_IFCR_CGIF4_Pos                  (12U)                              
#define DMA_IFCR_CGIF4_Msk                  (0x1U << DMA_IFCR_CGIF4_Pos)       /*!< 0x00001000 */
#define DMA_IFCR_CGIF4                      DMA_IFCR_CGIF4_Msk                 /*!< Channel 4 Global interrupt clear */
#define DMA_IFCR_CTCIF4_Pos                 (13U)                              
#define DMA_IFCR_CTCIF4_Msk                 (0x1U << DMA_IFCR_CTCIF4_Pos)      /*!< 0x00002000 */
#define DMA_IFCR_CTCIF4                     DMA_IFCR_CTCIF4_Msk                /*!< Channel 4 Transfer Complete clear */
#define DMA_IFCR_CHTIF4_Pos                 (14U)                              
#define DMA_IFCR_CHTIF4_Msk                 (0x1U << DMA_IFCR_CHTIF4_Pos)      /*!< 0x00004000 */
#define DMA_IFCR_CHTIF4                     DMA_IFCR_CHTIF4_Msk                /*!< Channel 4 Half Transfer clear */
#define DMA_IFCR_CTEIF4_Pos                 (15U)                              
#define DMA_IFCR_CTEIF4_Msk                 (0x1U << DMA_IFCR_CTEIF4_Pos)      /*!< 0x00008000 */
#define DMA_IFCR_CTEIF4                     DMA_IFCR_CTEIF4_Msk                /*!< Channel 4 Transfer Error clear */
#define DMA_IFCR_CGIF5_Pos                  (16U)                              
#define DMA_IFCR_CGIF5_Msk                  (0x1U << DMA_IFCR_CGIF5_Pos)       /*!< 0x00010000 */
#define DMA_IFCR_CGIF5                      DMA_IFCR_CGIF5_Msk                 /*!< Channel 5 Global interrupt clear */
#define DMA_IFCR_CTCIF5_Pos                 (17U)                              
#define DMA_IFCR_CTCIF5_Msk                 (0x1U << DMA_IFCR_CTCIF5_Pos)      /*!< 0x00020000 */
#define DMA_IFCR_CTCIF5                     DMA_IFCR_CTCIF5_Msk                /*!< Channel 5 Transfer Complete clear */
#define DMA_IFCR_CHTIF5_Pos                 (18U)                              
#define DMA_IFCR_CHTIF5_Msk                 (0x1U << DMA_IFCR_CHTIF5_Pos)      /*!< 0x00040000 */
#define DMA_IFCR_CHTIF5                     DMA_IFCR_CHTIF5_Msk                /*!< Channel 5 Half Transfer clear */
#define DMA_IFCR_CTEIF5_Pos                 (19U)                              
#define DMA_IFCR_CTEIF5_Msk                 (0x1U << DMA_IFCR_CTEIF5_Pos)      /*!< 0x00080000 */
#define DMA_IFCR_CTEIF5                     DMA_IFCR_CTEIF5_Msk                /*!< Channel 5 Transfer Error clear */
#define DMA_IFCR_CGIF6_Pos                  (20U)                              
#define DMA_IFCR_CGIF6_Msk                  (0x1U << DMA_IFCR_CGIF6_Pos)       /*!< 0x00100000 */
#define DMA_IFCR_CGIF6                      DMA_IFCR_CGIF6_Msk                 /*!< Channel 6 Global interrupt clear */
#define DMA_IFCR_CTCIF6_Pos                 (21U)                              
#define DMA_IFCR_CTCIF6_Msk                 (0x1U << DMA_IFCR_CTCIF6_Pos)      /*!< 0x00200000 */
#define DMA_IFCR_CTCIF6                     DMA_IFCR_CTCIF6_Msk                /*!< Channel 6 Transfer Complete clear */
#define DMA_IFCR_CHTIF6_Pos                 (22U)                              
#define DMA_IFCR_CHTIF6_Msk                 (0x1U << DMA_IFCR_CHTIF6_Pos)      /*!< 0x00400000 */
#define DMA_IFCR_CHTIF6                     DMA_IFCR_CHTIF6_Msk                /*!< Channel 6 Half Transfer clear */
#define DMA_IFCR_CTEIF6_Pos                 (23U)                              
#define DMA_IFCR_CTEIF6_Msk                 (0x1U << DMA_IFCR_CTEIF6_Pos)      /*!< 0x00800000 */
#define DMA_IFCR_CTEIF6                     DMA_IFCR_CTEIF6_Msk                /*!< Channel 6 Transfer Error clear */
#define DMA_IFCR_CGIF7_Pos                  (24U)                              
#define DMA_IFCR_CGIF7_Msk                  (0x1U << DMA_IFCR_CGIF7_Pos)       /*!< 0x01000000 */
#define DMA_IFCR_CGIF7                      DMA_IFCR_CGIF7_Msk                 /*!< Channel 7 Global interrupt clear */
#define DMA_IFCR_CTCIF7_Pos                 (25U)                              
#define DMA_IFCR_CTCIF7_Msk                 (0x1U << DMA_IFCR_CTCIF7_Pos)      /*!< 0x02000000 */
#define DMA_IFCR_CTCIF7                     DMA_IFCR_CTCIF7_Msk                /*!< Channel 7 Transfer Complete clear */
#define DMA_IFCR_CHTIF7_Pos                 (26U)                              
#define DMA_IFCR_CHTIF7_Msk                 (0x1U << DMA_IFCR_CHTIF7_Pos)      /*!< 0x04000000 */
#define DMA_IFCR_CHTIF7                     DMA_IFCR_CHTIF7_Msk                /*!< Channel 7 Half Transfer clear */
#define DMA_IFCR_CTEIF7_Pos                 (27U)                              
#define DMA_IFCR_CTEIF7_Msk                 (0x1U << DMA_IFCR_CTEIF7_Pos)      /*!< 0x08000000 */
#define DMA_IFCR_CTEIF7                     DMA_IFCR_CTEIF7_Msk                /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR register   *******************/
#define DMA_CCR_EN_Pos                      (0U)                               
#define DMA_CCR_EN_Msk                      (0x1U << DMA_CCR_EN_Pos)           /*!< 0x00000001 */
#define DMA_CCR_EN                          DMA_CCR_EN_Msk                     /*!< Channel enable */
#define DMA_CCR_TCIE_Pos                    (1U)                               
#define DMA_CCR_TCIE_Msk                    (0x1U << DMA_CCR_TCIE_Pos)         /*!< 0x00000002 */
#define DMA_CCR_TCIE                        DMA_CCR_TCIE_Msk                   /*!< Transfer complete interrupt enable */
#define DMA_CCR_HTIE_Pos                    (2U)                               
#define DMA_CCR_HTIE_Msk                    (0x1U << DMA_CCR_HTIE_Pos)         /*!< 0x00000004 */
#define DMA_CCR_HTIE                        DMA_CCR_HTIE_Msk                   /*!< Half Transfer interrupt enable */
#define DMA_CCR_TEIE_Pos                    (3U)                               
#define DMA_CCR_TEIE_Msk                    (0x1U << DMA_CCR_TEIE_Pos)         /*!< 0x00000008 */
#define DMA_CCR_TEIE                        DMA_CCR_TEIE_Msk                   /*!< Transfer error interrupt enable */
#define DMA_CCR_DIR_Pos                     (4U)                               
#define DMA_CCR_DIR_Msk                     (0x1U << DMA_CCR_DIR_Pos)          /*!< 0x00000010 */
#define DMA_CCR_DIR                         DMA_CCR_DIR_Msk                    /*!< Data transfer direction */
#define DMA_CCR_CIRC_Pos                    (5U)                               
#define DMA_CCR_CIRC_Msk                    (0x1U << DMA_CCR_CIRC_Pos)         /*!< 0x00000020 */
#define DMA_CCR_CIRC                        DMA_CCR_CIRC_Msk                   /*!< Circular mode */
#define DMA_CCR_PINC_Pos                    (6U)                               
#define DMA_CCR_PINC_Msk                    (0x1U << DMA_CCR_PINC_Pos)         /*!< 0x00000040 */
#define DMA_CCR_PINC                        DMA_CCR_PINC_Msk                   /*!< Peripheral increment mode */
#define DMA_CCR_MINC_Pos                    (7U)                               
#define DMA_CCR_MINC_Msk                    (0x1U << DMA_CCR_MINC_Pos)         /*!< 0x00000080 */
#define DMA_CCR_MINC                        DMA_CCR_MINC_Msk                   /*!< Memory increment mode */

#define DMA_CCR_PSIZE_Pos                   (8U)                               
#define DMA_CCR_PSIZE_Msk                   (0x3U << DMA_CCR_PSIZE_Pos)        /*!< 0x00000300 */
#define DMA_CCR_PSIZE                       DMA_CCR_PSIZE_Msk                  /*!< PSIZE[1:0] bits (Peripheral size) */
#define DMA_CCR_PSIZE_0                     (0x1U << DMA_CCR_PSIZE_Pos)        /*!< 0x00000100 */
#define DMA_CCR_PSIZE_1                     (0x2U << DMA_CCR_PSIZE_Pos)        /*!< 0x00000200 */

#define DMA_CCR_MSIZE_Pos                   (10U)                              
#define DMA_CCR_MSIZE_Msk                   (0x3U << DMA_CCR_MSIZE_Pos)        /*!< 0x00000C00 */
#define DMA_CCR_MSIZE                       DMA_CCR_MSIZE_Msk                  /*!< MSIZE[1:0] bits (Memory size) */
#define DMA_CCR_MSIZE_0                     (0x1U << DMA_CCR_MSIZE_Pos)        /*!< 0x00000400 */
#define DMA_CCR_MSIZE_1                     (0x2U << DMA_CCR_MSIZE_Pos)        /*!< 0x00000800 */

#define DMA_CCR_PL_Pos                      (12U)                              
#define DMA_CCR_PL_Msk                      (0x3U << DMA_CCR_PL_Pos)           /*!< 0x00003000 */
#define DMA_CCR_PL                          DMA_CCR_PL_Msk                     /*!< PL[1:0] bits(Channel Priority level) */
#define DMA_CCR_PL_0                        (0x1U << DMA_CCR_PL_Pos)           /*!< 0x00001000 */
#define DMA_CCR_PL_1                        (0x2U << DMA_CCR_PL_Pos)           /*!< 0x00002000 */

#define DMA_CCR_MEM2MEM_Pos                 (14U)                              
#define DMA_CCR_MEM2MEM_Msk                 (0x1U << DMA_CCR_MEM2MEM_Pos)      /*!< 0x00004000 */
#define DMA_CCR_MEM2MEM                     DMA_CCR_MEM2MEM_Msk                /*!< Memory to memory mode */


/******************  Bit definition for DMA_TCNT1 register  ******************/
#define  DMA_TCNT1_CNT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_TCNT2 register  ******************/
#define  DMA_TCNT2_CNT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_TCNT3 register  ******************/
#define  DMA_TCNT3_CNT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_TCNT4 register  ******************/
#define  DMA_TCNT4_CNT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_TCNT5 register  ******************/
#define  DMA_TCNT5_CNT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_TCNT6 register  ******************/
#define  DMA_TCNT6_CNT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_TCNT7 register  ******************/
#define  DMA_TCNT7_CNT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CPBA1 register  *******************/
#define  DMA_CPBA1_PA                       ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPBA2 register  *******************/
#define  DMA_CPBA2_PA                       ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPBA3 register  *******************/
#define  DMA_CPBA3_PA                       ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */


/******************  Bit definition for DMA_CPBA4 register  *******************/
#define  DMA_CPBA4_PA                       ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPBA5 register  *******************/
#define  DMA_CPBA5_PA                       ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPBA6 register  *******************/
#define  DMA_CPBA6_PA                       ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */


/******************  Bit definition for DMA_CPBA7 register  *******************/
#define  DMA_CPBA7_PA                       ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CMBA1 register  *******************/
#define  DMA_CMBA1_MA                       ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMBA2 register  *******************/
#define  DMA_CMBA2_MA                       ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMBA3 register  *******************/
#define  DMA_CMBA3_MA                       ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */


/******************  Bit definition for DMA_CMBA4 register  *******************/
#define  DMA_CMBA4_MA                       ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMBA5 register  *******************/
#define  DMA_CMBA5_MA                       ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMBA6 register  *******************/
#define  DMA_CMBA6_MA                       ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMBA7 register  *******************/
#define  DMA_CMBA7_MA                       ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_STS register  ********************/
#define  ADC_STS_AWD                        ((uint8_t)0x01)               /*!< Analog watchdog flag */
#define  ADC_STS_EC                         ((uint8_t)0x02)               /*!< End of conversion */
#define  ADC_STS_JEC                        ((uint8_t)0x04)               /*!< Injected channel end of conversion */
#define  ADC_STS_JSTR                       ((uint8_t)0x08)               /*!< Injected channel Start flag */
#define  ADC_STS_RSTR                       ((uint8_t)0x10)               /*!< Regular channel Start flag */

/*******************  Bit definition for ADC_CTRL1 register  ********************/
#define  ADC_CTRL1_AWDCS                    ((uint32_t)0x0000001F)        /*!< AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CTRL1_AWDCS_0                  ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_CTRL1_AWDCS_1                  ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_CTRL1_AWDCS_2                  ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_CTRL1_AWDCS_3                  ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_CTRL1_AWDCS_4                  ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_CTRL1_ECIEN                    ((uint32_t)0x00000020)        /*!< Interrupt enable for EOC */
#define  ADC_CTRL1_AWDIEN                   ((uint32_t)0x00000040)        /*!< Analog Watchdog interrupt enable */
#define  ADC_CTRL1_JECIEN                   ((uint32_t)0x00000080)        /*!< Interrupt enable for injected channels */
#define  ADC_CTRL1_SCN                      ((uint32_t)0x00000100)        /*!< Scan mode */
#define  ADC_CTRL1_AWDSGE                   ((uint32_t)0x00000200)        /*!< Enable the watchdog on a single channel in scan mode */
#define  ADC_CTRL1_JAUT                     ((uint32_t)0x00000400)        /*!< Automatic injected group conversion */
#define  ADC_CTRL1_RDISEN                   ((uint32_t)0x00000800)        /*!< Discontinuous mode on regular channels */
#define  ADC_CTRL1_JDISEN                   ((uint32_t)0x00001000)        /*!< Discontinuous mode on injected channels */

#define  ADC_CTRL1_DISN                     ((uint32_t)0x0000E000)        /*!< DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_CTRL1_DISN_0                   ((uint32_t)0x00002000)        /*!< Bit 0 */
#define  ADC_CTRL1_DISN_1                   ((uint32_t)0x00004000)        /*!< Bit 1 */
#define  ADC_CTRL1_DISN_2                   ((uint32_t)0x00008000)        /*!< Bit 2 */

#define  ADC_CTRL1_DUALM                    ((uint32_t)0x000F0000)        /*!< DUALMOD[3:0] bits (Dual mode selection) */
#define  ADC_CTRL1_DUALM_0                  ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  ADC_CTRL1_DUALM_1                  ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  ADC_CTRL1_DUALM_2                  ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  ADC_CTRL1_DUALM_3                  ((uint32_t)0x00080000)        /*!< Bit 3 */

#define  ADC_CTRL1_JAWDEN                   ((uint32_t)0x00400000)        /*!< Analog watchdog enable on injected channels */
#define  ADC_CTRL1_AWDEN                    ((uint32_t)0x00800000)        /*!< Analog watchdog enable on regular channels */


/*******************  Bit definition for ADC_CTRL2 register  ********************/
#define  ADC_CTRL2_ADON                     ((uint32_t)0x00000001)        /*!< A/D Converter ON / OFF */
#define  ADC_CTRL2_CON                      ((uint32_t)0x00000002)        /*!< Continuous Conversion */
#define  ADC_CTRL2_CAL                      ((uint32_t)0x00000004)        /*!< A/D Calibration */
#define  ADC_CTRL2_RSTCAL                   ((uint32_t)0x00000008)        /*!< Reset Calibration */
#define  ADC_CTRL2_DMAEN                    ((uint32_t)0x00000100)        /*!< Direct Memory access mode */
#define  ADC_CTRL2_DALIGN                   ((uint32_t)0x00000800)        /*!< Data Alignment */

#define  ADC_CTRL2_JEXSEL                   ((uint32_t)0x01007000)        /*!< JEXTSEL[3:0] bits (External event select for injected group) */
#define  ADC_CTRL2_JEXSEL_0                 ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  ADC_CTRL2_JEXSEL_1                 ((uint32_t)0x00002000)        /*!< Bit 1 */
#define  ADC_CTRL2_JEXSEL_2                 ((uint32_t)0x00004000)        /*!< Bit 2 */
#define  ADC_CTRL2_JEXSEL_3                 ((uint32_t)0x01000000)        /*!< Bit 3 */

#define  ADC_CTRL2_JEXTREN                  ((uint32_t)0x00008000)        /*!< External Trigger Conversion mode for injected channels */

#define  ADC_CTRL2_EXSEL                    ((uint32_t)0x020E0000)        /*!< EXTSEL[3:0] bits (External Event Select for regular group) */
#define  ADC_CTRL2_EXSEL_0                  ((uint32_t)0x00020000)        /*!< Bit 0 */
#define  ADC_CTRL2_EXSEL_1                  ((uint32_t)0x00040000)        /*!< Bit 1 */
#define  ADC_CTRL2_EXSEL_2                  ((uint32_t)0x00080000)        /*!< Bit 2 */
#define  ADC_CTRL2_EXSEL_3                  ((uint32_t)0x02000000)        /*!< Bit 3 */

#define  ADC_CTRL2_EXTREN                   ((uint32_t)0x00100000)        /*!< External Trigger Conversion mode for regular channels */
#define  ADC_CTRL2_JSWSTR                   ((uint32_t)0x00200000)        /*!< Start Conversion of injected channels */
#define  ADC_CTRL2_SWSTR                    ((uint32_t)0x00400000)        /*!< Start Conversion of regular channels */
#define  ADC_CTRL2_TSREF                    ((uint32_t)0x00800000)        /*!< Temperature Sensor and VREFINT Enable */

/******************  Bit definition for ADC_SMPT1 register  *******************/
#define  ADC_SMPT1_SMP10                    ((uint32_t)0x00000007)        /*!< SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMPT1_SMP10_0                  ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_SMPT1_SMP10_1                  ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_SMPT1_SMP10_2                  ((uint32_t)0x00000004)        /*!< Bit 2 */

#define  ADC_SMPT1_SMP11                    ((uint32_t)0x00000038)        /*!< SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMPT1_SMP11_0                  ((uint32_t)0x00000008)        /*!< Bit 0 */
#define  ADC_SMPT1_SMP11_1                  ((uint32_t)0x00000010)        /*!< Bit 1 */
#define  ADC_SMPT1_SMP11_2                  ((uint32_t)0x00000020)        /*!< Bit 2 */

#define  ADC_SMPT1_SMP12                    ((uint32_t)0x000001C0)        /*!< SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMPT1_SMP12_0                  ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  ADC_SMPT1_SMP12_1                  ((uint32_t)0x00000080)        /*!< Bit 1 */
#define  ADC_SMPT1_SMP12_2                  ((uint32_t)0x00000100)        /*!< Bit 2 */

#define  ADC_SMPT1_SMP13                    ((uint32_t)0x00000E00)        /*!< SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMPT1_SMP13_0                  ((uint32_t)0x00000200)        /*!< Bit 0 */
#define  ADC_SMPT1_SMP13_1                  ((uint32_t)0x00000400)        /*!< Bit 1 */
#define  ADC_SMPT1_SMP13_2                  ((uint32_t)0x00000800)        /*!< Bit 2 */

#define  ADC_SMPT1_SMP14                    ((uint32_t)0x00007000)        /*!< SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMPT1_SMP14_0                  ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  ADC_SMPT1_SMP14_1                  ((uint32_t)0x00002000)        /*!< Bit 1 */
#define  ADC_SMPT1_SMP14_2                  ((uint32_t)0x00004000)        /*!< Bit 2 */

#define  ADC_SMPT1_SMP15                    ((uint32_t)0x00038000)        /*!< SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMPT1_SMP15_0                  ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_SMPT1_SMP15_1                  ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_SMPT1_SMP15_2                  ((uint32_t)0x00020000)        /*!< Bit 2 */

#define  ADC_SMPT1_SMP16                    ((uint32_t)0x001C0000)        /*!< SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMPT1_SMP16_0                  ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  ADC_SMPT1_SMP16_1                  ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  ADC_SMPT1_SMP16_2                  ((uint32_t)0x00100000)        /*!< Bit 2 */

#define  ADC_SMPT1_SMP17                    ((uint32_t)0x00E00000)        /*!< SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMPT1_SMP17_0                  ((uint32_t)0x00200000)        /*!< Bit 0 */
#define  ADC_SMPT1_SMP17_1                  ((uint32_t)0x00400000)        /*!< Bit 1 */
#define  ADC_SMPT1_SMP17_2                  ((uint32_t)0x00800000)        /*!< Bit 2 */

/******************  Bit definition for ADC_SMPT2 register  *******************/
#define  ADC_SMPT2_SMP0                     ((uint32_t)0x00000007)        /*!< SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMPT2_SMP0_0                   ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_SMPT2_SMP0_1                   ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_SMPT2_SMP0_2                   ((uint32_t)0x00000004)        /*!< Bit 2 */

#define  ADC_SMPT2_SMP1                     ((uint32_t)0x00000038)        /*!< SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMPT2_SMP1_0                   ((uint32_t)0x00000008)        /*!< Bit 0 */
#define  ADC_SMPT2_SMP1_1                   ((uint32_t)0x00000010)        /*!< Bit 1 */
#define  ADC_SMPT2_SMP1_2                   ((uint32_t)0x00000020)        /*!< Bit 2 */

#define  ADC_SMPT2_SMP2                     ((uint32_t)0x000001C0)        /*!< SMP2[2:0] bits (Channel 2 Sample time selection) */
#define  ADC_SMPT2_SMP2_0                   ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  ADC_SMPT2_SMP2_1                   ((uint32_t)0x00000080)        /*!< Bit 1 */
#define  ADC_SMPT2_SMP2_2                   ((uint32_t)0x00000100)        /*!< Bit 2 */

#define  ADC_SMPT2_SMP3                     ((uint32_t)0x00000E00)        /*!< SMP3[2:0] bits (Channel 3 Sample time selection) */
#define  ADC_SMPT2_SMP3_0                   ((uint32_t)0x00000200)        /*!< Bit 0 */
#define  ADC_SMPT2_SMP3_1                   ((uint32_t)0x00000400)        /*!< Bit 1 */
#define  ADC_SMPT2_SMP3_2                   ((uint32_t)0x00000800)        /*!< Bit 2 */

#define  ADC_SMPT2_SMP4                     ((uint32_t)0x00007000)        /*!< SMP4[2:0] bits (Channel 4 Sample time selection) */
#define  ADC_SMPT2_SMP4_0                   ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  ADC_SMPT2_SMP4_1                   ((uint32_t)0x00002000)        /*!< Bit 1 */
#define  ADC_SMPT2_SMP4_2                   ((uint32_t)0x00004000)        /*!< Bit 2 */

#define  ADC_SMPT2_SMP5                     ((uint32_t)0x00038000)        /*!< SMP5[2:0] bits (Channel 5 Sample time selection) */
#define  ADC_SMPT2_SMP5_0                   ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_SMPT2_SMP5_1                   ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_SMPT2_SMP5_2                   ((uint32_t)0x00020000)        /*!< Bit 2 */

#define  ADC_SMPT2_SMP6                     ((uint32_t)0x001C0000)        /*!< SMP6[2:0] bits (Channel 6 Sample time selection) */
#define  ADC_SMPT2_SMP6_0                   ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  ADC_SMPT2_SMP6_1                   ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  ADC_SMPT2_SMP6_2                   ((uint32_t)0x00100000)        /*!< Bit 2 */

#define  ADC_SMPT2_SMP7                     ((uint32_t)0x00E00000)        /*!< SMP7[2:0] bits (Channel 7 Sample time selection) */
#define  ADC_SMPT2_SMP7_0                   ((uint32_t)0x00200000)        /*!< Bit 0 */
#define  ADC_SMPT2_SMP7_1                   ((uint32_t)0x00400000)        /*!< Bit 1 */
#define  ADC_SMPT2_SMP7_2                   ((uint32_t)0x00800000)        /*!< Bit 2 */

#define  ADC_SMPT2_SMP8                     ((uint32_t)0x07000000)        /*!< SMP8[2:0] bits (Channel 8 Sample time selection) */
#define  ADC_SMPT2_SMP8_0                   ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  ADC_SMPT2_SMP8_1                   ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  ADC_SMPT2_SMP8_2                   ((uint32_t)0x04000000)        /*!< Bit 2 */

#define  ADC_SMPT2_SMP9                     ((uint32_t)0x38000000)        /*!< SMP9[2:0] bits (Channel 9 Sample time selection) */
#define  ADC_SMPT2_SMP9_0                   ((uint32_t)0x08000000)        /*!< Bit 0 */
#define  ADC_SMPT2_SMP9_1                   ((uint32_t)0x10000000)        /*!< Bit 1 */
#define  ADC_SMPT2_SMP9_2                   ((uint32_t)0x20000000)        /*!< Bit 2 */

/******************  Bit definition for ADC_JOFS1 register  *******************/
#define  ADC_JOFS1_JOFST1                   ((uint16_t)0x0FFF)            /*!< Data offset for injected channel 1 */

/******************  Bit definition for ADC_JOFS2 register  *******************/
#define  ADC_JOFS2_JOFST2                   ((uint16_t)0x0FFF)            /*!< Data offset for injected channel 2 */

/******************  Bit definition for ADC_JOFS3 register  *******************/
#define  ADC_JOFS3_JOFST3                   ((uint16_t)0x0FFF)            /*!< Data offset for injected channel 3 */

/******************  Bit definition for ADC_JOFS4 register  *******************/
#define  ADC_JOFS4_JOFST4                   ((uint16_t)0x0FFF)            /*!< Data offset for injected channel 4 */

/*******************  Bit definition for ADC_WHTR register  ********************/
#define  ADC_WHTR_AWHT                      ((uint16_t)0x0FFF)            /*!< Analog watchdog high threshold */

/*******************  Bit definition for ADC_WLTR register  ********************/
#define  ADC_WLTR_AWLT                      ((uint16_t)0x0FFF)            /*!< Analog watchdog low threshold */

/*******************  Bit definition for ADC_RSQ1 register  *******************/
#define  ADC_RSQ1_SQ13                      ((uint32_t)0x0000001F)        /*!< SQ13[4:0] bits (13th conversion in regular sequence) */
#define  ADC_RSQ1_SQ13_0                    ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_RSQ1_SQ13_1                    ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_RSQ1_SQ13_2                    ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_RSQ1_SQ13_3                    ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_RSQ1_SQ13_4                    ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_RSQ1_SQ14                      ((uint32_t)0x000003E0)        /*!< SQ14[4:0] bits (14th conversion in regular sequence) */
#define  ADC_RSQ1_SQ14_0                    ((uint32_t)0x00000020)        /*!< Bit 0 */
#define  ADC_RSQ1_SQ14_1                    ((uint32_t)0x00000040)        /*!< Bit 1 */
#define  ADC_RSQ1_SQ14_2                    ((uint32_t)0x00000080)        /*!< Bit 2 */
#define  ADC_RSQ1_SQ14_3                    ((uint32_t)0x00000100)        /*!< Bit 3 */
#define  ADC_RSQ1_SQ14_4                    ((uint32_t)0x00000200)        /*!< Bit 4 */

#define  ADC_RSQ1_SQ15                      ((uint32_t)0x00007C00)        /*!< SQ15[4:0] bits (15th conversion in regular sequence) */
#define  ADC_RSQ1_SQ15_0                    ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  ADC_RSQ1_SQ15_1                    ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  ADC_RSQ1_SQ15_2                    ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  ADC_RSQ1_SQ15_3                    ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  ADC_RSQ1_SQ15_4                    ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  ADC_RSQ1_SQ16                      ((uint32_t)0x000F8000)        /*!< SQ16[4:0] bits (16th conversion in regular sequence) */
#define  ADC_RSQ1_SQ16_0                    ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_RSQ1_SQ16_1                    ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_RSQ1_SQ16_2                    ((uint32_t)0x00020000)        /*!< Bit 2 */
#define  ADC_RSQ1_SQ16_3                    ((uint32_t)0x00040000)        /*!< Bit 3 */
#define  ADC_RSQ1_SQ16_4                    ((uint32_t)0x00080000)        /*!< Bit 4 */

#define  ADC_RSQ1_LEN                       ((uint32_t)0x00F00000)        /*!< L[3:0] bits (Regular channel sequence length) */
#define  ADC_RSQ1_LEN_0                     ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  ADC_RSQ1_LEN_1                     ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  ADC_RSQ1_LEN_2                     ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  ADC_RSQ1_LEN_3                     ((uint32_t)0x00800000)        /*!< Bit 3 */

/*******************  Bit definition for ADC_RSQ2 register  *******************/
#define  ADC_RSQ2_SQ7                       ((uint32_t)0x0000001F)        /*!< SQ7[4:0] bits (7th conversion in regular sequence) */
#define  ADC_RSQ2_SQ7_0                     ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_RSQ2_SQ7_1                     ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_RSQ2_SQ7_2                     ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_RSQ2_SQ7_3                     ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_RSQ2_SQ7_4                     ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_RSQ2_SQ8                       ((uint32_t)0x000003E0)        /*!< SQ8[4:0] bits (8th conversion in regular sequence) */
#define  ADC_RSQ2_SQ8_0                     ((uint32_t)0x00000020)        /*!< Bit 0 */
#define  ADC_RSQ2_SQ8_1                     ((uint32_t)0x00000040)        /*!< Bit 1 */
#define  ADC_RSQ2_SQ8_2                     ((uint32_t)0x00000080)        /*!< Bit 2 */
#define  ADC_RSQ2_SQ8_3                     ((uint32_t)0x00000100)        /*!< Bit 3 */
#define  ADC_RSQ2_SQ8_4                     ((uint32_t)0x00000200)        /*!< Bit 4 */

#define  ADC_RSQ2_SQ9                       ((uint32_t)0x00007C00)        /*!< SQ9[4:0] bits (9th conversion in regular sequence) */
#define  ADC_RSQ2_SQ9_0                     ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  ADC_RSQ2_SQ9_1                     ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  ADC_RSQ2_SQ9_2                     ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  ADC_RSQ2_SQ9_3                     ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  ADC_RSQ2_SQ9_4                     ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  ADC_RSQ2_SQ10                      ((uint32_t)0x000F8000)        /*!< SQ10[4:0] bits (10th conversion in regular sequence) */
#define  ADC_RSQ2_SQ10_0                    ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_RSQ2_SQ10_1                    ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_RSQ2_SQ10_2                    ((uint32_t)0x00020000)        /*!< Bit 2 */
#define  ADC_RSQ2_SQ10_3                    ((uint32_t)0x00040000)        /*!< Bit 3 */
#define  ADC_RSQ2_SQ10_4                    ((uint32_t)0x00080000)        /*!< Bit 4 */

#define  ADC_RSQ2_SQ11                      ((uint32_t)0x01F00000)        /*!< SQ11[4:0] bits (11th conversion in regular sequence) */
#define  ADC_RSQ2_SQ11_0                    ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  ADC_RSQ2_SQ11_1                    ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  ADC_RSQ2_SQ11_2                    ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  ADC_RSQ2_SQ11_3                    ((uint32_t)0x00800000)        /*!< Bit 3 */
#define  ADC_RSQ2_SQ11_4                    ((uint32_t)0x01000000)        /*!< Bit 4 */

#define  ADC_RSQ2_SQ12                      ((uint32_t)0x3E000000)        /*!< SQ12[4:0] bits (12th conversion in regular sequence) */
#define  ADC_RSQ2_SQ12_0                    ((uint32_t)0x02000000)        /*!< Bit 0 */
#define  ADC_RSQ2_SQ12_1                    ((uint32_t)0x04000000)        /*!< Bit 1 */
#define  ADC_RSQ2_SQ12_2                    ((uint32_t)0x08000000)        /*!< Bit 2 */
#define  ADC_RSQ2_SQ12_3                    ((uint32_t)0x10000000)        /*!< Bit 3 */
#define  ADC_RSQ2_SQ12_4                    ((uint32_t)0x20000000)        /*!< Bit 4 */

/*******************  Bit definition for ADC_RSQ3 register  *******************/
#define  ADC_RSQ3_SQ1                       ((uint32_t)0x0000001F)        /*!< SQ1[4:0] bits (1st conversion in regular sequence) */
#define  ADC_RSQ3_SQ1_0                     ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_RSQ3_SQ1_1                     ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_RSQ3_SQ1_2                     ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_RSQ3_SQ1_3                     ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_RSQ3_SQ1_4                     ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_RSQ3_SQ2                       ((uint32_t)0x000003E0)        /*!< SQ2[4:0] bits (2nd conversion in regular sequence) */
#define  ADC_RSQ3_SQ2_0                     ((uint32_t)0x00000020)        /*!< Bit 0 */
#define  ADC_RSQ3_SQ2_1                     ((uint32_t)0x00000040)        /*!< Bit 1 */
#define  ADC_RSQ3_SQ2_2                     ((uint32_t)0x00000080)        /*!< Bit 2 */
#define  ADC_RSQ3_SQ2_3                     ((uint32_t)0x00000100)        /*!< Bit 3 */
#define  ADC_RSQ3_SQ2_4                     ((uint32_t)0x00000200)        /*!< Bit 4 */

#define  ADC_RSQ3_SQ3                       ((uint32_t)0x00007C00)        /*!< SQ3[4:0] bits (3rd conversion in regular sequence) */
#define  ADC_RSQ3_SQ3_0                     ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  ADC_RSQ3_SQ3_1                     ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  ADC_RSQ3_SQ3_2                     ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  ADC_RSQ3_SQ3_3                     ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  ADC_RSQ3_SQ3_4                     ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  ADC_RSQ3_SQ4                       ((uint32_t)0x000F8000)        /*!< SQ4[4:0] bits (4th conversion in regular sequence) */
#define  ADC_RSQ3_SQ4_0                     ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_RSQ3_SQ4_1                     ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_RSQ3_SQ4_2                     ((uint32_t)0x00020000)        /*!< Bit 2 */
#define  ADC_RSQ3_SQ4_3                     ((uint32_t)0x00040000)        /*!< Bit 3 */
#define  ADC_RSQ3_SQ4_4                     ((uint32_t)0x00080000)        /*!< Bit 4 */

#define  ADC_RSQ3_SQ5                       ((uint32_t)0x01F00000)        /*!< SQ5[4:0] bits (5th conversion in regular sequence) */
#define  ADC_RSQ3_SQ5_0                     ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  ADC_RSQ3_SQ5_1                     ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  ADC_RSQ3_SQ5_2                     ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  ADC_RSQ3_SQ5_3                     ((uint32_t)0x00800000)        /*!< Bit 3 */
#define  ADC_RSQ3_SQ5_4                     ((uint32_t)0x01000000)        /*!< Bit 4 */

#define  ADC_RSQ3_SQ6                       ((uint32_t)0x3E000000)        /*!< SQ6[4:0] bits (6th conversion in regular sequence) */
#define  ADC_RSQ3_SQ6_0                     ((uint32_t)0x02000000)        /*!< Bit 0 */
#define  ADC_RSQ3_SQ6_1                     ((uint32_t)0x04000000)        /*!< Bit 1 */
#define  ADC_RSQ3_SQ6_2                     ((uint32_t)0x08000000)        /*!< Bit 2 */
#define  ADC_RSQ3_SQ6_3                     ((uint32_t)0x10000000)        /*!< Bit 3 */
#define  ADC_RSQ3_SQ6_4                     ((uint32_t)0x20000000)        /*!< Bit 4 */

/*******************  Bit definition for ADC_JSQ register  *******************/
#define  ADC_JSQ_JSQ1                       ((uint32_t)0x0000001F)        /*!< JSQ1[4:0] bits (1st conversion in injected sequence) */
#define  ADC_JSQ_JSQ1_0                     ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_JSQ_JSQ1_1                     ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_JSQ_JSQ1_2                     ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_JSQ_JSQ1_3                     ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_JSQ_JSQ1_4                     ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_JSQ_JSQ2                       ((uint32_t)0x000003E0)        /*!< JSQ2[4:0] bits (2nd conversion in injected sequence) */
#define  ADC_JSQ_JSQ2_0                     ((uint32_t)0x00000020)        /*!< Bit 0 */
#define  ADC_JSQ_JSQ2_1                     ((uint32_t)0x00000040)        /*!< Bit 1 */
#define  ADC_JSQ_JSQ2_2                     ((uint32_t)0x00000080)        /*!< Bit 2 */
#define  ADC_JSQ_JSQ2_3                     ((uint32_t)0x00000100)        /*!< Bit 3 */
#define  ADC_JSQ_JSQ2_4                     ((uint32_t)0x00000200)        /*!< Bit 4 */

#define  ADC_JSQ_JSQ3                       ((uint32_t)0x00007C00)        /*!< JSQ3[4:0] bits (3rd conversion in injected sequence) */
#define  ADC_JSQ_JSQ3_0                     ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  ADC_JSQ_JSQ3_1                     ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  ADC_JSQ_JSQ3_2                     ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  ADC_JSQ_JSQ3_3                     ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  ADC_JSQ_JSQ3_4                     ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  ADC_JSQ_JSQ4                       ((uint32_t)0x000F8000)        /*!< JSQ4[4:0] bits (4th conversion in injected sequence) */
#define  ADC_JSQ_JSQ4_0                     ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_JSQ_JSQ4_1                     ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_JSQ_JSQ4_2                     ((uint32_t)0x00020000)        /*!< Bit 2 */
#define  ADC_JSQ_JSQ4_3                     ((uint32_t)0x00040000)        /*!< Bit 3 */
#define  ADC_JSQ_JSQ4_4                     ((uint32_t)0x00080000)        /*!< Bit 4 */

#define  ADC_JSQ_JLEN                       ((uint32_t)0x00300000)        /*!< JL[1:0] bits (Injected Sequence length) */
#define  ADC_JSQ_JLEN_0                     ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  ADC_JSQ_JLEN_1                     ((uint32_t)0x00200000)        /*!< Bit 1 */

/*******************  Bit definition for ADC_JDOR1 register  *******************/
#define  ADC_JDOR1_JD                       ((uint16_t)0xFFFF)            /*!< Injected data */

/*******************  Bit definition for ADC_JDOR2 register  *******************/
#define  ADC_JDOR2_JD                       ((uint16_t)0xFFFF)            /*!< Injected data */

/*******************  Bit definition for ADC_JDOR3 register  *******************/
#define  ADC_JDOR3_JD                       ((uint16_t)0xFFFF)            /*!< Injected data */

/*******************  Bit definition for ADC_JDOR4 register  *******************/
#define  ADC_JDOR4_JD                       ((uint16_t)0xFFFF)            /*!< Injected data */

/********************  Bit definition for ADC_RDOR register  ********************/
#define  ADC_RDOR_D                         ((uint32_t)0x0000FFFF)        /*!< Regular data */
#define  ADC_RDOR_AD2D                      ((uint32_t)0xFFFF0000)        /*!< ADC2 data */

//copied from stm32 file

/********************  Bit definition for ADC_SR register  ********************/
#define ADC_SR_AWD_Pos                      (0U)                               
#define ADC_SR_AWD_Msk                      (0x1U << ADC_SR_AWD_Pos)           /*!< 0x00000001 */
#define ADC_SR_AWD                          ADC_SR_AWD_Msk                     /*!< ADC analog watchdog 1 flag */
#define ADC_SR_EOS_Pos                      (1U)                               
#define ADC_SR_EOS_Msk                      (0x1U << ADC_SR_EOS_Pos)           /*!< 0x00000002 */
#define ADC_SR_EOS                          ADC_SR_EOS_Msk                     /*!< ADC group regular end of sequence conversions flag */
#define ADC_SR_JEOS_Pos                     (2U)                               
#define ADC_SR_JEOS_Msk                     (0x1U << ADC_SR_JEOS_Pos)          /*!< 0x00000004 */
#define ADC_SR_JEOS                         ADC_SR_JEOS_Msk                    /*!< ADC group injected end of sequence conversions flag */
#define ADC_SR_JSTRT_Pos                    (3U)                               
#define ADC_SR_JSTRT_Msk                    (0x1U << ADC_SR_JSTRT_Pos)         /*!< 0x00000008 */
#define ADC_SR_JSTRT                        ADC_SR_JSTRT_Msk                   /*!< ADC group injected conversion start flag */
#define ADC_SR_STRT_Pos                     (4U)                               
#define ADC_SR_STRT_Msk                     (0x1U << ADC_SR_STRT_Pos)          /*!< 0x00000010 */
#define ADC_SR_STRT                         ADC_SR_STRT_Msk                    /*!< ADC group regular conversion start flag */

/* Legacy defines */
#define  ADC_SR_EOC                          (ADC_SR_EOS)
#define  ADC_SR_JEOC                         (ADC_SR_JEOS)

/*******************  Bit definition for ADC_CR1 register  ********************/
#define ADC_CR1_AWDCH_Pos                   (0U)                               
#define ADC_CR1_AWDCH_Msk                   (0x1FU << ADC_CR1_AWDCH_Pos)       /*!< 0x0000001F */
#define ADC_CR1_AWDCH                       ADC_CR1_AWDCH_Msk                  /*!< ADC analog watchdog 1 monitored channel selection */
#define ADC_CR1_AWDCH_0                     (0x01U << ADC_CR1_AWDCH_Pos)       /*!< 0x00000001 */
#define ADC_CR1_AWDCH_1                     (0x02U << ADC_CR1_AWDCH_Pos)       /*!< 0x00000002 */
#define ADC_CR1_AWDCH_2                     (0x04U << ADC_CR1_AWDCH_Pos)       /*!< 0x00000004 */
#define ADC_CR1_AWDCH_3                     (0x08U << ADC_CR1_AWDCH_Pos)       /*!< 0x00000008 */
#define ADC_CR1_AWDCH_4                     (0x10U << ADC_CR1_AWDCH_Pos)       /*!< 0x00000010 */

#define ADC_CR1_EOSIE_Pos                   (5U)                               
#define ADC_CR1_EOSIE_Msk                   (0x1U << ADC_CR1_EOSIE_Pos)        /*!< 0x00000020 */
#define ADC_CR1_EOSIE                       ADC_CR1_EOSIE_Msk                  /*!< ADC group regular end of sequence conversions interrupt */
#define ADC_CR1_AWDIE_Pos                   (6U)                               
#define ADC_CR1_AWDIE_Msk                   (0x1U << ADC_CR1_AWDIE_Pos)        /*!< 0x00000040 */
#define ADC_CR1_AWDIE                       ADC_CR1_AWDIE_Msk                  /*!< ADC analog watchdog 1 interrupt */
#define ADC_CR1_JEOSIE_Pos                  (7U)                               
#define ADC_CR1_JEOSIE_Msk                  (0x1U << ADC_CR1_JEOSIE_Pos)       /*!< 0x00000080 */
#define ADC_CR1_JEOSIE                      ADC_CR1_JEOSIE_Msk                 /*!< ADC group injected end of sequence conversions interrupt */
#define ADC_CR1_SCAN_Pos                    (8U)                               
#define ADC_CR1_SCAN_Msk                    (0x1U << ADC_CR1_SCAN_Pos)         /*!< 0x00000100 */
#define ADC_CR1_SCAN                        ADC_CR1_SCAN_Msk                   /*!< ADC scan mode */
#define ADC_CR1_AWDSGL_Pos                  (9U)                               
#define ADC_CR1_AWDSGL_Msk                  (0x1U << ADC_CR1_AWDSGL_Pos)       /*!< 0x00000200 */
#define ADC_CR1_AWDSGL                      ADC_CR1_AWDSGL_Msk                 /*!< ADC analog watchdog 1 monitoring a single channel or all channels */
#define ADC_CR1_JAUTO_Pos                   (10U)                              
#define ADC_CR1_JAUTO_Msk                   (0x1U << ADC_CR1_JAUTO_Pos)        /*!< 0x00000400 */
#define ADC_CR1_JAUTO                       ADC_CR1_JAUTO_Msk                  /*!< ADC group injected automatic trigger mode */
#define ADC_CR1_DISCEN_Pos                  (11U)                              
#define ADC_CR1_DISCEN_Msk                  (0x1U << ADC_CR1_DISCEN_Pos)       /*!< 0x00000800 */
#define ADC_CR1_DISCEN                      ADC_CR1_DISCEN_Msk                 /*!< ADC group regular sequencer discontinuous mode */
#define ADC_CR1_JDISCEN_Pos                 (12U)                              
#define ADC_CR1_JDISCEN_Msk                 (0x1U << ADC_CR1_JDISCEN_Pos)      /*!< 0x00001000 */
#define ADC_CR1_JDISCEN                     ADC_CR1_JDISCEN_Msk                /*!< ADC group injected sequencer discontinuous mode */

#define ADC_CR1_DISCNUM_Pos                 (13U)                              
#define ADC_CR1_DISCNUM_Msk                 (0x7U << ADC_CR1_DISCNUM_Pos)      /*!< 0x0000E000 */
#define ADC_CR1_DISCNUM                     ADC_CR1_DISCNUM_Msk                /*!< ADC group regular sequencer discontinuous number of ranks */
#define ADC_CR1_DISCNUM_0                   (0x1U << ADC_CR1_DISCNUM_Pos)      /*!< 0x00002000 */
#define ADC_CR1_DISCNUM_1                   (0x2U << ADC_CR1_DISCNUM_Pos)      /*!< 0x00004000 */
#define ADC_CR1_DISCNUM_2                   (0x4U << ADC_CR1_DISCNUM_Pos)      /*!< 0x00008000 */

#define ADC_CR1_DUALMOD_Pos                 (16U)                              
#define ADC_CR1_DUALMOD_Msk                 (0xFU << ADC_CR1_DUALMOD_Pos)      /*!< 0x000F0000 */
#define ADC_CR1_DUALMOD                     ADC_CR1_DUALMOD_Msk                /*!< ADC multimode mode selection */
#define ADC_CR1_DUALMOD_0                   (0x1U << ADC_CR1_DUALMOD_Pos)      /*!< 0x00010000 */
#define ADC_CR1_DUALMOD_1                   (0x2U << ADC_CR1_DUALMOD_Pos)      /*!< 0x00020000 */
#define ADC_CR1_DUALMOD_2                   (0x4U << ADC_CR1_DUALMOD_Pos)      /*!< 0x00040000 */
#define ADC_CR1_DUALMOD_3                   (0x8U << ADC_CR1_DUALMOD_Pos)      /*!< 0x00080000 */

#define ADC_CR1_JAWDEN_Pos                  (22U)                              
#define ADC_CR1_JAWDEN_Msk                  (0x1U << ADC_CR1_JAWDEN_Pos)       /*!< 0x00400000 */
#define ADC_CR1_JAWDEN                      ADC_CR1_JAWDEN_Msk                 /*!< ADC analog watchdog 1 enable on scope ADC group injected */
#define ADC_CR1_AWDEN_Pos                   (23U)                              
#define ADC_CR1_AWDEN_Msk                   (0x1U << ADC_CR1_AWDEN_Pos)        /*!< 0x00800000 */
#define ADC_CR1_AWDEN                       ADC_CR1_AWDEN_Msk                  /*!< ADC analog watchdog 1 enable on scope ADC group regular */

/* Legacy defines */
#define  ADC_CR1_EOCIE                       (ADC_CR1_EOSIE)
#define  ADC_CR1_JEOCIE                      (ADC_CR1_JEOSIE)

/*******************  Bit definition for ADC_CR2 register  ********************/
#define ADC_CR2_ADON_Pos                    (0U)                               
#define ADC_CR2_ADON_Msk                    (0x1U << ADC_CR2_ADON_Pos)         /*!< 0x00000001 */
#define ADC_CR2_ADON                        ADC_CR2_ADON_Msk                   /*!< ADC enable */
#define ADC_CR2_CONT_Pos                    (1U)                               
#define ADC_CR2_CONT_Msk                    (0x1U << ADC_CR2_CONT_Pos)         /*!< 0x00000002 */
#define ADC_CR2_CONT                        ADC_CR2_CONT_Msk                   /*!< ADC group regular continuous conversion mode */
#define ADC_CR2_CAL_Pos                     (2U)                               
#define ADC_CR2_CAL_Msk                     (0x1U << ADC_CR2_CAL_Pos)          /*!< 0x00000004 */
#define ADC_CR2_CAL                         ADC_CR2_CAL_Msk                    /*!< ADC calibration start */
#define ADC_CR2_RSTCAL_Pos                  (3U)                               
#define ADC_CR2_RSTCAL_Msk                  (0x1U << ADC_CR2_RSTCAL_Pos)       /*!< 0x00000008 */
#define ADC_CR2_RSTCAL                      ADC_CR2_RSTCAL_Msk                 /*!< ADC calibration reset */
#define ADC_CR2_DMA_Pos                     (8U)                               
#define ADC_CR2_DMA_Msk                     (0x1U << ADC_CR2_DMA_Pos)          /*!< 0x00000100 */
#define ADC_CR2_DMA                         ADC_CR2_DMA_Msk                    /*!< ADC DMA transfer enable */
#define ADC_CR2_ALIGN_Pos                   (11U)                              
#define ADC_CR2_ALIGN_Msk                   (0x1U << ADC_CR2_ALIGN_Pos)        /*!< 0x00000800 */
#define ADC_CR2_ALIGN                       ADC_CR2_ALIGN_Msk                  /*!< ADC data alignement */

#define ADC_CR2_JEXTSEL_Pos                 (12U)                              
#define ADC_CR2_JEXTSEL_Msk                 (0x7U << ADC_CR2_JEXTSEL_Pos)      /*!< 0x00007000 */
#define ADC_CR2_JEXTSEL                     ADC_CR2_JEXTSEL_Msk                /*!< ADC group injected external trigger source */
#define ADC_CR2_JEXTSEL_0                   (0x1U << ADC_CR2_JEXTSEL_Pos)      /*!< 0x00001000 */
#define ADC_CR2_JEXTSEL_1                   (0x2U << ADC_CR2_JEXTSEL_Pos)      /*!< 0x00002000 */
#define ADC_CR2_JEXTSEL_2                   (0x4U << ADC_CR2_JEXTSEL_Pos)      /*!< 0x00004000 */

#define ADC_CR2_JEXTTRIG_Pos                (15U)                              
#define ADC_CR2_JEXTTRIG_Msk                (0x1U << ADC_CR2_JEXTTRIG_Pos)     /*!< 0x00008000 */
#define ADC_CR2_JEXTTRIG                    ADC_CR2_JEXTTRIG_Msk               /*!< ADC group injected external trigger enable */

#define ADC_CR2_EXTSEL_Pos                  (17U)                              
#define ADC_CR2_EXTSEL_Msk                  (0x7U << ADC_CR2_EXTSEL_Pos)       /*!< 0x000E0000 */
#define ADC_CR2_EXTSEL                      ADC_CR2_EXTSEL_Msk                 /*!< ADC group regular external trigger source */
#define ADC_CR2_EXTSEL_0                    (0x1U << ADC_CR2_EXTSEL_Pos)       /*!< 0x00020000 */
#define ADC_CR2_EXTSEL_1                    (0x2U << ADC_CR2_EXTSEL_Pos)       /*!< 0x00040000 */
#define ADC_CR2_EXTSEL_2                    (0x4U << ADC_CR2_EXTSEL_Pos)       /*!< 0x00080000 */

#define ADC_CR2_EXTTRIG_Pos                 (20U)                              
#define ADC_CR2_EXTTRIG_Msk                 (0x1U << ADC_CR2_EXTTRIG_Pos)      /*!< 0x00100000 */
#define ADC_CR2_EXTTRIG                     ADC_CR2_EXTTRIG_Msk                /*!< ADC group regular external trigger enable */
#define ADC_CR2_JSWSTART_Pos                (21U)                              
#define ADC_CR2_JSWSTART_Msk                (0x1U << ADC_CR2_JSWSTART_Pos)     /*!< 0x00200000 */
#define ADC_CR2_JSWSTART                    ADC_CR2_JSWSTART_Msk               /*!< ADC group injected conversion start */
#define ADC_CR2_SWSTART_Pos                 (22U)                              
#define ADC_CR2_SWSTART_Msk                 (0x1U << ADC_CR2_SWSTART_Pos)      /*!< 0x00400000 */
#define ADC_CR2_SWSTART                     ADC_CR2_SWSTART_Msk                /*!< ADC group regular conversion start */
#define ADC_CR2_TSVREFE_Pos                 (23U)                              
#define ADC_CR2_TSVREFE_Msk                 (0x1U << ADC_CR2_TSVREFE_Pos)      /*!< 0x00800000 */
#define ADC_CR2_TSVREFE                     ADC_CR2_TSVREFE_Msk                /*!< ADC internal path to VrefInt and temperature sensor enable */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define ADC_SMPR1_SMP10_Pos                 (0U)                               
#define ADC_SMPR1_SMP10_Msk                 (0x7U << ADC_SMPR1_SMP10_Pos)      /*!< 0x00000007 */
#define ADC_SMPR1_SMP10                     ADC_SMPR1_SMP10_Msk                /*!< ADC channel 10 sampling time selection  */
#define ADC_SMPR1_SMP10_0                   (0x1U << ADC_SMPR1_SMP10_Pos)      /*!< 0x00000001 */
#define ADC_SMPR1_SMP10_1                   (0x2U << ADC_SMPR1_SMP10_Pos)      /*!< 0x00000002 */
#define ADC_SMPR1_SMP10_2                   (0x4U << ADC_SMPR1_SMP10_Pos)      /*!< 0x00000004 */

#define ADC_SMPR1_SMP11_Pos                 (3U)                               
#define ADC_SMPR1_SMP11_Msk                 (0x7U << ADC_SMPR1_SMP11_Pos)      /*!< 0x00000038 */
#define ADC_SMPR1_SMP11                     ADC_SMPR1_SMP11_Msk                /*!< ADC channel 11 sampling time selection  */
#define ADC_SMPR1_SMP11_0                   (0x1U << ADC_SMPR1_SMP11_Pos)      /*!< 0x00000008 */
#define ADC_SMPR1_SMP11_1                   (0x2U << ADC_SMPR1_SMP11_Pos)      /*!< 0x00000010 */
#define ADC_SMPR1_SMP11_2                   (0x4U << ADC_SMPR1_SMP11_Pos)      /*!< 0x00000020 */

#define ADC_SMPR1_SMP12_Pos                 (6U)                               
#define ADC_SMPR1_SMP12_Msk                 (0x7U << ADC_SMPR1_SMP12_Pos)      /*!< 0x000001C0 */
#define ADC_SMPR1_SMP12                     ADC_SMPR1_SMP12_Msk                /*!< ADC channel 12 sampling time selection  */
#define ADC_SMPR1_SMP12_0                   (0x1U << ADC_SMPR1_SMP12_Pos)      /*!< 0x00000040 */
#define ADC_SMPR1_SMP12_1                   (0x2U << ADC_SMPR1_SMP12_Pos)      /*!< 0x00000080 */
#define ADC_SMPR1_SMP12_2                   (0x4U << ADC_SMPR1_SMP12_Pos)      /*!< 0x00000100 */

#define ADC_SMPR1_SMP13_Pos                 (9U)                               
#define ADC_SMPR1_SMP13_Msk                 (0x7U << ADC_SMPR1_SMP13_Pos)      /*!< 0x00000E00 */
#define ADC_SMPR1_SMP13                     ADC_SMPR1_SMP13_Msk                /*!< ADC channel 13 sampling time selection  */
#define ADC_SMPR1_SMP13_0                   (0x1U << ADC_SMPR1_SMP13_Pos)      /*!< 0x00000200 */
#define ADC_SMPR1_SMP13_1                   (0x2U << ADC_SMPR1_SMP13_Pos)      /*!< 0x00000400 */
#define ADC_SMPR1_SMP13_2                   (0x4U << ADC_SMPR1_SMP13_Pos)      /*!< 0x00000800 */

#define ADC_SMPR1_SMP14_Pos                 (12U)                              
#define ADC_SMPR1_SMP14_Msk                 (0x7U << ADC_SMPR1_SMP14_Pos)      /*!< 0x00007000 */
#define ADC_SMPR1_SMP14                     ADC_SMPR1_SMP14_Msk                /*!< ADC channel 14 sampling time selection  */
#define ADC_SMPR1_SMP14_0                   (0x1U << ADC_SMPR1_SMP14_Pos)      /*!< 0x00001000 */
#define ADC_SMPR1_SMP14_1                   (0x2U << ADC_SMPR1_SMP14_Pos)      /*!< 0x00002000 */
#define ADC_SMPR1_SMP14_2                   (0x4U << ADC_SMPR1_SMP14_Pos)      /*!< 0x00004000 */

#define ADC_SMPR1_SMP15_Pos                 (15U)                              
#define ADC_SMPR1_SMP15_Msk                 (0x7U << ADC_SMPR1_SMP15_Pos)      /*!< 0x00038000 */
#define ADC_SMPR1_SMP15                     ADC_SMPR1_SMP15_Msk                /*!< ADC channel 15 sampling time selection  */
#define ADC_SMPR1_SMP15_0                   (0x1U << ADC_SMPR1_SMP15_Pos)      /*!< 0x00008000 */
#define ADC_SMPR1_SMP15_1                   (0x2U << ADC_SMPR1_SMP15_Pos)      /*!< 0x00010000 */
#define ADC_SMPR1_SMP15_2                   (0x4U << ADC_SMPR1_SMP15_Pos)      /*!< 0x00020000 */

#define ADC_SMPR1_SMP16_Pos                 (18U)                              
#define ADC_SMPR1_SMP16_Msk                 (0x7U << ADC_SMPR1_SMP16_Pos)      /*!< 0x001C0000 */
#define ADC_SMPR1_SMP16                     ADC_SMPR1_SMP16_Msk                /*!< ADC channel 16 sampling time selection  */
#define ADC_SMPR1_SMP16_0                   (0x1U << ADC_SMPR1_SMP16_Pos)      /*!< 0x00040000 */
#define ADC_SMPR1_SMP16_1                   (0x2U << ADC_SMPR1_SMP16_Pos)      /*!< 0x00080000 */
#define ADC_SMPR1_SMP16_2                   (0x4U << ADC_SMPR1_SMP16_Pos)      /*!< 0x00100000 */

#define ADC_SMPR1_SMP17_Pos                 (21U)                              
#define ADC_SMPR1_SMP17_Msk                 (0x7U << ADC_SMPR1_SMP17_Pos)      /*!< 0x00E00000 */
#define ADC_SMPR1_SMP17                     ADC_SMPR1_SMP17_Msk                /*!< ADC channel 17 sampling time selection  */
#define ADC_SMPR1_SMP17_0                   (0x1U << ADC_SMPR1_SMP17_Pos)      /*!< 0x00200000 */
#define ADC_SMPR1_SMP17_1                   (0x2U << ADC_SMPR1_SMP17_Pos)      /*!< 0x00400000 */
#define ADC_SMPR1_SMP17_2                   (0x4U << ADC_SMPR1_SMP17_Pos)      /*!< 0x00800000 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define ADC_SMPR2_SMP0_Pos                  (0U)                               
#define ADC_SMPR2_SMP0_Msk                  (0x7U << ADC_SMPR2_SMP0_Pos)       /*!< 0x00000007 */
#define ADC_SMPR2_SMP0                      ADC_SMPR2_SMP0_Msk                 /*!< ADC channel 0 sampling time selection  */
#define ADC_SMPR2_SMP0_0                    (0x1U << ADC_SMPR2_SMP0_Pos)       /*!< 0x00000001 */
#define ADC_SMPR2_SMP0_1                    (0x2U << ADC_SMPR2_SMP0_Pos)       /*!< 0x00000002 */
#define ADC_SMPR2_SMP0_2                    (0x4U << ADC_SMPR2_SMP0_Pos)       /*!< 0x00000004 */

#define ADC_SMPR2_SMP1_Pos                  (3U)                               
#define ADC_SMPR2_SMP1_Msk                  (0x7U << ADC_SMPR2_SMP1_Pos)       /*!< 0x00000038 */
#define ADC_SMPR2_SMP1                      ADC_SMPR2_SMP1_Msk                 /*!< ADC channel 1 sampling time selection  */
#define ADC_SMPR2_SMP1_0                    (0x1U << ADC_SMPR2_SMP1_Pos)       /*!< 0x00000008 */
#define ADC_SMPR2_SMP1_1                    (0x2U << ADC_SMPR2_SMP1_Pos)       /*!< 0x00000010 */
#define ADC_SMPR2_SMP1_2                    (0x4U << ADC_SMPR2_SMP1_Pos)       /*!< 0x00000020 */

#define ADC_SMPR2_SMP2_Pos                  (6U)                               
#define ADC_SMPR2_SMP2_Msk                  (0x7U << ADC_SMPR2_SMP2_Pos)       /*!< 0x000001C0 */
#define ADC_SMPR2_SMP2                      ADC_SMPR2_SMP2_Msk                 /*!< ADC channel 2 sampling time selection  */
#define ADC_SMPR2_SMP2_0                    (0x1U << ADC_SMPR2_SMP2_Pos)       /*!< 0x00000040 */
#define ADC_SMPR2_SMP2_1                    (0x2U << ADC_SMPR2_SMP2_Pos)       /*!< 0x00000080 */
#define ADC_SMPR2_SMP2_2                    (0x4U << ADC_SMPR2_SMP2_Pos)       /*!< 0x00000100 */

#define ADC_SMPR2_SMP3_Pos                  (9U)                               
#define ADC_SMPR2_SMP3_Msk                  (0x7U << ADC_SMPR2_SMP3_Pos)       /*!< 0x00000E00 */
#define ADC_SMPR2_SMP3                      ADC_SMPR2_SMP3_Msk                 /*!< ADC channel 3 sampling time selection  */
#define ADC_SMPR2_SMP3_0                    (0x1U << ADC_SMPR2_SMP3_Pos)       /*!< 0x00000200 */
#define ADC_SMPR2_SMP3_1                    (0x2U << ADC_SMPR2_SMP3_Pos)       /*!< 0x00000400 */
#define ADC_SMPR2_SMP3_2                    (0x4U << ADC_SMPR2_SMP3_Pos)       /*!< 0x00000800 */

#define ADC_SMPR2_SMP4_Pos                  (12U)                              
#define ADC_SMPR2_SMP4_Msk                  (0x7U << ADC_SMPR2_SMP4_Pos)       /*!< 0x00007000 */
#define ADC_SMPR2_SMP4                      ADC_SMPR2_SMP4_Msk                 /*!< ADC channel 4 sampling time selection  */
#define ADC_SMPR2_SMP4_0                    (0x1U << ADC_SMPR2_SMP4_Pos)       /*!< 0x00001000 */
#define ADC_SMPR2_SMP4_1                    (0x2U << ADC_SMPR2_SMP4_Pos)       /*!< 0x00002000 */
#define ADC_SMPR2_SMP4_2                    (0x4U << ADC_SMPR2_SMP4_Pos)       /*!< 0x00004000 */

#define ADC_SMPR2_SMP5_Pos                  (15U)                              
#define ADC_SMPR2_SMP5_Msk                  (0x7U << ADC_SMPR2_SMP5_Pos)       /*!< 0x00038000 */
#define ADC_SMPR2_SMP5                      ADC_SMPR2_SMP5_Msk                 /*!< ADC channel 5 sampling time selection  */
#define ADC_SMPR2_SMP5_0                    (0x1U << ADC_SMPR2_SMP5_Pos)       /*!< 0x00008000 */
#define ADC_SMPR2_SMP5_1                    (0x2U << ADC_SMPR2_SMP5_Pos)       /*!< 0x00010000 */
#define ADC_SMPR2_SMP5_2                    (0x4U << ADC_SMPR2_SMP5_Pos)       /*!< 0x00020000 */

#define ADC_SMPR2_SMP6_Pos                  (18U)                              
#define ADC_SMPR2_SMP6_Msk                  (0x7U << ADC_SMPR2_SMP6_Pos)       /*!< 0x001C0000 */
#define ADC_SMPR2_SMP6                      ADC_SMPR2_SMP6_Msk                 /*!< ADC channel 6 sampling time selection  */
#define ADC_SMPR2_SMP6_0                    (0x1U << ADC_SMPR2_SMP6_Pos)       /*!< 0x00040000 */
#define ADC_SMPR2_SMP6_1                    (0x2U << ADC_SMPR2_SMP6_Pos)       /*!< 0x00080000 */
#define ADC_SMPR2_SMP6_2                    (0x4U << ADC_SMPR2_SMP6_Pos)       /*!< 0x00100000 */

#define ADC_SMPR2_SMP7_Pos                  (21U)                              
#define ADC_SMPR2_SMP7_Msk                  (0x7U << ADC_SMPR2_SMP7_Pos)       /*!< 0x00E00000 */
#define ADC_SMPR2_SMP7                      ADC_SMPR2_SMP7_Msk                 /*!< ADC channel 7 sampling time selection  */
#define ADC_SMPR2_SMP7_0                    (0x1U << ADC_SMPR2_SMP7_Pos)       /*!< 0x00200000 */
#define ADC_SMPR2_SMP7_1                    (0x2U << ADC_SMPR2_SMP7_Pos)       /*!< 0x00400000 */
#define ADC_SMPR2_SMP7_2                    (0x4U << ADC_SMPR2_SMP7_Pos)       /*!< 0x00800000 */

#define ADC_SMPR2_SMP8_Pos                  (24U)                              
#define ADC_SMPR2_SMP8_Msk                  (0x7U << ADC_SMPR2_SMP8_Pos)       /*!< 0x07000000 */
#define ADC_SMPR2_SMP8                      ADC_SMPR2_SMP8_Msk                 /*!< ADC channel 8 sampling time selection  */
#define ADC_SMPR2_SMP8_0                    (0x1U << ADC_SMPR2_SMP8_Pos)       /*!< 0x01000000 */
#define ADC_SMPR2_SMP8_1                    (0x2U << ADC_SMPR2_SMP8_Pos)       /*!< 0x02000000 */
#define ADC_SMPR2_SMP8_2                    (0x4U << ADC_SMPR2_SMP8_Pos)       /*!< 0x04000000 */

#define ADC_SMPR2_SMP9_Pos                  (27U)                              
#define ADC_SMPR2_SMP9_Msk                  (0x7U << ADC_SMPR2_SMP9_Pos)       /*!< 0x38000000 */
#define ADC_SMPR2_SMP9                      ADC_SMPR2_SMP9_Msk                 /*!< ADC channel 9 sampling time selection  */
#define ADC_SMPR2_SMP9_0                    (0x1U << ADC_SMPR2_SMP9_Pos)       /*!< 0x08000000 */
#define ADC_SMPR2_SMP9_1                    (0x2U << ADC_SMPR2_SMP9_Pos)       /*!< 0x10000000 */
#define ADC_SMPR2_SMP9_2                    (0x4U << ADC_SMPR2_SMP9_Pos)       /*!< 0x20000000 */

/******************  Bit definition for ADC_JOFR1 register  *******************/
#define ADC_JOFR1_JOFFSET1_Pos              (0U)                               
#define ADC_JOFR1_JOFFSET1_Msk              (0xFFFU << ADC_JOFR1_JOFFSET1_Pos) /*!< 0x00000FFF */
#define ADC_JOFR1_JOFFSET1                  ADC_JOFR1_JOFFSET1_Msk             /*!< ADC group injected sequencer rank 1 offset value */

/******************  Bit definition for ADC_JOFR2 register  *******************/
#define ADC_JOFR2_JOFFSET2_Pos              (0U)                               
#define ADC_JOFR2_JOFFSET2_Msk              (0xFFFU << ADC_JOFR2_JOFFSET2_Pos) /*!< 0x00000FFF */
#define ADC_JOFR2_JOFFSET2                  ADC_JOFR2_JOFFSET2_Msk             /*!< ADC group injected sequencer rank 2 offset value */

/******************  Bit definition for ADC_JOFR3 register  *******************/
#define ADC_JOFR3_JOFFSET3_Pos              (0U)                               
#define ADC_JOFR3_JOFFSET3_Msk              (0xFFFU << ADC_JOFR3_JOFFSET3_Pos) /*!< 0x00000FFF */
#define ADC_JOFR3_JOFFSET3                  ADC_JOFR3_JOFFSET3_Msk             /*!< ADC group injected sequencer rank 3 offset value */

/******************  Bit definition for ADC_JOFR4 register  *******************/
#define ADC_JOFR4_JOFFSET4_Pos              (0U)                               
#define ADC_JOFR4_JOFFSET4_Msk              (0xFFFU << ADC_JOFR4_JOFFSET4_Pos) /*!< 0x00000FFF */
#define ADC_JOFR4_JOFFSET4                  ADC_JOFR4_JOFFSET4_Msk             /*!< ADC group injected sequencer rank 4 offset value */

/*******************  Bit definition for ADC_HTR register  ********************/
#define ADC_HTR_HT_Pos                      (0U)                               
#define ADC_HTR_HT_Msk                      (0xFFFU << ADC_HTR_HT_Pos)         /*!< 0x00000FFF */
#define ADC_HTR_HT                          ADC_HTR_HT_Msk                     /*!< ADC analog watchdog 1 threshold high */

/*******************  Bit definition for ADC_LTR register  ********************/
#define ADC_LTR_LT_Pos                      (0U)                               
#define ADC_LTR_LT_Msk                      (0xFFFU << ADC_LTR_LT_Pos)         /*!< 0x00000FFF */
#define ADC_LTR_LT                          ADC_LTR_LT_Msk                     /*!< ADC analog watchdog 1 threshold low */

/*******************  Bit definition for ADC_SQR1 register  *******************/
#define ADC_SQR1_SQ13_Pos                   (0U)                               
#define ADC_SQR1_SQ13_Msk                   (0x1FU << ADC_SQR1_SQ13_Pos)       /*!< 0x0000001F */
#define ADC_SQR1_SQ13                       ADC_SQR1_SQ13_Msk                  /*!< ADC group regular sequencer rank 13 */
#define ADC_SQR1_SQ13_0                     (0x01U << ADC_SQR1_SQ13_Pos)       /*!< 0x00000001 */
#define ADC_SQR1_SQ13_1                     (0x02U << ADC_SQR1_SQ13_Pos)       /*!< 0x00000002 */
#define ADC_SQR1_SQ13_2                     (0x04U << ADC_SQR1_SQ13_Pos)       /*!< 0x00000004 */
#define ADC_SQR1_SQ13_3                     (0x08U << ADC_SQR1_SQ13_Pos)       /*!< 0x00000008 */
#define ADC_SQR1_SQ13_4                     (0x10U << ADC_SQR1_SQ13_Pos)       /*!< 0x00000010 */

#define ADC_SQR1_SQ14_Pos                   (5U)                               
#define ADC_SQR1_SQ14_Msk                   (0x1FU << ADC_SQR1_SQ14_Pos)       /*!< 0x000003E0 */
#define ADC_SQR1_SQ14                       ADC_SQR1_SQ14_Msk                  /*!< ADC group regular sequencer rank 14 */
#define ADC_SQR1_SQ14_0                     (0x01U << ADC_SQR1_SQ14_Pos)       /*!< 0x00000020 */
#define ADC_SQR1_SQ14_1                     (0x02U << ADC_SQR1_SQ14_Pos)       /*!< 0x00000040 */
#define ADC_SQR1_SQ14_2                     (0x04U << ADC_SQR1_SQ14_Pos)       /*!< 0x00000080 */
#define ADC_SQR1_SQ14_3                     (0x08U << ADC_SQR1_SQ14_Pos)       /*!< 0x00000100 */
#define ADC_SQR1_SQ14_4                     (0x10U << ADC_SQR1_SQ14_Pos)       /*!< 0x00000200 */

#define ADC_SQR1_SQ15_Pos                   (10U)                              
#define ADC_SQR1_SQ15_Msk                   (0x1FU << ADC_SQR1_SQ15_Pos)       /*!< 0x00007C00 */
#define ADC_SQR1_SQ15                       ADC_SQR1_SQ15_Msk                  /*!< ADC group regular sequencer rank 15 */
#define ADC_SQR1_SQ15_0                     (0x01U << ADC_SQR1_SQ15_Pos)       /*!< 0x00000400 */
#define ADC_SQR1_SQ15_1                     (0x02U << ADC_SQR1_SQ15_Pos)       /*!< 0x00000800 */
#define ADC_SQR1_SQ15_2                     (0x04U << ADC_SQR1_SQ15_Pos)       /*!< 0x00001000 */
#define ADC_SQR1_SQ15_3                     (0x08U << ADC_SQR1_SQ15_Pos)       /*!< 0x00002000 */
#define ADC_SQR1_SQ15_4                     (0x10U << ADC_SQR1_SQ15_Pos)       /*!< 0x00004000 */

#define ADC_SQR1_SQ16_Pos                   (15U)                              
#define ADC_SQR1_SQ16_Msk                   (0x1FU << ADC_SQR1_SQ16_Pos)       /*!< 0x000F8000 */
#define ADC_SQR1_SQ16                       ADC_SQR1_SQ16_Msk                  /*!< ADC group regular sequencer rank 16 */
#define ADC_SQR1_SQ16_0                     (0x01U << ADC_SQR1_SQ16_Pos)       /*!< 0x00008000 */
#define ADC_SQR1_SQ16_1                     (0x02U << ADC_SQR1_SQ16_Pos)       /*!< 0x00010000 */
#define ADC_SQR1_SQ16_2                     (0x04U << ADC_SQR1_SQ16_Pos)       /*!< 0x00020000 */
#define ADC_SQR1_SQ16_3                     (0x08U << ADC_SQR1_SQ16_Pos)       /*!< 0x00040000 */
#define ADC_SQR1_SQ16_4                     (0x10U << ADC_SQR1_SQ16_Pos)       /*!< 0x00080000 */

#define ADC_SQR1_L_Pos                      (20U)                              
#define ADC_SQR1_L_Msk                      (0xFU << ADC_SQR1_L_Pos)           /*!< 0x00F00000 */
#define ADC_SQR1_L                          ADC_SQR1_L_Msk                     /*!< ADC group regular sequencer scan length */
#define ADC_SQR1_L_0                        (0x1U << ADC_SQR1_L_Pos)           /*!< 0x00100000 */
#define ADC_SQR1_L_1                        (0x2U << ADC_SQR1_L_Pos)           /*!< 0x00200000 */
#define ADC_SQR1_L_2                        (0x4U << ADC_SQR1_L_Pos)           /*!< 0x00400000 */
#define ADC_SQR1_L_3                        (0x8U << ADC_SQR1_L_Pos)           /*!< 0x00800000 */

/*******************  Bit definition for ADC_SQR2 register  *******************/
#define ADC_SQR2_SQ7_Pos                    (0U)                               
#define ADC_SQR2_SQ7_Msk                    (0x1FU << ADC_SQR2_SQ7_Pos)        /*!< 0x0000001F */
#define ADC_SQR2_SQ7                        ADC_SQR2_SQ7_Msk                   /*!< ADC group regular sequencer rank 7 */
#define ADC_SQR2_SQ7_0                      (0x01U << ADC_SQR2_SQ7_Pos)        /*!< 0x00000001 */
#define ADC_SQR2_SQ7_1                      (0x02U << ADC_SQR2_SQ7_Pos)        /*!< 0x00000002 */
#define ADC_SQR2_SQ7_2                      (0x04U << ADC_SQR2_SQ7_Pos)        /*!< 0x00000004 */
#define ADC_SQR2_SQ7_3                      (0x08U << ADC_SQR2_SQ7_Pos)        /*!< 0x00000008 */
#define ADC_SQR2_SQ7_4                      (0x10U << ADC_SQR2_SQ7_Pos)        /*!< 0x00000010 */

#define ADC_SQR2_SQ8_Pos                    (5U)                               
#define ADC_SQR2_SQ8_Msk                    (0x1FU << ADC_SQR2_SQ8_Pos)        /*!< 0x000003E0 */
#define ADC_SQR2_SQ8                        ADC_SQR2_SQ8_Msk                   /*!< ADC group regular sequencer rank 8 */
#define ADC_SQR2_SQ8_0                      (0x01U << ADC_SQR2_SQ8_Pos)        /*!< 0x00000020 */
#define ADC_SQR2_SQ8_1                      (0x02U << ADC_SQR2_SQ8_Pos)        /*!< 0x00000040 */
#define ADC_SQR2_SQ8_2                      (0x04U << ADC_SQR2_SQ8_Pos)        /*!< 0x00000080 */
#define ADC_SQR2_SQ8_3                      (0x08U << ADC_SQR2_SQ8_Pos)        /*!< 0x00000100 */
#define ADC_SQR2_SQ8_4                      (0x10U << ADC_SQR2_SQ8_Pos)        /*!< 0x00000200 */

#define ADC_SQR2_SQ9_Pos                    (10U)                              
#define ADC_SQR2_SQ9_Msk                    (0x1FU << ADC_SQR2_SQ9_Pos)        /*!< 0x00007C00 */
#define ADC_SQR2_SQ9                        ADC_SQR2_SQ9_Msk                   /*!< ADC group regular sequencer rank 9 */
#define ADC_SQR2_SQ9_0                      (0x01U << ADC_SQR2_SQ9_Pos)        /*!< 0x00000400 */
#define ADC_SQR2_SQ9_1                      (0x02U << ADC_SQR2_SQ9_Pos)        /*!< 0x00000800 */
#define ADC_SQR2_SQ9_2                      (0x04U << ADC_SQR2_SQ9_Pos)        /*!< 0x00001000 */
#define ADC_SQR2_SQ9_3                      (0x08U << ADC_SQR2_SQ9_Pos)        /*!< 0x00002000 */
#define ADC_SQR2_SQ9_4                      (0x10U << ADC_SQR2_SQ9_Pos)        /*!< 0x00004000 */

#define ADC_SQR2_SQ10_Pos                   (15U)                              
#define ADC_SQR2_SQ10_Msk                   (0x1FU << ADC_SQR2_SQ10_Pos)       /*!< 0x000F8000 */
#define ADC_SQR2_SQ10                       ADC_SQR2_SQ10_Msk                  /*!< ADC group regular sequencer rank 10 */
#define ADC_SQR2_SQ10_0                     (0x01U << ADC_SQR2_SQ10_Pos)       /*!< 0x00008000 */
#define ADC_SQR2_SQ10_1                     (0x02U << ADC_SQR2_SQ10_Pos)       /*!< 0x00010000 */
#define ADC_SQR2_SQ10_2                     (0x04U << ADC_SQR2_SQ10_Pos)       /*!< 0x00020000 */
#define ADC_SQR2_SQ10_3                     (0x08U << ADC_SQR2_SQ10_Pos)       /*!< 0x00040000 */
#define ADC_SQR2_SQ10_4                     (0x10U << ADC_SQR2_SQ10_Pos)       /*!< 0x00080000 */

#define ADC_SQR2_SQ11_Pos                   (20U)                              
#define ADC_SQR2_SQ11_Msk                   (0x1FU << ADC_SQR2_SQ11_Pos)       /*!< 0x01F00000 */
#define ADC_SQR2_SQ11                       ADC_SQR2_SQ11_Msk                  /*!< ADC group regular sequencer rank 1 */
#define ADC_SQR2_SQ11_0                     (0x01U << ADC_SQR2_SQ11_Pos)       /*!< 0x00100000 */
#define ADC_SQR2_SQ11_1                     (0x02U << ADC_SQR2_SQ11_Pos)       /*!< 0x00200000 */
#define ADC_SQR2_SQ11_2                     (0x04U << ADC_SQR2_SQ11_Pos)       /*!< 0x00400000 */
#define ADC_SQR2_SQ11_3                     (0x08U << ADC_SQR2_SQ11_Pos)       /*!< 0x00800000 */
#define ADC_SQR2_SQ11_4                     (0x10U << ADC_SQR2_SQ11_Pos)       /*!< 0x01000000 */

#define ADC_SQR2_SQ12_Pos                   (25U)                              
#define ADC_SQR2_SQ12_Msk                   (0x1FU << ADC_SQR2_SQ12_Pos)       /*!< 0x3E000000 */
#define ADC_SQR2_SQ12                       ADC_SQR2_SQ12_Msk                  /*!< ADC group regular sequencer rank 12 */
#define ADC_SQR2_SQ12_0                     (0x01U << ADC_SQR2_SQ12_Pos)       /*!< 0x02000000 */
#define ADC_SQR2_SQ12_1                     (0x02U << ADC_SQR2_SQ12_Pos)       /*!< 0x04000000 */
#define ADC_SQR2_SQ12_2                     (0x04U << ADC_SQR2_SQ12_Pos)       /*!< 0x08000000 */
#define ADC_SQR2_SQ12_3                     (0x08U << ADC_SQR2_SQ12_Pos)       /*!< 0x10000000 */
#define ADC_SQR2_SQ12_4                     (0x10U << ADC_SQR2_SQ12_Pos)       /*!< 0x20000000 */

/*******************  Bit definition for ADC_SQR3 register  *******************/
#define ADC_SQR3_SQ1_Pos                    (0U)                               
#define ADC_SQR3_SQ1_Msk                    (0x1FU << ADC_SQR3_SQ1_Pos)        /*!< 0x0000001F */
#define ADC_SQR3_SQ1                        ADC_SQR3_SQ1_Msk                   /*!< ADC group regular sequencer rank 1 */
#define ADC_SQR3_SQ1_0                      (0x01U << ADC_SQR3_SQ1_Pos)        /*!< 0x00000001 */
#define ADC_SQR3_SQ1_1                      (0x02U << ADC_SQR3_SQ1_Pos)        /*!< 0x00000002 */
#define ADC_SQR3_SQ1_2                      (0x04U << ADC_SQR3_SQ1_Pos)        /*!< 0x00000004 */
#define ADC_SQR3_SQ1_3                      (0x08U << ADC_SQR3_SQ1_Pos)        /*!< 0x00000008 */
#define ADC_SQR3_SQ1_4                      (0x10U << ADC_SQR3_SQ1_Pos)        /*!< 0x00000010 */

#define ADC_SQR3_SQ2_Pos                    (5U)                               
#define ADC_SQR3_SQ2_Msk                    (0x1FU << ADC_SQR3_SQ2_Pos)        /*!< 0x000003E0 */
#define ADC_SQR3_SQ2                        ADC_SQR3_SQ2_Msk                   /*!< ADC group regular sequencer rank 2 */
#define ADC_SQR3_SQ2_0                      (0x01U << ADC_SQR3_SQ2_Pos)        /*!< 0x00000020 */
#define ADC_SQR3_SQ2_1                      (0x02U << ADC_SQR3_SQ2_Pos)        /*!< 0x00000040 */
#define ADC_SQR3_SQ2_2                      (0x04U << ADC_SQR3_SQ2_Pos)        /*!< 0x00000080 */
#define ADC_SQR3_SQ2_3                      (0x08U << ADC_SQR3_SQ2_Pos)        /*!< 0x00000100 */
#define ADC_SQR3_SQ2_4                      (0x10U << ADC_SQR3_SQ2_Pos)        /*!< 0x00000200 */

#define ADC_SQR3_SQ3_Pos                    (10U)                              
#define ADC_SQR3_SQ3_Msk                    (0x1FU << ADC_SQR3_SQ3_Pos)        /*!< 0x00007C00 */
#define ADC_SQR3_SQ3                        ADC_SQR3_SQ3_Msk                   /*!< ADC group regular sequencer rank 3 */
#define ADC_SQR3_SQ3_0                      (0x01U << ADC_SQR3_SQ3_Pos)        /*!< 0x00000400 */
#define ADC_SQR3_SQ3_1                      (0x02U << ADC_SQR3_SQ3_Pos)        /*!< 0x00000800 */
#define ADC_SQR3_SQ3_2                      (0x04U << ADC_SQR3_SQ3_Pos)        /*!< 0x00001000 */
#define ADC_SQR3_SQ3_3                      (0x08U << ADC_SQR3_SQ3_Pos)        /*!< 0x00002000 */
#define ADC_SQR3_SQ3_4                      (0x10U << ADC_SQR3_SQ3_Pos)        /*!< 0x00004000 */

#define ADC_SQR3_SQ4_Pos                    (15U)                              
#define ADC_SQR3_SQ4_Msk                    (0x1FU << ADC_SQR3_SQ4_Pos)        /*!< 0x000F8000 */
#define ADC_SQR3_SQ4                        ADC_SQR3_SQ4_Msk                   /*!< ADC group regular sequencer rank 4 */
#define ADC_SQR3_SQ4_0                      (0x01U << ADC_SQR3_SQ4_Pos)        /*!< 0x00008000 */
#define ADC_SQR3_SQ4_1                      (0x02U << ADC_SQR3_SQ4_Pos)        /*!< 0x00010000 */
#define ADC_SQR3_SQ4_2                      (0x04U << ADC_SQR3_SQ4_Pos)        /*!< 0x00020000 */
#define ADC_SQR3_SQ4_3                      (0x08U << ADC_SQR3_SQ4_Pos)        /*!< 0x00040000 */
#define ADC_SQR3_SQ4_4                      (0x10U << ADC_SQR3_SQ4_Pos)        /*!< 0x00080000 */

#define ADC_SQR3_SQ5_Pos                    (20U)                              
#define ADC_SQR3_SQ5_Msk                    (0x1FU << ADC_SQR3_SQ5_Pos)        /*!< 0x01F00000 */
#define ADC_SQR3_SQ5                        ADC_SQR3_SQ5_Msk                   /*!< ADC group regular sequencer rank 5 */
#define ADC_SQR3_SQ5_0                      (0x01U << ADC_SQR3_SQ5_Pos)        /*!< 0x00100000 */
#define ADC_SQR3_SQ5_1                      (0x02U << ADC_SQR3_SQ5_Pos)        /*!< 0x00200000 */
#define ADC_SQR3_SQ5_2                      (0x04U << ADC_SQR3_SQ5_Pos)        /*!< 0x00400000 */
#define ADC_SQR3_SQ5_3                      (0x08U << ADC_SQR3_SQ5_Pos)        /*!< 0x00800000 */
#define ADC_SQR3_SQ5_4                      (0x10U << ADC_SQR3_SQ5_Pos)        /*!< 0x01000000 */

#define ADC_SQR3_SQ6_Pos                    (25U)                              
#define ADC_SQR3_SQ6_Msk                    (0x1FU << ADC_SQR3_SQ6_Pos)        /*!< 0x3E000000 */
#define ADC_SQR3_SQ6                        ADC_SQR3_SQ6_Msk                   /*!< ADC group regular sequencer rank 6 */
#define ADC_SQR3_SQ6_0                      (0x01U << ADC_SQR3_SQ6_Pos)        /*!< 0x02000000 */
#define ADC_SQR3_SQ6_1                      (0x02U << ADC_SQR3_SQ6_Pos)        /*!< 0x04000000 */
#define ADC_SQR3_SQ6_2                      (0x04U << ADC_SQR3_SQ6_Pos)        /*!< 0x08000000 */
#define ADC_SQR3_SQ6_3                      (0x08U << ADC_SQR3_SQ6_Pos)        /*!< 0x10000000 */
#define ADC_SQR3_SQ6_4                      (0x10U << ADC_SQR3_SQ6_Pos)        /*!< 0x20000000 */

/*******************  Bit definition for ADC_JSQR register  *******************/
#define ADC_JSQR_JSQ1_Pos                   (0U)                               
#define ADC_JSQR_JSQ1_Msk                   (0x1FU << ADC_JSQR_JSQ1_Pos)       /*!< 0x0000001F */
#define ADC_JSQR_JSQ1                       ADC_JSQR_JSQ1_Msk                  /*!< ADC group injected sequencer rank 1 */
#define ADC_JSQR_JSQ1_0                     (0x01U << ADC_JSQR_JSQ1_Pos)       /*!< 0x00000001 */
#define ADC_JSQR_JSQ1_1                     (0x02U << ADC_JSQR_JSQ1_Pos)       /*!< 0x00000002 */
#define ADC_JSQR_JSQ1_2                     (0x04U << ADC_JSQR_JSQ1_Pos)       /*!< 0x00000004 */
#define ADC_JSQR_JSQ1_3                     (0x08U << ADC_JSQR_JSQ1_Pos)       /*!< 0x00000008 */
#define ADC_JSQR_JSQ1_4                     (0x10U << ADC_JSQR_JSQ1_Pos)       /*!< 0x00000010 */

#define ADC_JSQR_JSQ2_Pos                   (5U)                               
#define ADC_JSQR_JSQ2_Msk                   (0x1FU << ADC_JSQR_JSQ2_Pos)       /*!< 0x000003E0 */
#define ADC_JSQR_JSQ2                       ADC_JSQR_JSQ2_Msk                  /*!< ADC group injected sequencer rank 2 */
#define ADC_JSQR_JSQ2_0                     (0x01U << ADC_JSQR_JSQ2_Pos)       /*!< 0x00000020 */
#define ADC_JSQR_JSQ2_1                     (0x02U << ADC_JSQR_JSQ2_Pos)       /*!< 0x00000040 */
#define ADC_JSQR_JSQ2_2                     (0x04U << ADC_JSQR_JSQ2_Pos)       /*!< 0x00000080 */
#define ADC_JSQR_JSQ2_3                     (0x08U << ADC_JSQR_JSQ2_Pos)       /*!< 0x00000100 */
#define ADC_JSQR_JSQ2_4                     (0x10U << ADC_JSQR_JSQ2_Pos)       /*!< 0x00000200 */

#define ADC_JSQR_JSQ3_Pos                   (10U)                              
#define ADC_JSQR_JSQ3_Msk                   (0x1FU << ADC_JSQR_JSQ3_Pos)       /*!< 0x00007C00 */
#define ADC_JSQR_JSQ3                       ADC_JSQR_JSQ3_Msk                  /*!< ADC group injected sequencer rank 3 */
#define ADC_JSQR_JSQ3_0                     (0x01U << ADC_JSQR_JSQ3_Pos)       /*!< 0x00000400 */
#define ADC_JSQR_JSQ3_1                     (0x02U << ADC_JSQR_JSQ3_Pos)       /*!< 0x00000800 */
#define ADC_JSQR_JSQ3_2                     (0x04U << ADC_JSQR_JSQ3_Pos)       /*!< 0x00001000 */
#define ADC_JSQR_JSQ3_3                     (0x08U << ADC_JSQR_JSQ3_Pos)       /*!< 0x00002000 */
#define ADC_JSQR_JSQ3_4                     (0x10U << ADC_JSQR_JSQ3_Pos)       /*!< 0x00004000 */

#define ADC_JSQR_JSQ4_Pos                   (15U)                              
#define ADC_JSQR_JSQ4_Msk                   (0x1FU << ADC_JSQR_JSQ4_Pos)       /*!< 0x000F8000 */
#define ADC_JSQR_JSQ4                       ADC_JSQR_JSQ4_Msk                  /*!< ADC group injected sequencer rank 4 */
#define ADC_JSQR_JSQ4_0                     (0x01U << ADC_JSQR_JSQ4_Pos)       /*!< 0x00008000 */
#define ADC_JSQR_JSQ4_1                     (0x02U << ADC_JSQR_JSQ4_Pos)       /*!< 0x00010000 */
#define ADC_JSQR_JSQ4_2                     (0x04U << ADC_JSQR_JSQ4_Pos)       /*!< 0x00020000 */
#define ADC_JSQR_JSQ4_3                     (0x08U << ADC_JSQR_JSQ4_Pos)       /*!< 0x00040000 */
#define ADC_JSQR_JSQ4_4                     (0x10U << ADC_JSQR_JSQ4_Pos)       /*!< 0x00080000 */

#define ADC_JSQR_JL_Pos                     (20U)                              
#define ADC_JSQR_JL_Msk                     (0x3U << ADC_JSQR_JL_Pos)          /*!< 0x00300000 */
#define ADC_JSQR_JL                         ADC_JSQR_JL_Msk                    /*!< ADC group injected sequencer scan length */
#define ADC_JSQR_JL_0                       (0x1U << ADC_JSQR_JL_Pos)          /*!< 0x00100000 */
#define ADC_JSQR_JL_1                       (0x2U << ADC_JSQR_JL_Pos)          /*!< 0x00200000 */

/*******************  Bit definition for ADC_JDR1 register  *******************/
#define ADC_JDR1_JDATA_Pos                  (0U)                               
#define ADC_JDR1_JDATA_Msk                  (0xFFFFU << ADC_JDR1_JDATA_Pos)    /*!< 0x0000FFFF */
#define ADC_JDR1_JDATA                      ADC_JDR1_JDATA_Msk                 /*!< ADC group injected sequencer rank 1 conversion data */

/*******************  Bit definition for ADC_JDR2 register  *******************/
#define ADC_JDR2_JDATA_Pos                  (0U)                               
#define ADC_JDR2_JDATA_Msk                  (0xFFFFU << ADC_JDR2_JDATA_Pos)    /*!< 0x0000FFFF */
#define ADC_JDR2_JDATA                      ADC_JDR2_JDATA_Msk                 /*!< ADC group injected sequencer rank 2 conversion data */

/*******************  Bit definition for ADC_JDR3 register  *******************/
#define ADC_JDR3_JDATA_Pos                  (0U)                               
#define ADC_JDR3_JDATA_Msk                  (0xFFFFU << ADC_JDR3_JDATA_Pos)    /*!< 0x0000FFFF */
#define ADC_JDR3_JDATA                      ADC_JDR3_JDATA_Msk                 /*!< ADC group injected sequencer rank 3 conversion data */

/*******************  Bit definition for ADC_JDR4 register  *******************/
#define ADC_JDR4_JDATA_Pos                  (0U)                               
#define ADC_JDR4_JDATA_Msk                  (0xFFFFU << ADC_JDR4_JDATA_Pos)    /*!< 0x0000FFFF */
#define ADC_JDR4_JDATA                      ADC_JDR4_JDATA_Msk                 /*!< ADC group injected sequencer rank 4 conversion data */

/********************  Bit definition for ADC_DR register  ********************/
#define ADC_DR_DATA_Pos                     (0U)                               
#define ADC_DR_DATA_Msk                     (0xFFFFU << ADC_DR_DATA_Pos)       /*!< 0x0000FFFF */
#define ADC_DR_DATA                         ADC_DR_DATA_Msk                    /*!< ADC group regular conversion data */
#define ADC_DR_ADC2DATA_Pos                 (16U)                              
#define ADC_DR_ADC2DATA_Msk                 (0xFFFFU << ADC_DR_ADC2DATA_Pos)   /*!< 0xFFFF0000 */
#define ADC_DR_ADC2DATA                     ADC_DR_ADC2DATA_Msk                /*!< ADC group regular conversion data for ADC slave, in multimode */


/******************************************************************************/
/*                                                                            */
/*                      Digital to Analog Converter                           */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for DAC_CTRL register  ********************/
#define  DAC_CTRL_EN1                       ((uint32_t)0x00000001)        /*!< DAC channel1 enable */
#define  DAC_CTRL_BF1                       ((uint32_t)0x00000002)        /*!< DAC channel1 output buffer disable */
#define  DAC_CTRL_TEN1                      ((uint32_t)0x00000004)        /*!< DAC channel1 Trigger enable */

#define  DAC_CTRL_TGSL1                     ((uint32_t)0x00000038)        /*!< TSEL1[2:0] (DAC channel1 Trigger selection) */
#define  DAC_CTRL_TGSL1_0                   ((uint32_t)0x00000008)        /*!< Bit 0 */
#define  DAC_CTRL_TGSL1_1                   ((uint32_t)0x00000010)        /*!< Bit 1 */
#define  DAC_CTRL_TGSL1_2                   ((uint32_t)0x00000020)        /*!< Bit 2 */

#define  DAC_CTRL_WAVE1                     ((uint32_t)0x000000C0)        /*!< WAVE1[1:0] (DAC channel1 noise/triangle wave generation enable) */
#define  DAC_CTRL_WAVE1_0                   ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  DAC_CTRL_WAVE1_1                   ((uint32_t)0x00000080)        /*!< Bit 1 */

#define  DAC_CTRL_MAMS1                     ((uint32_t)0x00000F00)        /*!< MAMP1[3:0] (DAC channel1 Mask/Amplitude selector) */
#define  DAC_CTRL_MAMS1_0                   ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  DAC_CTRL_MAMS1_1                   ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  DAC_CTRL_MAMS1_2                   ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  DAC_CTRL_MAMS1_3                   ((uint32_t)0x00000800)        /*!< Bit 3 */

#define  DAC_CTRL_DMAEN1                    ((uint32_t)0x00001000)        /*!< DAC channel1 DMA enable */
#define  DAC_CTRL_EN2                       ((uint32_t)0x00010000)        /*!< DAC channel2 enable */
#define  DAC_CTRL_BF2                       ((uint32_t)0x00020000)        /*!< DAC channel2 output buffer disable */
#define  DAC_CTRL_TEN2                      ((uint32_t)0x00040000)        /*!< DAC channel2 Trigger enable */

#define  DAC_CTRL_TGSL2                     ((uint32_t)0x00380000)        /*!< TSEL2[2:0] (DAC channel2 Trigger selection) */
#define  DAC_CTRL_TGSL2_0                   ((uint32_t)0x00080000)        /*!< Bit 0 */
#define  DAC_CTRL_TGSL2_1                   ((uint32_t)0x00100000)        /*!< Bit 1 */
#define  DAC_CTRL_TGSL2_2                   ((uint32_t)0x00200000)        /*!< Bit 2 */

#define  DAC_CTRL_WAVE2                     ((uint32_t)0x00C00000)        /*!< WAVE2[1:0] (DAC channel2 noise/triangle wave generation enable) */
#define  DAC_CTRL_WAVE2_0                   ((uint32_t)0x00400000)        /*!< Bit 0 */
#define  DAC_CTRL_WAVE2_1                   ((uint32_t)0x00800000)        /*!< Bit 1 */

#define  DAC_CTRL_MAMS2                     ((uint32_t)0x0F000000)        /*!< MAMP2[3:0] (DAC channel2 Mask/Amplitude selector) */
#define  DAC_CTRL_MAMS2_0                   ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  DAC_CTRL_MAMS2_1                   ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  DAC_CTRL_MAMS2_2                   ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  DAC_CTRL_MAMS2_3                   ((uint32_t)0x08000000)        /*!< Bit 3 */

#define  DAC_CTRL_DMAEN2                    ((uint32_t)0x10000000)        /*!< DAC channel2 DMA enabled */

/*****************  Bit definition for DAC_SWTRG register  ******************/
#define  DAC_SWTRG_SWTRG1                   ((uint8_t)0x01)               /*!< DAC channel1 software trigger */
#define  DAC_SWTRG_SWTRG2                   ((uint8_t)0x02)               /*!< DAC channel2 software trigger */

/*****************  Bit definition for DAC_HDR12R1 register  ******************/
#define  DAC_HDR12R1_D1HDR                  ((uint16_t)0x0FFF)            /*!< DAC channel1 12-bit Right aligned data */

/*****************  Bit definition for DAC_HDR12L1register  ******************/
#define  DAC_HDR12L1_D1HDR                  ((uint16_t)0xFFF0)            /*!< DAC channel1 12-bit Left aligned data */

/******************  Bit definition for DAC_HDR8R1 register  ******************/
#define  DAC_HDR8R1_D1HDR                   ((uint8_t)0xFF)               /*!< DAC channel1 8-bit Right aligned data */

/*****************  Bit definition for DAC_HDR12R2 register  ******************/
#define  DAC_HDR12R2_D2HDR                  ((uint16_t)0x0FFF)            /*!< DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_HDR12L2 register  ******************/
#define  DAC_HDR12L2_D2HDR                  ((uint16_t)0xFFF0)            /*!< DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_HDR8R2 register  ******************/
#define  DAC_HDR8R2_D2HDR                   ((uint8_t)0xFF)               /*!< DAC channel2 8-bit Right aligned data */

/*****************  Bit definition for DAC_HDR12RD register  ******************/
#define  DAC_HDR12RD_D1HDR                  ((uint32_t)0x00000FFF)        /*!< DAC channel1 12-bit Right aligned data */
#define  DAC_HDR12RD_D2HDR                  ((uint32_t)0x0FFF0000)        /*!< DAC channel2 12-bit Right aligned data */

/*****************  Bit definition for DAC_HDR12LD register  ******************/
#define  DAC_HDR12LD_D1HDR                  ((uint32_t)0x0000FFF0)        /*!< DAC channel1 12-bit Left aligned data */
#define  DAC_HDR12LD_D2HDR                  ((uint32_t)0xFFF00000)        /*!< DAC channel2 12-bit Left aligned data */

/******************  Bit definition for DAC_HDR8RD register  ******************/
#define  DAC_HDR8RD_D1HDR                   ((uint16_t)0x00FF)            /*!< DAC channel1 8-bit Right aligned data */
#define  DAC_HDR8RD_D2HDR                   ((uint16_t)0xFF00)            /*!< DAC channel2 8-bit Right aligned data */

/*******************  Bit definition for DAC_ODT1 register  *******************/
#define  DAC_ODT1_D1ODT                     ((uint16_t)0x0FFF)            /*!< DAC channel1 data output */

/*******************  Bit definition for DAC_ODT2 register  *******************/
#define  DAC_ODT2_D2ODT                     ((uint16_t)0x0FFF)            /*!< DAC channel2 data output */

/********************  Bit definition for DAC_STS register  ********************/
#define  DAC_STS_DMAUDR1                    ((uint32_t)0x00002000)        /*!< DAC channel1 DMA underrun flag */
#define  DAC_STS_DMAUDR2                    ((uint32_t)0x20000000)        /*!< DAC channel2 DMA underrun flag */

/*****************************************************************************/
/*                                                                           */
/*                               Timers (TMR)                                */
/*                                                                           */
/*****************************************************************************/
/*******************  Bit definition for TMR_CTRL1 register  ********************/
#define  TMR_CTRL1_CNTEN                    ((uint16_t)0x0001)            /*!< Counter enable */
#define  TMR_CTRL1_UEVDIS                   ((uint16_t)0x0002)            /*!< Update disable */
#define  TMR_CTRL1_UVERS                    ((uint16_t)0x0004)            /*!< Update request source */
#define  TMR_CTRL1_OPMODE                   ((uint16_t)0x0008)            /*!< One pulse mode */
#define  TMR_CTRL1_DIR                      ((uint16_t)0x0010)            /*!< Direction */

#define  TMR_CTRL1_CMSEL                    ((uint16_t)0x0060)            /*!< CMS[1:0] bits (Center-aligned mode selection) */
#define  TMR_CTRL1_CMSEL_0                  ((uint16_t)0x0020)            /*!< Bit 0 */
#define  TMR_CTRL1_CMSEL_1                  ((uint16_t)0x0040)            /*!< Bit 1 */

#define  TMR_CTRL1_ARPEN                    ((uint16_t)0x0080)            /*!< Auto-reload preload enable */

#define  TMR_CTRL1_CLKDIV                   ((uint16_t)0x0300)            /*!< CKD[1:0] bits (clock division) */
#define  TMR_CTRL1_CLKDIV_0                 ((uint16_t)0x0100)            /*!< Bit 0 */
#define  TMR_CTRL1_CLKDIV_1                 ((uint16_t)0x0200)            /*!< Bit 1 */

#define  TMR_CTRL1_PMEN                     ((uint16_t)0x0400)            /*!< 32bit counter enable */

/*******************  Bit definition for TMR_CTRL2 register  ********************/
#define  TMR_CTRL2_CPC                      ((uint16_t)0x0001)            /*!< Capture/Compare Preloaded Control */
#define  TMR_CTRL2_CUSEL                    ((uint16_t)0x0004)            /*!< Capture/Compare Control Update Selection */
#define  TMR_CTRL2_CDSEL                    ((uint16_t)0x0008)            /*!< Capture/Compare DMA Selection */

#define  TMR_CTRL2_MMSEL                    ((uint16_t)0x0070)            /*!< MMS[2:0] bits (Master Mode Selection) */
#define  TMR_CTRL2_MMSEL_0                  ((uint16_t)0x0010)            /*!< Bit 0 */
#define  TMR_CTRL2_MMSEL_1                  ((uint16_t)0x0020)            /*!< Bit 1 */
#define  TMR_CTRL2_MMSEL_2                  ((uint16_t)0x0040)            /*!< Bit 2 */

#define  TMR_CTRL2_TI1SEL                   ((uint16_t)0x0080)            /*!< TI1 Selection */
#define  TMR_CTRL2_OC1IS                    ((uint16_t)0x0100)            /*!< Output Idle state 1 (OC1 output) */
#define  TMR_CTRL2_OC1NIS                   ((uint16_t)0x0200)            /*!< Output Idle state 1 (OC1N output) */
#define  TMR_CTRL2_OC2IS                    ((uint16_t)0x0400)            /*!< Output Idle state 2 (OC2 output) */
#define  TMR_CTRL2_OC2NIS                   ((uint16_t)0x0800)            /*!< Output Idle state 2 (OC2N output) */
#define  TMR_CTRL2_OC3IS                    ((uint16_t)0x1000)            /*!< Output Idle state 3 (OC3 output) */
#define  TMR_CTRL2_OC3NIS                   ((uint16_t)0x2000)            /*!< Output Idle state 3 (OC3N output) */
#define  TMR_CTRL2_OC4IS                    ((uint16_t)0x4000)            /*!< Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TMR_SMC register  *******************/
#define  TMR_SMC_SMSEL                      ((uint16_t)0x0007)            /*!< SMS[2:0] bits (Slave mode selection) */
#define  TMR_SMC_SMSEL_0                    ((uint16_t)0x0001)            /*!< Bit 0 */
#define  TMR_SMC_SMSEL_1                    ((uint16_t)0x0002)            /*!< Bit 1 */
#define  TMR_SMC_SMSEL_2                    ((uint16_t)0x0004)            /*!< Bit 2 */

#define  TMR_SMC_TRGSEL                     ((uint16_t)0x0070)            /*!< TS[2:0] bits (Trigger selection) */
#define  TMR_SMC_TRGSEL_0                   ((uint16_t)0x0010)            /*!< Bit 0 */
#define  TMR_SMC_TRGSEL_1                   ((uint16_t)0x0020)            /*!< Bit 1 */
#define  TMR_SMC_TRGSEL_2                   ((uint16_t)0x0040)            /*!< Bit 2 */

#define  TMR_SMC_MSMODE                     ((uint16_t)0x0080)            /*!< Master/slave mode */

#define  TMR_SMC_ETDF                       ((uint16_t)0x0F00)            /*!< ETF[3:0] bits (External trigger filter) */
#define  TMR_SMC_ETDF_0                     ((uint16_t)0x0100)            /*!< Bit 0 */
#define  TMR_SMC_ETDF_1                     ((uint16_t)0x0200)            /*!< Bit 1 */
#define  TMR_SMC_ETDF_2                     ((uint16_t)0x0400)            /*!< Bit 2 */
#define  TMR_SMC_ETDF_3                     ((uint16_t)0x0800)            /*!< Bit 3 */

#define  TMR_SMC_ETD                        ((uint16_t)0x3000)            /*!< ETPS[1:0] bits (External trigger prescaler) */
#define  TMR_SMC_ETD_0                      ((uint16_t)0x1000)            /*!< Bit 0 */
#define  TMR_SMC_ETD_1                      ((uint16_t)0x2000)            /*!< Bit 1 */

#define  TMR_SMC_ECLKEN                     ((uint16_t)0x4000)            /*!< External clock enable */
#define  TMR_SMC_ETRGP                      ((uint16_t)0x8000)            /*!< External trigger polarity */

/*******************  Bit definition for TMR_DIE register  *******************/
#define  TMR_DIE_UEVIE                      ((uint16_t)0x0001)            /*!< Update interrupt enable */
#define  TMR_DIE_C1IE                       ((uint16_t)0x0002)            /*!< Capture/Compare 1 interrupt enable */
#define  TMR_DIE_C2IE                       ((uint16_t)0x0004)            /*!< Capture/Compare 2 interrupt enable */
#define  TMR_DIE_C3IE                       ((uint16_t)0x0008)            /*!< Capture/Compare 3 interrupt enable */
#define  TMR_DIE_C4IE                       ((uint16_t)0x0010)            /*!< Capture/Compare 4 interrupt enable */
#define  TMR_DIE_HALLIE                     ((uint16_t)0x0020)            /*!< COM interrupt enable */
#define  TMR_DIE_TRGIE                      ((uint16_t)0x0040)            /*!< Trigger interrupt enable */
#define  TMR_DIE_BRKIE                      ((uint16_t)0x0080)            /*!< Break interrupt enable */
#define  TMR_DIE_UEVDE                      ((uint16_t)0x0100)            /*!< Update DMA request enable */
#define  TMR_DIE_C1DE                       ((uint16_t)0x0200)            /*!< Capture/Compare 1 DMA request enable */
#define  TMR_DIE_C2DE                       ((uint16_t)0x0400)            /*!< Capture/Compare 2 DMA request enable */
#define  TMR_DIE_C3DE                       ((uint16_t)0x0800)            /*!< Capture/Compare 3 DMA request enable */
#define  TMR_DIE_C4DE                       ((uint16_t)0x1000)            /*!< Capture/Compare 4 DMA request enable */
#define  TMR_DIE_HALLDE                     ((uint16_t)0x2000)            /*!< COM DMA request enable */
#define  TMR_DIE_TRGDE                      ((uint16_t)0x4000)            /*!< Trigger DMA request enable */

/********************  Bit definition for TMR_STS register  ********************/
#define  TMR_STS_UEVIF                      ((uint16_t)0x0001)            /*!< Update interrupt Flag */
#define  TMR_STS_C1IF                       ((uint16_t)0x0002)            /*!< Capture/Compare 1 interrupt Flag */
#define  TMR_STS_C2IF                       ((uint16_t)0x0004)            /*!< Capture/Compare 2 interrupt Flag */
#define  TMR_STS_C3IF                       ((uint16_t)0x0008)            /*!< Capture/Compare 3 interrupt Flag */
#define  TMR_STS_C4IF                       ((uint16_t)0x0010)            /*!< Capture/Compare 4 interrupt Flag */
#define  TMR_STS_HALLIF                     ((uint16_t)0x0020)            /*!< COM interrupt Flag */
#define  TMR_STS_TRGIF                      ((uint16_t)0x0040)            /*!< Trigger interrupt Flag */
#define  TMR_STS_BRKIF                      ((uint16_t)0x0080)            /*!< Break interrupt Flag */
#define  TMR_STS_C1OF                       ((uint16_t)0x0200)            /*!< Capture/Compare 1 Overcapture Flag */
#define  TMR_STS_C2OF                       ((uint16_t)0x0400)            /*!< Capture/Compare 2 Overcapture Flag */
#define  TMR_STS_C3OF                       ((uint16_t)0x0800)            /*!< Capture/Compare 3 Overcapture Flag */
#define  TMR_STS_C4OF                       ((uint16_t)0x1000)            /*!< Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TMR_EVEG register  ********************/
#define  TMR_EVEG_UEVG                      ((uint8_t)0x01)               /*!< Update Generation */
#define  TMR_EVEG_C1G                       ((uint8_t)0x02)               /*!< Capture/Compare 1 Generation */
#define  TMR_EVEG_C2G                       ((uint8_t)0x04)               /*!< Capture/Compare 2 Generation */
#define  TMR_EVEG_C3G                       ((uint8_t)0x08)               /*!< Capture/Compare 3 Generation */
#define  TMR_EVEG_C4G                       ((uint8_t)0x10)               /*!< Capture/Compare 4 Generation */
#define  TMR_EVEG_HALLG                     ((uint8_t)0x20)               /*!< Capture/Compare Control Update Generation */
#define  TMR_EVEG_TRGG                      ((uint8_t)0x40)               /*!< Trigger Generation */
#define  TMR_EVEG_BRKG                      ((uint8_t)0x80)               /*!< Break Generation */

/******************  Bit definition for TMR_CCM1 register  *******************/
#define  TMR_CCM1_C1SEL                     ((uint16_t)0x0003)            /*!< CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define  TMR_CCM1_C1SEL_0                   ((uint16_t)0x0001)            /*!< Bit 0 */
#define  TMR_CCM1_C1SEL_1                   ((uint16_t)0x0002)            /*!< Bit 1 */

#define  TMR_CCM1_OC1FEN                    ((uint16_t)0x0004)            /*!< Output Compare 1 Fast enable */
#define  TMR_CCM1_OC1PEN                    ((uint16_t)0x0008)            /*!< Output Compare 1 Preload enable */

#define  TMR_CCM1_OC1MODE                   ((uint16_t)0x0070)            /*!< OC1M[2:0] bits (Output Compare 1 Mode) */
#define  TMR_CCM1_OC1MODE_0                 ((uint16_t)0x0010)            /*!< Bit 0 */
#define  TMR_CCM1_OC1MODE_1                 ((uint16_t)0x0020)            /*!< Bit 1 */
#define  TMR_CCM1_OC1MODE_2                 ((uint16_t)0x0040)            /*!< Bit 2 */

#define  TMR_CCM1_C1CDIS                    ((uint16_t)0x0080)            /*!< Output Compare 1Clear Enable */

#define  TMR_CCM1_C2SEL                     ((uint16_t)0x0300)            /*!< CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define  TMR_CCM1_C2SEL_0                   ((uint16_t)0x0100)            /*!< Bit 0 */
#define  TMR_CCM1_C2SEL_1                   ((uint16_t)0x0200)            /*!< Bit 1 */

#define  TMR_CCM1_OC2FNE                    ((uint16_t)0x0400)            /*!< Output Compare 2 Fast enable */
#define  TMR_CCM1_OC2PEN                    ((uint16_t)0x0800)            /*!< Output Compare 2 Preload enable */

#define  TMR_CCM1_OC2MODE                   ((uint16_t)0x7000)            /*!< OC2M[2:0] bits (Output Compare 2 Mode) */
#define  TMR_CCM1_OC2MODE_0                 ((uint16_t)0x1000)            /*!< Bit 0 */
#define  TMR_CCM1_OC2MODE_1                 ((uint16_t)0x2000)            /*!< Bit 1 */
#define  TMR_CCM1_OC2MODE_2                 ((uint16_t)0x4000)            /*!< Bit 2 */

#define  TMR_CCM1_OC2CDIS                   ((uint16_t)0x8000)            /*!< Output Compare 2 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TMR_CCM1_IC1DIV                    ((uint16_t)0x000C)            /*!< IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define  TMR_CCM1_IC1DIV_0                  ((uint16_t)0x0004)            /*!< Bit 0 */
#define  TMR_CCM1_IC1DIV_1                  ((uint16_t)0x0008)            /*!< Bit 1 */

#define  TMR_CCM1_IC1F                      ((uint16_t)0x00F0)            /*!< IC1F[3:0] bits (Input Capture 1 Filter) */
#define  TMR_CCM1_IC1F_0                    ((uint16_t)0x0010)            /*!< Bit 0 */
#define  TMR_CCM1_IC1F_1                    ((uint16_t)0x0020)            /*!< Bit 1 */
#define  TMR_CCM1_IC1F_2                    ((uint16_t)0x0040)            /*!< Bit 2 */
#define  TMR_CCM1_IC1F_3                    ((uint16_t)0x0080)            /*!< Bit 3 */

#define  TMR_CCM1_IC2DIV                    ((uint16_t)0x0C00)            /*!< IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define  TMR_CCM1_IC2DIV_0                  ((uint16_t)0x0400)            /*!< Bit 0 */
#define  TMR_CCM1_IC2DIVC_1                 ((uint16_t)0x0800)            /*!< Bit 1 */

#define  TMR_CCM1_IC2DF                     ((uint16_t)0xF000)            /*!< IC2F[3:0] bits (Input Capture 2 Filter) */
#define  TMR_CCM1_IC2DF_0                   ((uint16_t)0x1000)            /*!< Bit 0 */
#define  TMR_CCM1_IC2DF_1                   ((uint16_t)0x2000)            /*!< Bit 1 */
#define  TMR_CCM1_IC2DF_2                   ((uint16_t)0x4000)            /*!< Bit 2 */
#define  TMR_CCM1_IC2DF_3                   ((uint16_t)0x8000)            /*!< Bit 3 */

/******************  Bit definition for TMR_CCM2 register  *******************/
#define  TMR_CCM2_C3SEL                     ((uint16_t)0x0003)            /*!< CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define  TMR_CCM2_C3SEL_0                   ((uint16_t)0x0001)            /*!< Bit 0 */
#define  TMR_CCM2_C3SEL_1                   ((uint16_t)0x0002)            /*!< Bit 1 */

#define  TMR_CCM2_OC3FEN                    ((uint16_t)0x0004)            /*!< Output Compare 3 Fast enable */
#define  TMR_CCM2_OC3PEN                    ((uint16_t)0x0008)            /*!< Output Compare 3 Preload enable */

#define  TMR_CCM2_OC3MODE                   ((uint16_t)0x0070)            /*!< OC3M[2:0] bits (Output Compare 3 Mode) */
#define  TMR_CCM2_OC3MODE_0                 ((uint16_t)0x0010)            /*!< Bit 0 */
#define  TMR_CCM2_OC3MODE_1                 ((uint16_t)0x0020)            /*!< Bit 1 */
#define  TMR_CCM2_OC3MODE_2                 ((uint16_t)0x0040)            /*!< Bit 2 */

#define  TMR_CCM2_OC3CDIS                   ((uint16_t)0x0080)            /*!< Output Compare 3 Clear Enable */

#define  TMR_CCM2_CC4S                      ((uint16_t)0x0300)            /*!< CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define  TMR_CCM2_CC4S_0                    ((uint16_t)0x0100)            /*!< Bit 0 */
#define  TMR_CCM2_CC4S_1                    ((uint16_t)0x0200)            /*!< Bit 1 */

#define  TMR_CCM2_OC4FE                     ((uint16_t)0x0400)            /*!< Output Compare 4 Fast enable */
#define  TMR_CCM2_OC4PE                     ((uint16_t)0x0800)            /*!< Output Compare 4 Preload enable */

#define  TMR_CCM2_OC4MODE                   ((uint16_t)0x7000)            /*!< OC4M[2:0] bits (Output Compare 4 Mode) */
#define  TMR_CCM2_OC4MODE_0                 ((uint16_t)0x1000)            /*!< Bit 0 */
#define  TMR_CCM2_OC4MODE_1                 ((uint16_t)0x2000)            /*!< Bit 1 */
#define  TMR_CCM2_OC4MODE_2                 ((uint16_t)0x4000)            /*!< Bit 2 */

#define  TMR_CCM2_OC4CDIS                   ((uint16_t)0x8000)            /*!< Output Compare 4 Clear Enable */

/*----------------------------------------------------------------------------*/

#define  TMR_CCM2_IC3DIV                    ((uint16_t)0x000C)            /*!< IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define  TMR_CCM2_IC3DIV_0                  ((uint16_t)0x0004)            /*!< Bit 0 */
#define  TMR_CCM2_IC3DIV_1                  ((uint16_t)0x0008)            /*!< Bit 1 */

#define  TMR_CCM2_IC3DF                     ((uint16_t)0x00F0)            /*!< IC3F[3:0] bits (Input Capture 3 Filter) */
#define  TMR_CCM2_IC3DF_0                   ((uint16_t)0x0010)            /*!< Bit 0 */
#define  TMR_CCM2_IC3DF_1                   ((uint16_t)0x0020)            /*!< Bit 1 */
#define  TMR_CCM2_IC3DF_2                   ((uint16_t)0x0040)            /*!< Bit 2 */
#define  TMR_CCM2_IC3DF_3                   ((uint16_t)0x0080)            /*!< Bit 3 */

#define  TMR_CCM2_IC4DIV                    ((uint16_t)0x0C00)            /*!< IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define  TMR_CCM2_IC4DIV_0                  ((uint16_t)0x0400)            /*!< Bit 0 */
#define  TMR_CCM2_IC4DIV_1                  ((uint16_t)0x0800)            /*!< Bit 1 */

#define  TMR_CCM2_IC4DF                     ((uint16_t)0xF000)            /*!< IC4F[3:0] bits (Input Capture 4 Filter) */
#define  TMR_CCM2_IC4DF_0                   ((uint16_t)0x1000)            /*!< Bit 0 */
#define  TMR_CCM2_IC4DF_1                   ((uint16_t)0x2000)            /*!< Bit 1 */
#define  TMR_CCM2_IC4DF_2                   ((uint16_t)0x4000)            /*!< Bit 2 */
#define  TMR_CCM2_IC4DF_3                   ((uint16_t)0x8000)            /*!< Bit 3 */

/*******************  Bit definition for TMR_CCE register  *******************/
#define  TMR_CCE_C1EN                       ((uint16_t)0x0001)            /*!< Capture/Compare 1 output enable */
#define  TMR_CCE_C1P                        ((uint16_t)0x0002)            /*!< Capture/Compare 1 output Polarity */
#define  TMR_CCE_C1NEN                       ((uint16_t)0x0004)            /*!< Capture/Compare 1 Complementary output enable */
#define  TMR_CCE_C1NP                       ((uint16_t)0x0008)            /*!< Capture/Compare 1 Complementary output Polarity */
#define  TMR_CCE_C2EN                       ((uint16_t)0x0010)            /*!< Capture/Compare 2 output enable */
#define  TMR_CCE_C2P                        ((uint16_t)0x0020)            /*!< Capture/Compare 2 output Polarity */
#define  TMR_CCE_C2NEN                      ((uint16_t)0x0040)            /*!< Capture/Compare 2 Complementary output enable */
#define  TMR_CCE_C2NP                       ((uint16_t)0x0080)            /*!< Capture/Compare 2 Complementary output Polarity */
#define  TMR_CCE_C3EN                       ((uint16_t)0x0100)            /*!< Capture/Compare 3 output enable */
#define  TMR_CCE_C3P                        ((uint16_t)0x0200)            /*!< Capture/Compare 3 output Polarity */
#define  TMR_CCE_C3NEN                      ((uint16_t)0x0400)            /*!< Capture/Compare 3 Complementary output enable */
#define  TMR_CCE_C3NP                       ((uint16_t)0x0800)            /*!< Capture/Compare 3 Complementary output Polarity */
#define  TMR_CCE_C4EN                       ((uint16_t)0x1000)            /*!< Capture/Compare 4 output enable */
#define  TMR_CCE_C4P                        ((uint16_t)0x2000)            /*!< Capture/Compare 4 output Polarity */
#define  TMR_CCE_C4NP                       ((uint16_t)0x8000)            /*!< Capture/Compare 4 Complementary output Polarity */

/*******************  Bit definition for TMR_CNT register  ********************/
#define  TMR_CNT_CNT                        ((uint16_t)0xFFFF)            /*!< Counter Value */

/*******************  Bit definition for TMR_DIV register  ********************/
#define  TMR_DIV_DIV                        ((uint16_t)0xFFFF)            /*!< Prescaler Value */

/*******************  Bit definition for TMR_AR register  ********************/
#define  TMR_AR_AR                          ((uint16_t)0xFFFF)            /*!< actual auto-reload Value */

/*******************  Bit definition for TMR_RC register  ********************/
#define  TMR_RC_RC                          ((uint8_t)0xFF)               /*!< Repetition Counter Value */

/*******************  Bit definition for TMR_CC1 register  *******************/
#define  TMR_CC1_CC1                        ((uint16_t)0xFFFF)            /*!< Capture/Compare 1 Value */

/*******************  Bit definition for TMR_CC2 register  *******************/
#define  TMR_CC2_CC2                        ((uint16_t)0xFFFF)            /*!< Capture/Compare 2 Value */

/*******************  Bit definition for TMR_CC3 register  *******************/
#define  TMR_CC3_CC3                        ((uint16_t)0xFFFF)            /*!< Capture/Compare 3 Value */

/*******************  Bit definition for TMR_CC4 register  *******************/
#define  TMR_CC4_CC4                        ((uint16_t)0xFFFF)            /*!< Capture/Compare 4 Value */

/*******************  Bit definition for TMR_BRKDT register  *******************/
#define  TMR_BRKDT_DTGS                     ((uint16_t)0x00FF)            /*!< DTG[0:7] bits (Dead-Time Generator set-up) */
#define  TMR_BRKDT_DTGS_0                   ((uint16_t)0x0001)            /*!< Bit 0 */
#define  TMR_BRKDT_DTGS_1                   ((uint16_t)0x0002)            /*!< Bit 1 */
#define  TMR_BRKDT_DTGS_2                   ((uint16_t)0x0004)            /*!< Bit 2 */
#define  TMR_BRKDT_DTGS_3                   ((uint16_t)0x0008)            /*!< Bit 3 */
#define  TMR_BRKDT_DTGS_4                   ((uint16_t)0x0010)            /*!< Bit 4 */
#define  TMR_BRKDT_DTGS_5                   ((uint16_t)0x0020)            /*!< Bit 5 */
#define  TMR_BRKDT_DTGS_6                   ((uint16_t)0x0040)            /*!< Bit 6 */
#define  TMR_BRKDT_DTGS_7                   ((uint16_t)0x0080)            /*!< Bit 7 */

#define  TMR_BRKDT_LOCKC                    ((uint16_t)0x0300)            /*!< LOCK[1:0] bits (Lock Configuration) */
#define  TMR_BRKDT_LOCKC_0                  ((uint16_t)0x0100)            /*!< Bit 0 */
#define  TMR_BRKDT_LOCKC_1                  ((uint16_t)0x0200)            /*!< Bit 1 */

#define  TMR_BRKDT_OSIMI                    ((uint16_t)0x0400)            /*!< Off-State Selection for Idle mode */
#define  TMR_BRKDT_OSIMR                    ((uint16_t)0x0800)            /*!< Off-State Selection for Run mode */
#define  TMR_BRKDT_BRKEN                    ((uint16_t)0x1000)            /*!< Break enable */
#define  TMR_BRKDT_BRKP                     ((uint16_t)0x2000)            /*!< Break Polarity */
#define  TMR_BRKDT_AOEN                     ((uint16_t)0x4000)            /*!< Automatic Output enable */
#define  TMR_BRKDT_MOEN                     ((uint16_t)0x8000)            /*!< Main Output enable */

/*******************  Bit definition for TMR_DMAC register  ********************/
#define  TMR_DMAC_ADDR                      ((uint16_t)0x001F)            /*!< DBA[4:0] bits (DMA Base Address) */
#define  TMR_DMAC_ADDR_0                    ((uint16_t)0x0001)            /*!< Bit 0 */
#define  TMR_DMAC_ADDR_1                    ((uint16_t)0x0002)            /*!< Bit 1 */
#define  TMR_DMAC_ADDR_2                    ((uint16_t)0x0004)            /*!< Bit 2 */
#define  TMR_DMAC_ADDR_3                    ((uint16_t)0x0008)            /*!< Bit 3 */
#define  TMR_DMAC_ADDR_4                    ((uint16_t)0x0010)            /*!< Bit 4 */

#define  TMR_DMAC_DBLEN                     ((uint16_t)0x1F00)            /*!< DBL[4:0] bits (DMA Burst Length) */
#define  TMR_DMAC_DBLEN_0                   ((uint16_t)0x0100)            /*!< Bit 0 */
#define  TMR_DMAC_DBLEN_1                   ((uint16_t)0x0200)            /*!< Bit 1 */
#define  TMR_DMAC_DBLEN_2                   ((uint16_t)0x0400)            /*!< Bit 2 */
#define  TMR_DMAC_DBLEN_3                   ((uint16_t)0x0800)            /*!< Bit 3 */
#define  TMR_DMAC_DBLEN_4                   ((uint16_t)0x1000)            /*!< Bit 4 */

/*******************  Bit definition for TMR_DMABA register  *******************/
#define  TMR_DMABA_DMABA                    ((uint16_t)0xFFFF)            /*!< DMA register for burst accesses */

//Copied all registers directly from stm32 file

/*****************************************************************************/
/*                                                                           */
/*                               Timers (TIM)                                */
/*                                                                           */
/*****************************************************************************/
/*******************  Bit definition for TIM_CR1 register  *******************/
#define TIM_CR1_CEN_Pos                     (0U)                               
#define TIM_CR1_CEN_Msk                     (0x1U << TIM_CR1_CEN_Pos)          /*!< 0x00000001 */
#define TIM_CR1_CEN                         TIM_CR1_CEN_Msk                    /*!<Counter enable */
#define TIM_CR1_UDIS_Pos                    (1U)                               
#define TIM_CR1_UDIS_Msk                    (0x1U << TIM_CR1_UDIS_Pos)         /*!< 0x00000002 */
#define TIM_CR1_UDIS                        TIM_CR1_UDIS_Msk                   /*!<Update disable */
#define TIM_CR1_URS_Pos                     (2U)                               
#define TIM_CR1_URS_Msk                     (0x1U << TIM_CR1_URS_Pos)          /*!< 0x00000004 */
#define TIM_CR1_URS                         TIM_CR1_URS_Msk                    /*!<Update request source */
#define TIM_CR1_OPM_Pos                     (3U)                               
#define TIM_CR1_OPM_Msk                     (0x1U << TIM_CR1_OPM_Pos)          /*!< 0x00000008 */
#define TIM_CR1_OPM                         TIM_CR1_OPM_Msk                    /*!<One pulse mode */
#define TIM_CR1_DIR_Pos                     (4U)                               
#define TIM_CR1_DIR_Msk                     (0x1U << TIM_CR1_DIR_Pos)          /*!< 0x00000010 */
#define TIM_CR1_DIR                         TIM_CR1_DIR_Msk                    /*!<Direction */

#define TIM_CR1_CMS_Pos                     (5U)                               
#define TIM_CR1_CMS_Msk                     (0x3U << TIM_CR1_CMS_Pos)          /*!< 0x00000060 */
#define TIM_CR1_CMS                         TIM_CR1_CMS_Msk                    /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0                       (0x1U << TIM_CR1_CMS_Pos)          /*!< 0x00000020 */
#define TIM_CR1_CMS_1                       (0x2U << TIM_CR1_CMS_Pos)          /*!< 0x00000040 */

#define TIM_CR1_ARPE_Pos                    (7U)                               
#define TIM_CR1_ARPE_Msk                    (0x1U << TIM_CR1_ARPE_Pos)         /*!< 0x00000080 */
#define TIM_CR1_ARPE                        TIM_CR1_ARPE_Msk                   /*!<Auto-reload preload enable */

#define TIM_CR1_CKD_Pos                     (8U)                               
#define TIM_CR1_CKD_Msk                     (0x3U << TIM_CR1_CKD_Pos)          /*!< 0x00000300 */
#define TIM_CR1_CKD                         TIM_CR1_CKD_Msk                    /*!<CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0                       (0x1U << TIM_CR1_CKD_Pos)          /*!< 0x00000100 */
#define TIM_CR1_CKD_1                       (0x2U << TIM_CR1_CKD_Pos)          /*!< 0x00000200 */

/*******************  Bit definition for TIM_CR2 register  *******************/
#define TIM_CR2_CCPC_Pos                    (0U)                               
#define TIM_CR2_CCPC_Msk                    (0x1U << TIM_CR2_CCPC_Pos)         /*!< 0x00000001 */
#define TIM_CR2_CCPC                        TIM_CR2_CCPC_Msk                   /*!<Capture/Compare Preloaded Control */
#define TIM_CR2_CCUS_Pos                    (2U)                               
#define TIM_CR2_CCUS_Msk                    (0x1U << TIM_CR2_CCUS_Pos)         /*!< 0x00000004 */
#define TIM_CR2_CCUS                        TIM_CR2_CCUS_Msk                   /*!<Capture/Compare Control Update Selection */
#define TIM_CR2_CCDS_Pos                    (3U)                               
#define TIM_CR2_CCDS_Msk                    (0x1U << TIM_CR2_CCDS_Pos)         /*!< 0x00000008 */
#define TIM_CR2_CCDS                        TIM_CR2_CCDS_Msk                   /*!<Capture/Compare DMA Selection */

#define TIM_CR2_MMS_Pos                     (4U)                               
#define TIM_CR2_MMS_Msk                     (0x7U << TIM_CR2_MMS_Pos)          /*!< 0x00000070 */
#define TIM_CR2_MMS                         TIM_CR2_MMS_Msk                    /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0                       (0x1U << TIM_CR2_MMS_Pos)          /*!< 0x00000010 */
#define TIM_CR2_MMS_1                       (0x2U << TIM_CR2_MMS_Pos)          /*!< 0x00000020 */
#define TIM_CR2_MMS_2                       (0x4U << TIM_CR2_MMS_Pos)          /*!< 0x00000040 */

#define TIM_CR2_TI1S_Pos                    (7U)                               
#define TIM_CR2_TI1S_Msk                    (0x1U << TIM_CR2_TI1S_Pos)         /*!< 0x00000080 */
#define TIM_CR2_TI1S                        TIM_CR2_TI1S_Msk                   /*!<TI1 Selection */
#define TIM_CR2_OIS1_Pos                    (8U)                               
#define TIM_CR2_OIS1_Msk                    (0x1U << TIM_CR2_OIS1_Pos)         /*!< 0x00000100 */
#define TIM_CR2_OIS1                        TIM_CR2_OIS1_Msk                   /*!<Output Idle state 1 (OC1 output) */
#define TIM_CR2_OIS1N_Pos                   (9U)                               
#define TIM_CR2_OIS1N_Msk                   (0x1U << TIM_CR2_OIS1N_Pos)        /*!< 0x00000200 */
#define TIM_CR2_OIS1N                       TIM_CR2_OIS1N_Msk                  /*!<Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2_Pos                    (10U)                              
#define TIM_CR2_OIS2_Msk                    (0x1U << TIM_CR2_OIS2_Pos)         /*!< 0x00000400 */
#define TIM_CR2_OIS2                        TIM_CR2_OIS2_Msk                   /*!<Output Idle state 2 (OC2 output) */
#define TIM_CR2_OIS2N_Pos                   (11U)                              
#define TIM_CR2_OIS2N_Msk                   (0x1U << TIM_CR2_OIS2N_Pos)        /*!< 0x00000800 */
#define TIM_CR2_OIS2N                       TIM_CR2_OIS2N_Msk                  /*!<Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3_Pos                    (12U)                              
#define TIM_CR2_OIS3_Msk                    (0x1U << TIM_CR2_OIS3_Pos)         /*!< 0x00001000 */
#define TIM_CR2_OIS3                        TIM_CR2_OIS3_Msk                   /*!<Output Idle state 3 (OC3 output) */
#define TIM_CR2_OIS3N_Pos                   (13U)                              
#define TIM_CR2_OIS3N_Msk                   (0x1U << TIM_CR2_OIS3N_Pos)        /*!< 0x00002000 */
#define TIM_CR2_OIS3N                       TIM_CR2_OIS3N_Msk                  /*!<Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4_Pos                    (14U)                              
#define TIM_CR2_OIS4_Msk                    (0x1U << TIM_CR2_OIS4_Pos)         /*!< 0x00004000 */
#define TIM_CR2_OIS4                        TIM_CR2_OIS4_Msk                   /*!<Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCR register  ******************/
#define TIM_SMCR_SMS_Pos                    (0U)                               
#define TIM_SMCR_SMS_Msk                    (0x7U << TIM_SMCR_SMS_Pos)         /*!< 0x00000007 */
#define TIM_SMCR_SMS                        TIM_SMCR_SMS_Msk                   /*!<SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCR_SMS_0                      (0x1U << TIM_SMCR_SMS_Pos)         /*!< 0x00000001 */
#define TIM_SMCR_SMS_1                      (0x2U << TIM_SMCR_SMS_Pos)         /*!< 0x00000002 */
#define TIM_SMCR_SMS_2                      (0x4U << TIM_SMCR_SMS_Pos)         /*!< 0x00000004 */

#define TIM_SMCR_TS_Pos                     (4U)                               
#define TIM_SMCR_TS_Msk                     (0x7U << TIM_SMCR_TS_Pos)          /*!< 0x00000070 */
#define TIM_SMCR_TS                         TIM_SMCR_TS_Msk                    /*!<TS[2:0] bits (Trigger selection) */
#define TIM_SMCR_TS_0                       (0x1U << TIM_SMCR_TS_Pos)          /*!< 0x00000010 */
#define TIM_SMCR_TS_1                       (0x2U << TIM_SMCR_TS_Pos)          /*!< 0x00000020 */
#define TIM_SMCR_TS_2                       (0x4U << TIM_SMCR_TS_Pos)          /*!< 0x00000040 */

#define TIM_SMCR_MSM_Pos                    (7U)                               
#define TIM_SMCR_MSM_Msk                    (0x1U << TIM_SMCR_MSM_Pos)         /*!< 0x00000080 */
#define TIM_SMCR_MSM                        TIM_SMCR_MSM_Msk                   /*!<Master/slave mode */

#define TIM_SMCR_ETF_Pos                    (8U)                               
#define TIM_SMCR_ETF_Msk                    (0xFU << TIM_SMCR_ETF_Pos)         /*!< 0x00000F00 */
#define TIM_SMCR_ETF                        TIM_SMCR_ETF_Msk                   /*!<ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0                      (0x1U << TIM_SMCR_ETF_Pos)         /*!< 0x00000100 */
#define TIM_SMCR_ETF_1                      (0x2U << TIM_SMCR_ETF_Pos)         /*!< 0x00000200 */
#define TIM_SMCR_ETF_2                      (0x4U << TIM_SMCR_ETF_Pos)         /*!< 0x00000400 */
#define TIM_SMCR_ETF_3                      (0x8U << TIM_SMCR_ETF_Pos)         /*!< 0x00000800 */

#define TIM_SMCR_ETPS_Pos                   (12U)                              
#define TIM_SMCR_ETPS_Msk                   (0x3U << TIM_SMCR_ETPS_Pos)        /*!< 0x00003000 */
#define TIM_SMCR_ETPS                       TIM_SMCR_ETPS_Msk                  /*!<ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0                     (0x1U << TIM_SMCR_ETPS_Pos)        /*!< 0x00001000 */
#define TIM_SMCR_ETPS_1                     (0x2U << TIM_SMCR_ETPS_Pos)        /*!< 0x00002000 */

#define TIM_SMCR_ECE_Pos                    (14U)                              
#define TIM_SMCR_ECE_Msk                    (0x1U << TIM_SMCR_ECE_Pos)         /*!< 0x00004000 */
#define TIM_SMCR_ECE                        TIM_SMCR_ECE_Msk                   /*!<External clock enable */
#define TIM_SMCR_ETP_Pos                    (15U)                              
#define TIM_SMCR_ETP_Msk                    (0x1U << TIM_SMCR_ETP_Pos)         /*!< 0x00008000 */
#define TIM_SMCR_ETP                        TIM_SMCR_ETP_Msk                   /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  ******************/
#define TIM_DIER_UIE_Pos                    (0U)                               
#define TIM_DIER_UIE_Msk                    (0x1U << TIM_DIER_UIE_Pos)         /*!< 0x00000001 */
#define TIM_DIER_UIE                        TIM_DIER_UIE_Msk                   /*!<Update interrupt enable */
#define TIM_DIER_CC1IE_Pos                  (1U)                               
#define TIM_DIER_CC1IE_Msk                  (0x1U << TIM_DIER_CC1IE_Pos)       /*!< 0x00000002 */
#define TIM_DIER_CC1IE                      TIM_DIER_CC1IE_Msk                 /*!<Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE_Pos                  (2U)                               
#define TIM_DIER_CC2IE_Msk                  (0x1U << TIM_DIER_CC2IE_Pos)       /*!< 0x00000004 */
#define TIM_DIER_CC2IE                      TIM_DIER_CC2IE_Msk                 /*!<Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE_Pos                  (3U)                               
#define TIM_DIER_CC3IE_Msk                  (0x1U << TIM_DIER_CC3IE_Pos)       /*!< 0x00000008 */
#define TIM_DIER_CC3IE                      TIM_DIER_CC3IE_Msk                 /*!<Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE_Pos                  (4U)                               
#define TIM_DIER_CC4IE_Msk                  (0x1U << TIM_DIER_CC4IE_Pos)       /*!< 0x00000010 */
#define TIM_DIER_CC4IE                      TIM_DIER_CC4IE_Msk                 /*!<Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE_Pos                  (5U)                               
#define TIM_DIER_COMIE_Msk                  (0x1U << TIM_DIER_COMIE_Pos)       /*!< 0x00000020 */
#define TIM_DIER_COMIE                      TIM_DIER_COMIE_Msk                 /*!<COM interrupt enable */
#define TIM_DIER_TIE_Pos                    (6U)                               
#define TIM_DIER_TIE_Msk                    (0x1U << TIM_DIER_TIE_Pos)         /*!< 0x00000040 */
#define TIM_DIER_TIE                        TIM_DIER_TIE_Msk                   /*!<Trigger interrupt enable */
#define TIM_DIER_BIE_Pos                    (7U)                               
#define TIM_DIER_BIE_Msk                    (0x1U << TIM_DIER_BIE_Pos)         /*!< 0x00000080 */
#define TIM_DIER_BIE                        TIM_DIER_BIE_Msk                   /*!<Break interrupt enable */
#define TIM_DIER_UDE_Pos                    (8U)                               
#define TIM_DIER_UDE_Msk                    (0x1U << TIM_DIER_UDE_Pos)         /*!< 0x00000100 */
#define TIM_DIER_UDE                        TIM_DIER_UDE_Msk                   /*!<Update DMA request enable */
#define TIM_DIER_CC1DE_Pos                  (9U)                               
#define TIM_DIER_CC1DE_Msk                  (0x1U << TIM_DIER_CC1DE_Pos)       /*!< 0x00000200 */
#define TIM_DIER_CC1DE                      TIM_DIER_CC1DE_Msk                 /*!<Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE_Pos                  (10U)                              
#define TIM_DIER_CC2DE_Msk                  (0x1U << TIM_DIER_CC2DE_Pos)       /*!< 0x00000400 */
#define TIM_DIER_CC2DE                      TIM_DIER_CC2DE_Msk                 /*!<Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE_Pos                  (11U)                              
#define TIM_DIER_CC3DE_Msk                  (0x1U << TIM_DIER_CC3DE_Pos)       /*!< 0x00000800 */
#define TIM_DIER_CC3DE                      TIM_DIER_CC3DE_Msk                 /*!<Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE_Pos                  (12U)                              
#define TIM_DIER_CC4DE_Msk                  (0x1U << TIM_DIER_CC4DE_Pos)       /*!< 0x00001000 */
#define TIM_DIER_CC4DE                      TIM_DIER_CC4DE_Msk                 /*!<Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE_Pos                  (13U)                              
#define TIM_DIER_COMDE_Msk                  (0x1U << TIM_DIER_COMDE_Pos)       /*!< 0x00002000 */
#define TIM_DIER_COMDE                      TIM_DIER_COMDE_Msk                 /*!<COM DMA request enable */
#define TIM_DIER_TDE_Pos                    (14U)                              
#define TIM_DIER_TDE_Msk                    (0x1U << TIM_DIER_TDE_Pos)         /*!< 0x00004000 */
#define TIM_DIER_TDE                        TIM_DIER_TDE_Msk                   /*!<Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  *******************/
#define TIM_SR_UIF_Pos                      (0U)                               
#define TIM_SR_UIF_Msk                      (0x1U << TIM_SR_UIF_Pos)           /*!< 0x00000001 */
#define TIM_SR_UIF                          TIM_SR_UIF_Msk                     /*!<Update interrupt Flag */
#define TIM_SR_CC1IF_Pos                    (1U)                               
#define TIM_SR_CC1IF_Msk                    (0x1U << TIM_SR_CC1IF_Pos)         /*!< 0x00000002 */
#define TIM_SR_CC1IF                        TIM_SR_CC1IF_Msk                   /*!<Capture/Compare 1 interrupt Flag */
#define TIM_SR_CC2IF_Pos                    (2U)                               
#define TIM_SR_CC2IF_Msk                    (0x1U << TIM_SR_CC2IF_Pos)         /*!< 0x00000004 */
#define TIM_SR_CC2IF                        TIM_SR_CC2IF_Msk                   /*!<Capture/Compare 2 interrupt Flag */
#define TIM_SR_CC3IF_Pos                    (3U)                               
#define TIM_SR_CC3IF_Msk                    (0x1U << TIM_SR_CC3IF_Pos)         /*!< 0x00000008 */
#define TIM_SR_CC3IF                        TIM_SR_CC3IF_Msk                   /*!<Capture/Compare 3 interrupt Flag */
#define TIM_SR_CC4IF_Pos                    (4U)                               
#define TIM_SR_CC4IF_Msk                    (0x1U << TIM_SR_CC4IF_Pos)         /*!< 0x00000010 */
#define TIM_SR_CC4IF                        TIM_SR_CC4IF_Msk                   /*!<Capture/Compare 4 interrupt Flag */
#define TIM_SR_COMIF_Pos                    (5U)                               
#define TIM_SR_COMIF_Msk                    (0x1U << TIM_SR_COMIF_Pos)         /*!< 0x00000020 */
#define TIM_SR_COMIF                        TIM_SR_COMIF_Msk                   /*!<COM interrupt Flag */
#define TIM_SR_TIF_Pos                      (6U)                               
#define TIM_SR_TIF_Msk                      (0x1U << TIM_SR_TIF_Pos)           /*!< 0x00000040 */
#define TIM_SR_TIF                          TIM_SR_TIF_Msk                     /*!<Trigger interrupt Flag */
#define TIM_SR_BIF_Pos                      (7U)                               
#define TIM_SR_BIF_Msk                      (0x1U << TIM_SR_BIF_Pos)           /*!< 0x00000080 */
#define TIM_SR_BIF                          TIM_SR_BIF_Msk                     /*!<Break interrupt Flag */
#define TIM_SR_CC1OF_Pos                    (9U)                               
#define TIM_SR_CC1OF_Msk                    (0x1U << TIM_SR_CC1OF_Pos)         /*!< 0x00000200 */
#define TIM_SR_CC1OF                        TIM_SR_CC1OF_Msk                   /*!<Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF_Pos                    (10U)                              
#define TIM_SR_CC2OF_Msk                    (0x1U << TIM_SR_CC2OF_Pos)         /*!< 0x00000400 */
#define TIM_SR_CC2OF                        TIM_SR_CC2OF_Msk                   /*!<Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF_Pos                    (11U)                              
#define TIM_SR_CC3OF_Msk                    (0x1U << TIM_SR_CC3OF_Pos)         /*!< 0x00000800 */
#define TIM_SR_CC3OF                        TIM_SR_CC3OF_Msk                   /*!<Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF_Pos                    (12U)                              
#define TIM_SR_CC4OF_Msk                    (0x1U << TIM_SR_CC4OF_Pos)         /*!< 0x00001000 */
#define TIM_SR_CC4OF                        TIM_SR_CC4OF_Msk                   /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  *******************/
#define TIM_EGR_UG_Pos                      (0U)                               
#define TIM_EGR_UG_Msk                      (0x1U << TIM_EGR_UG_Pos)           /*!< 0x00000001 */
#define TIM_EGR_UG                          TIM_EGR_UG_Msk                     /*!<Update Generation */
#define TIM_EGR_CC1G_Pos                    (1U)                               
#define TIM_EGR_CC1G_Msk                    (0x1U << TIM_EGR_CC1G_Pos)         /*!< 0x00000002 */
#define TIM_EGR_CC1G                        TIM_EGR_CC1G_Msk                   /*!<Capture/Compare 1 Generation */
#define TIM_EGR_CC2G_Pos                    (2U)                               
#define TIM_EGR_CC2G_Msk                    (0x1U << TIM_EGR_CC2G_Pos)         /*!< 0x00000004 */
#define TIM_EGR_CC2G                        TIM_EGR_CC2G_Msk                   /*!<Capture/Compare 2 Generation */
#define TIM_EGR_CC3G_Pos                    (3U)                               
#define TIM_EGR_CC3G_Msk                    (0x1U << TIM_EGR_CC3G_Pos)         /*!< 0x00000008 */
#define TIM_EGR_CC3G                        TIM_EGR_CC3G_Msk                   /*!<Capture/Compare 3 Generation */
#define TIM_EGR_CC4G_Pos                    (4U)                               
#define TIM_EGR_CC4G_Msk                    (0x1U << TIM_EGR_CC4G_Pos)         /*!< 0x00000010 */
#define TIM_EGR_CC4G                        TIM_EGR_CC4G_Msk                   /*!<Capture/Compare 4 Generation */
#define TIM_EGR_COMG_Pos                    (5U)                               
#define TIM_EGR_COMG_Msk                    (0x1U << TIM_EGR_COMG_Pos)         /*!< 0x00000020 */
#define TIM_EGR_COMG                        TIM_EGR_COMG_Msk                   /*!<Capture/Compare Control Update Generation */
#define TIM_EGR_TG_Pos                      (6U)                               
#define TIM_EGR_TG_Msk                      (0x1U << TIM_EGR_TG_Pos)           /*!< 0x00000040 */
#define TIM_EGR_TG                          TIM_EGR_TG_Msk                     /*!<Trigger Generation */
#define TIM_EGR_BG_Pos                      (7U)                               
#define TIM_EGR_BG_Msk                      (0x1U << TIM_EGR_BG_Pos)           /*!< 0x00000080 */
#define TIM_EGR_BG                          TIM_EGR_BG_Msk                     /*!<Break Generation */

/******************  Bit definition for TIM_CCMR1 register  ******************/
#define TIM_CCMR1_CC1S_Pos                  (0U)                               
#define TIM_CCMR1_CC1S_Msk                  (0x3U << TIM_CCMR1_CC1S_Pos)       /*!< 0x00000003 */
#define TIM_CCMR1_CC1S                      TIM_CCMR1_CC1S_Msk                 /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0                    (0x1U << TIM_CCMR1_CC1S_Pos)       /*!< 0x00000001 */
#define TIM_CCMR1_CC1S_1                    (0x2U << TIM_CCMR1_CC1S_Pos)       /*!< 0x00000002 */

#define TIM_CCMR1_OC1FE_Pos                 (2U)                               
#define TIM_CCMR1_OC1FE_Msk                 (0x1U << TIM_CCMR1_OC1FE_Pos)      /*!< 0x00000004 */
#define TIM_CCMR1_OC1FE                     TIM_CCMR1_OC1FE_Msk                /*!<Output Compare 1 Fast enable */
#define TIM_CCMR1_OC1PE_Pos                 (3U)                               
#define TIM_CCMR1_OC1PE_Msk                 (0x1U << TIM_CCMR1_OC1PE_Pos)      /*!< 0x00000008 */
#define TIM_CCMR1_OC1PE                     TIM_CCMR1_OC1PE_Msk                /*!<Output Compare 1 Preload enable */

#define TIM_CCMR1_OC1M_Pos                  (4U)                               
#define TIM_CCMR1_OC1M_Msk                  (0x7U << TIM_CCMR1_OC1M_Pos)       /*!< 0x00000070 */
#define TIM_CCMR1_OC1M                      TIM_CCMR1_OC1M_Msk                 /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1M_0                    (0x1U << TIM_CCMR1_OC1M_Pos)       /*!< 0x00000010 */
#define TIM_CCMR1_OC1M_1                    (0x2U << TIM_CCMR1_OC1M_Pos)       /*!< 0x00000020 */
#define TIM_CCMR1_OC1M_2                    (0x4U << TIM_CCMR1_OC1M_Pos)       /*!< 0x00000040 */

#define TIM_CCMR1_OC1CE_Pos                 (7U)                               
#define TIM_CCMR1_OC1CE_Msk                 (0x1U << TIM_CCMR1_OC1CE_Pos)      /*!< 0x00000080 */
#define TIM_CCMR1_OC1CE                     TIM_CCMR1_OC1CE_Msk                /*!<Output Compare 1Clear Enable */

#define TIM_CCMR1_CC2S_Pos                  (8U)                               
#define TIM_CCMR1_CC2S_Msk                  (0x3U << TIM_CCMR1_CC2S_Pos)       /*!< 0x00000300 */
#define TIM_CCMR1_CC2S                      TIM_CCMR1_CC2S_Msk                 /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0                    (0x1U << TIM_CCMR1_CC2S_Pos)       /*!< 0x00000100 */
#define TIM_CCMR1_CC2S_1                    (0x2U << TIM_CCMR1_CC2S_Pos)       /*!< 0x00000200 */

#define TIM_CCMR1_OC2FE_Pos                 (10U)                              
#define TIM_CCMR1_OC2FE_Msk                 (0x1U << TIM_CCMR1_OC2FE_Pos)      /*!< 0x00000400 */
#define TIM_CCMR1_OC2FE                     TIM_CCMR1_OC2FE_Msk                /*!<Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE_Pos                 (11U)                              
#define TIM_CCMR1_OC2PE_Msk                 (0x1U << TIM_CCMR1_OC2PE_Pos)      /*!< 0x00000800 */
#define TIM_CCMR1_OC2PE                     TIM_CCMR1_OC2PE_Msk                /*!<Output Compare 2 Preload enable */

#define TIM_CCMR1_OC2M_Pos                  (12U)                              
#define TIM_CCMR1_OC2M_Msk                  (0x7U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00007000 */
#define TIM_CCMR1_OC2M                      TIM_CCMR1_OC2M_Msk                 /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2M_0                    (0x1U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00001000 */
#define TIM_CCMR1_OC2M_1                    (0x2U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_2                    (0x4U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00004000 */

#define TIM_CCMR1_OC2CE_Pos                 (15U)                              
#define TIM_CCMR1_OC2CE_Msk                 (0x1U << TIM_CCMR1_OC2CE_Pos)      /*!< 0x00008000 */
#define TIM_CCMR1_OC2CE                     TIM_CCMR1_OC2CE_Msk                /*!<Output Compare 2 Clear Enable */

/*---------------------------------------------------------------------------*/

#define TIM_CCMR1_IC1PSC_Pos                (2U)                               
#define TIM_CCMR1_IC1PSC_Msk                (0x3U << TIM_CCMR1_IC1PSC_Pos)     /*!< 0x0000000C */
#define TIM_CCMR1_IC1PSC                    TIM_CCMR1_IC1PSC_Msk               /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0                  (0x1U << TIM_CCMR1_IC1PSC_Pos)     /*!< 0x00000004 */
#define TIM_CCMR1_IC1PSC_1                  (0x2U << TIM_CCMR1_IC1PSC_Pos)     /*!< 0x00000008 */

#define TIM_CCMR1_IC1F_Pos                  (4U)                               
#define TIM_CCMR1_IC1F_Msk                  (0xFU << TIM_CCMR1_IC1F_Pos)       /*!< 0x000000F0 */
#define TIM_CCMR1_IC1F                      TIM_CCMR1_IC1F_Msk                 /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMR1_IC1F_0                    (0x1U << TIM_CCMR1_IC1F_Pos)       /*!< 0x00000010 */
#define TIM_CCMR1_IC1F_1                    (0x2U << TIM_CCMR1_IC1F_Pos)       /*!< 0x00000020 */
#define TIM_CCMR1_IC1F_2                    (0x4U << TIM_CCMR1_IC1F_Pos)       /*!< 0x00000040 */
#define TIM_CCMR1_IC1F_3                    (0x8U << TIM_CCMR1_IC1F_Pos)       /*!< 0x00000080 */

#define TIM_CCMR1_IC2PSC_Pos                (10U)                              
#define TIM_CCMR1_IC2PSC_Msk                (0x3U << TIM_CCMR1_IC2PSC_Pos)     /*!< 0x00000C00 */
#define TIM_CCMR1_IC2PSC                    TIM_CCMR1_IC2PSC_Msk               /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2PSC_0                  (0x1U << TIM_CCMR1_IC2PSC_Pos)     /*!< 0x00000400 */
#define TIM_CCMR1_IC2PSC_1                  (0x2U << TIM_CCMR1_IC2PSC_Pos)     /*!< 0x00000800 */

#define TIM_CCMR1_IC2F_Pos                  (12U)                              
#define TIM_CCMR1_IC2F_Msk                  (0xFU << TIM_CCMR1_IC2F_Pos)       /*!< 0x0000F000 */
#define TIM_CCMR1_IC2F                      TIM_CCMR1_IC2F_Msk                 /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_CCMR1_IC2F_0                    (0x1U << TIM_CCMR1_IC2F_Pos)       /*!< 0x00001000 */
#define TIM_CCMR1_IC2F_1                    (0x2U << TIM_CCMR1_IC2F_Pos)       /*!< 0x00002000 */
#define TIM_CCMR1_IC2F_2                    (0x4U << TIM_CCMR1_IC2F_Pos)       /*!< 0x00004000 */
#define TIM_CCMR1_IC2F_3                    (0x8U << TIM_CCMR1_IC2F_Pos)       /*!< 0x00008000 */

/******************  Bit definition for TIM_CCMR2 register  ******************/
#define TIM_CCMR2_CC3S_Pos                  (0U)                               
#define TIM_CCMR2_CC3S_Msk                  (0x3U << TIM_CCMR2_CC3S_Pos)       /*!< 0x00000003 */
#define TIM_CCMR2_CC3S                      TIM_CCMR2_CC3S_Msk                 /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMR2_CC3S_0                    (0x1U << TIM_CCMR2_CC3S_Pos)       /*!< 0x00000001 */
#define TIM_CCMR2_CC3S_1                    (0x2U << TIM_CCMR2_CC3S_Pos)       /*!< 0x00000002 */

#define TIM_CCMR2_OC3FE_Pos                 (2U)                               
#define TIM_CCMR2_OC3FE_Msk                 (0x1U << TIM_CCMR2_OC3FE_Pos)      /*!< 0x00000004 */
#define TIM_CCMR2_OC3FE                     TIM_CCMR2_OC3FE_Msk                /*!<Output Compare 3 Fast enable */
#define TIM_CCMR2_OC3PE_Pos                 (3U)                               
#define TIM_CCMR2_OC3PE_Msk                 (0x1U << TIM_CCMR2_OC3PE_Pos)      /*!< 0x00000008 */
#define TIM_CCMR2_OC3PE                     TIM_CCMR2_OC3PE_Msk                /*!<Output Compare 3 Preload enable */

#define TIM_CCMR2_OC3M_Pos                  (4U)                               
#define TIM_CCMR2_OC3M_Msk                  (0x7U << TIM_CCMR2_OC3M_Pos)       /*!< 0x00000070 */
#define TIM_CCMR2_OC3M                      TIM_CCMR2_OC3M_Msk                 /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0                    (0x1U << TIM_CCMR2_OC3M_Pos)       /*!< 0x00000010 */
#define TIM_CCMR2_OC3M_1                    (0x2U << TIM_CCMR2_OC3M_Pos)       /*!< 0x00000020 */
#define TIM_CCMR2_OC3M_2                    (0x4U << TIM_CCMR2_OC3M_Pos)       /*!< 0x00000040 */

#define TIM_CCMR2_OC3CE_Pos                 (7U)                               
#define TIM_CCMR2_OC3CE_Msk                 (0x1U << TIM_CCMR2_OC3CE_Pos)      /*!< 0x00000080 */
#define TIM_CCMR2_OC3CE                     TIM_CCMR2_OC3CE_Msk                /*!<Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S_Pos                  (8U)                               
#define TIM_CCMR2_CC4S_Msk                  (0x3U << TIM_CCMR2_CC4S_Pos)       /*!< 0x00000300 */
#define TIM_CCMR2_CC4S                      TIM_CCMR2_CC4S_Msk                 /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0                    (0x1U << TIM_CCMR2_CC4S_Pos)       /*!< 0x00000100 */
#define TIM_CCMR2_CC4S_1                    (0x2U << TIM_CCMR2_CC4S_Pos)       /*!< 0x00000200 */

#define TIM_CCMR2_OC4FE_Pos                 (10U)                              
#define TIM_CCMR2_OC4FE_Msk                 (0x1U << TIM_CCMR2_OC4FE_Pos)      /*!< 0x00000400 */
#define TIM_CCMR2_OC4FE                     TIM_CCMR2_OC4FE_Msk                /*!<Output Compare 4 Fast enable */
#define TIM_CCMR2_OC4PE_Pos                 (11U)                              
#define TIM_CCMR2_OC4PE_Msk                 (0x1U << TIM_CCMR2_OC4PE_Pos)      /*!< 0x00000800 */
#define TIM_CCMR2_OC4PE                     TIM_CCMR2_OC4PE_Msk                /*!<Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M_Pos                  (12U)                              
#define TIM_CCMR2_OC4M_Msk                  (0x7U << TIM_CCMR2_OC4M_Pos)       /*!< 0x00007000 */
#define TIM_CCMR2_OC4M                      TIM_CCMR2_OC4M_Msk                 /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0                    (0x1U << TIM_CCMR2_OC4M_Pos)       /*!< 0x00001000 */
#define TIM_CCMR2_OC4M_1                    (0x2U << TIM_CCMR2_OC4M_Pos)       /*!< 0x00002000 */
#define TIM_CCMR2_OC4M_2                    (0x4U << TIM_CCMR2_OC4M_Pos)       /*!< 0x00004000 */

#define TIM_CCMR2_OC4CE_Pos                 (15U)                              
#define TIM_CCMR2_OC4CE_Msk                 (0x1U << TIM_CCMR2_OC4CE_Pos)      /*!< 0x00008000 */
#define TIM_CCMR2_OC4CE                     TIM_CCMR2_OC4CE_Msk                /*!<Output Compare 4 Clear Enable */

/*---------------------------------------------------------------------------*/

#define TIM_CCMR2_IC3PSC_Pos                (2U)                               
#define TIM_CCMR2_IC3PSC_Msk                (0x3U << TIM_CCMR2_IC3PSC_Pos)     /*!< 0x0000000C */
#define TIM_CCMR2_IC3PSC                    TIM_CCMR2_IC3PSC_Msk               /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0                  (0x1U << TIM_CCMR2_IC3PSC_Pos)     /*!< 0x00000004 */
#define TIM_CCMR2_IC3PSC_1                  (0x2U << TIM_CCMR2_IC3PSC_Pos)     /*!< 0x00000008 */

#define TIM_CCMR2_IC3F_Pos                  (4U)                               
#define TIM_CCMR2_IC3F_Msk                  (0xFU << TIM_CCMR2_IC3F_Pos)       /*!< 0x000000F0 */
#define TIM_CCMR2_IC3F                      TIM_CCMR2_IC3F_Msk                 /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0                    (0x1U << TIM_CCMR2_IC3F_Pos)       /*!< 0x00000010 */
#define TIM_CCMR2_IC3F_1                    (0x2U << TIM_CCMR2_IC3F_Pos)       /*!< 0x00000020 */
#define TIM_CCMR2_IC3F_2                    (0x4U << TIM_CCMR2_IC3F_Pos)       /*!< 0x00000040 */
#define TIM_CCMR2_IC3F_3                    (0x8U << TIM_CCMR2_IC3F_Pos)       /*!< 0x00000080 */

#define TIM_CCMR2_IC4PSC_Pos                (10U)                              
#define TIM_CCMR2_IC4PSC_Msk                (0x3U << TIM_CCMR2_IC4PSC_Pos)     /*!< 0x00000C00 */
#define TIM_CCMR2_IC4PSC                    TIM_CCMR2_IC4PSC_Msk               /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4PSC_0                  (0x1U << TIM_CCMR2_IC4PSC_Pos)     /*!< 0x00000400 */
#define TIM_CCMR2_IC4PSC_1                  (0x2U << TIM_CCMR2_IC4PSC_Pos)     /*!< 0x00000800 */

#define TIM_CCMR2_IC4F_Pos                  (12U)                              
#define TIM_CCMR2_IC4F_Msk                  (0xFU << TIM_CCMR2_IC4F_Pos)       /*!< 0x0000F000 */
#define TIM_CCMR2_IC4F                      TIM_CCMR2_IC4F_Msk                 /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMR2_IC4F_0                    (0x1U << TIM_CCMR2_IC4F_Pos)       /*!< 0x00001000 */
#define TIM_CCMR2_IC4F_1                    (0x2U << TIM_CCMR2_IC4F_Pos)       /*!< 0x00002000 */
#define TIM_CCMR2_IC4F_2                    (0x4U << TIM_CCMR2_IC4F_Pos)       /*!< 0x00004000 */
#define TIM_CCMR2_IC4F_3                    (0x8U << TIM_CCMR2_IC4F_Pos)       /*!< 0x00008000 */

/*******************  Bit definition for TIM_CCER register  ******************/
#define TIM_CCER_CC1E_Pos                   (0U)                               
#define TIM_CCER_CC1E_Msk                   (0x1U << TIM_CCER_CC1E_Pos)        /*!< 0x00000001 */
#define TIM_CCER_CC1E                       TIM_CCER_CC1E_Msk                  /*!<Capture/Compare 1 output enable */
#define TIM_CCER_CC1P_Pos                   (1U)                               
#define TIM_CCER_CC1P_Msk                   (0x1U << TIM_CCER_CC1P_Pos)        /*!< 0x00000002 */
#define TIM_CCER_CC1P                       TIM_CCER_CC1P_Msk                  /*!<Capture/Compare 1 output Polarity */
#define TIM_CCER_CC1NE_Pos                  (2U)                               
#define TIM_CCER_CC1NE_Msk                  (0x1U << TIM_CCER_CC1NE_Pos)       /*!< 0x00000004 */
#define TIM_CCER_CC1NE                      TIM_CCER_CC1NE_Msk                 /*!<Capture/Compare 1 Complementary output enable */
#define TIM_CCER_CC1NP_Pos                  (3U)                               
#define TIM_CCER_CC1NP_Msk                  (0x1U << TIM_CCER_CC1NP_Pos)       /*!< 0x00000008 */
#define TIM_CCER_CC1NP                      TIM_CCER_CC1NP_Msk                 /*!<Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E_Pos                   (4U)                               
#define TIM_CCER_CC2E_Msk                   (0x1U << TIM_CCER_CC2E_Pos)        /*!< 0x00000010 */
#define TIM_CCER_CC2E                       TIM_CCER_CC2E_Msk                  /*!<Capture/Compare 2 output enable */
#define TIM_CCER_CC2P_Pos                   (5U)                               
#define TIM_CCER_CC2P_Msk                   (0x1U << TIM_CCER_CC2P_Pos)        /*!< 0x00000020 */
#define TIM_CCER_CC2P                       TIM_CCER_CC2P_Msk                  /*!<Capture/Compare 2 output Polarity */
#define TIM_CCER_CC2NE_Pos                  (6U)                               
#define TIM_CCER_CC2NE_Msk                  (0x1U << TIM_CCER_CC2NE_Pos)       /*!< 0x00000040 */
#define TIM_CCER_CC2NE                      TIM_CCER_CC2NE_Msk                 /*!<Capture/Compare 2 Complementary output enable */
#define TIM_CCER_CC2NP_Pos                  (7U)                               
#define TIM_CCER_CC2NP_Msk                  (0x1U << TIM_CCER_CC2NP_Pos)       /*!< 0x00000080 */
#define TIM_CCER_CC2NP                      TIM_CCER_CC2NP_Msk                 /*!<Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E_Pos                   (8U)                               
#define TIM_CCER_CC3E_Msk                   (0x1U << TIM_CCER_CC3E_Pos)        /*!< 0x00000100 */
#define TIM_CCER_CC3E                       TIM_CCER_CC3E_Msk                  /*!<Capture/Compare 3 output enable */
#define TIM_CCER_CC3P_Pos                   (9U)                               
#define TIM_CCER_CC3P_Msk                   (0x1U << TIM_CCER_CC3P_Pos)        /*!< 0x00000200 */
#define TIM_CCER_CC3P                       TIM_CCER_CC3P_Msk                  /*!<Capture/Compare 3 output Polarity */
#define TIM_CCER_CC3NE_Pos                  (10U)                              
#define TIM_CCER_CC3NE_Msk                  (0x1U << TIM_CCER_CC3NE_Pos)       /*!< 0x00000400 */
#define TIM_CCER_CC3NE                      TIM_CCER_CC3NE_Msk                 /*!<Capture/Compare 3 Complementary output enable */
#define TIM_CCER_CC3NP_Pos                  (11U)                              
#define TIM_CCER_CC3NP_Msk                  (0x1U << TIM_CCER_CC3NP_Pos)       /*!< 0x00000800 */
#define TIM_CCER_CC3NP                      TIM_CCER_CC3NP_Msk                 /*!<Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E_Pos                   (12U)                              
#define TIM_CCER_CC4E_Msk                   (0x1U << TIM_CCER_CC4E_Pos)        /*!< 0x00001000 */
#define TIM_CCER_CC4E                       TIM_CCER_CC4E_Msk                  /*!<Capture/Compare 4 output enable */
#define TIM_CCER_CC4P_Pos                   (13U)                              
#define TIM_CCER_CC4P_Msk                   (0x1U << TIM_CCER_CC4P_Pos)        /*!< 0x00002000 */
#define TIM_CCER_CC4P                       TIM_CCER_CC4P_Msk                  /*!<Capture/Compare 4 output Polarity */

/*******************  Bit definition for TIM_CNT register  *******************/
#define TIM_CNT_CNT_Pos                     (0U)                               
#define TIM_CNT_CNT_Msk                     (0xFFFFFFFFU << TIM_CNT_CNT_Pos)   /*!< 0xFFFFFFFF */
#define TIM_CNT_CNT                         TIM_CNT_CNT_Msk                    /*!<Counter Value */

/*******************  Bit definition for TIM_PSC register  *******************/
#define TIM_PSC_PSC_Pos                     (0U)                               
#define TIM_PSC_PSC_Msk                     (0xFFFFU << TIM_PSC_PSC_Pos)       /*!< 0x0000FFFF */
#define TIM_PSC_PSC                         TIM_PSC_PSC_Msk                    /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  *******************/
#define TIM_ARR_ARR_Pos                     (0U)                               
#define TIM_ARR_ARR_Msk                     (0xFFFFFFFFU << TIM_ARR_ARR_Pos)   /*!< 0xFFFFFFFF */
#define TIM_ARR_ARR                         TIM_ARR_ARR_Msk                    /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  *******************/
#define TIM_RCR_REP_Pos                     (0U)                               
#define TIM_RCR_REP_Msk                     (0xFFU << TIM_RCR_REP_Pos)         /*!< 0x000000FF */
#define TIM_RCR_REP                         TIM_RCR_REP_Msk                    /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  ******************/
#define TIM_CCR1_CCR1_Pos                   (0U)                               
#define TIM_CCR1_CCR1_Msk                   (0xFFFFU << TIM_CCR1_CCR1_Pos)     /*!< 0x0000FFFF */
#define TIM_CCR1_CCR1                       TIM_CCR1_CCR1_Msk                  /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  ******************/
#define TIM_CCR2_CCR2_Pos                   (0U)                               
#define TIM_CCR2_CCR2_Msk                   (0xFFFFU << TIM_CCR2_CCR2_Pos)     /*!< 0x0000FFFF */
#define TIM_CCR2_CCR2                       TIM_CCR2_CCR2_Msk                  /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  ******************/
#define TIM_CCR3_CCR3_Pos                   (0U)                               
#define TIM_CCR3_CCR3_Msk                   (0xFFFFU << TIM_CCR3_CCR3_Pos)     /*!< 0x0000FFFF */
#define TIM_CCR3_CCR3                       TIM_CCR3_CCR3_Msk                  /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  ******************/
#define TIM_CCR4_CCR4_Pos                   (0U)                               
#define TIM_CCR4_CCR4_Msk                   (0xFFFFU << TIM_CCR4_CCR4_Pos)     /*!< 0x0000FFFF */
#define TIM_CCR4_CCR4                       TIM_CCR4_CCR4_Msk                  /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BDTR register  ******************/
#define TIM_BDTR_DTG_Pos                    (0U)                               
#define TIM_BDTR_DTG_Msk                    (0xFFU << TIM_BDTR_DTG_Pos)        /*!< 0x000000FF */
#define TIM_BDTR_DTG                        TIM_BDTR_DTG_Msk                   /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0                      (0x01U << TIM_BDTR_DTG_Pos)        /*!< 0x00000001 */
#define TIM_BDTR_DTG_1                      (0x02U << TIM_BDTR_DTG_Pos)        /*!< 0x00000002 */
#define TIM_BDTR_DTG_2                      (0x04U << TIM_BDTR_DTG_Pos)        /*!< 0x00000004 */
#define TIM_BDTR_DTG_3                      (0x08U << TIM_BDTR_DTG_Pos)        /*!< 0x00000008 */
#define TIM_BDTR_DTG_4                      (0x10U << TIM_BDTR_DTG_Pos)        /*!< 0x00000010 */
#define TIM_BDTR_DTG_5                      (0x20U << TIM_BDTR_DTG_Pos)        /*!< 0x00000020 */
#define TIM_BDTR_DTG_6                      (0x40U << TIM_BDTR_DTG_Pos)        /*!< 0x00000040 */
#define TIM_BDTR_DTG_7                      (0x80U << TIM_BDTR_DTG_Pos)        /*!< 0x00000080 */

#define TIM_BDTR_LOCK_Pos                   (8U)                               
#define TIM_BDTR_LOCK_Msk                   (0x3U << TIM_BDTR_LOCK_Pos)        /*!< 0x00000300 */
#define TIM_BDTR_LOCK                       TIM_BDTR_LOCK_Msk                  /*!<LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0                     (0x1U << TIM_BDTR_LOCK_Pos)        /*!< 0x00000100 */
#define TIM_BDTR_LOCK_1                     (0x2U << TIM_BDTR_LOCK_Pos)        /*!< 0x00000200 */

#define TIM_BDTR_OSSI_Pos                   (10U)                              
#define TIM_BDTR_OSSI_Msk                   (0x1U << TIM_BDTR_OSSI_Pos)        /*!< 0x00000400 */
#define TIM_BDTR_OSSI                       TIM_BDTR_OSSI_Msk                  /*!<Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR_Pos                   (11U)                              
#define TIM_BDTR_OSSR_Msk                   (0x1U << TIM_BDTR_OSSR_Pos)        /*!< 0x00000800 */
#define TIM_BDTR_OSSR                       TIM_BDTR_OSSR_Msk                  /*!<Off-State Selection for Run mode */
#define TIM_BDTR_BKE_Pos                    (12U)                              
#define TIM_BDTR_BKE_Msk                    (0x1U << TIM_BDTR_BKE_Pos)         /*!< 0x00001000 */
#define TIM_BDTR_BKE                        TIM_BDTR_BKE_Msk                   /*!<Break enable */
#define TIM_BDTR_BKP_Pos                    (13U)                              
#define TIM_BDTR_BKP_Msk                    (0x1U << TIM_BDTR_BKP_Pos)         /*!< 0x00002000 */
#define TIM_BDTR_BKP                        TIM_BDTR_BKP_Msk                   /*!<Break Polarity */
#define TIM_BDTR_AOE_Pos                    (14U)                              
#define TIM_BDTR_AOE_Msk                    (0x1U << TIM_BDTR_AOE_Pos)         /*!< 0x00004000 */
#define TIM_BDTR_AOE                        TIM_BDTR_AOE_Msk                   /*!<Automatic Output enable */
#define TIM_BDTR_MOE_Pos                    (15U)                              
#define TIM_BDTR_MOE_Msk                    (0x1U << TIM_BDTR_MOE_Pos)         /*!< 0x00008000 */
#define TIM_BDTR_MOE                        TIM_BDTR_MOE_Msk                   /*!<Main Output enable */

/*******************  Bit definition for TIM_DCR register  *******************/
#define TIM_DCR_DBA_Pos                     (0U)                               
#define TIM_DCR_DBA_Msk                     (0x1FU << TIM_DCR_DBA_Pos)         /*!< 0x0000001F */
#define TIM_DCR_DBA                         TIM_DCR_DBA_Msk                    /*!<DBA[4:0] bits (DMA Base Address) */
#define TIM_DCR_DBA_0                       (0x01U << TIM_DCR_DBA_Pos)         /*!< 0x00000001 */
#define TIM_DCR_DBA_1                       (0x02U << TIM_DCR_DBA_Pos)         /*!< 0x00000002 */
#define TIM_DCR_DBA_2                       (0x04U << TIM_DCR_DBA_Pos)         /*!< 0x00000004 */
#define TIM_DCR_DBA_3                       (0x08U << TIM_DCR_DBA_Pos)         /*!< 0x00000008 */
#define TIM_DCR_DBA_4                       (0x10U << TIM_DCR_DBA_Pos)         /*!< 0x00000010 */

#define TIM_DCR_DBL_Pos                     (8U)                               
#define TIM_DCR_DBL_Msk                     (0x1FU << TIM_DCR_DBL_Pos)         /*!< 0x00001F00 */
#define TIM_DCR_DBL                         TIM_DCR_DBL_Msk                    /*!<DBL[4:0] bits (DMA Burst Length) */
#define TIM_DCR_DBL_0                       (0x01U << TIM_DCR_DBL_Pos)         /*!< 0x00000100 */
#define TIM_DCR_DBL_1                       (0x02U << TIM_DCR_DBL_Pos)         /*!< 0x00000200 */
#define TIM_DCR_DBL_2                       (0x04U << TIM_DCR_DBL_Pos)         /*!< 0x00000400 */
#define TIM_DCR_DBL_3                       (0x08U << TIM_DCR_DBL_Pos)         /*!< 0x00000800 */
#define TIM_DCR_DBL_4                       (0x10U << TIM_DCR_DBL_Pos)         /*!< 0x00001000 */

/*******************  Bit definition for TIM_DMAR register  ******************/
#define TIM_DMAR_DMAB_Pos                   (0U)                               
#define TIM_DMAR_DMAB_Msk                   (0xFFFFU << TIM_DMAR_DMAB_Pos)     /*!< 0x0000FFFF */
#define TIM_DMAR_DMAB                       TIM_DMAR_DMAB_Msk                  /*!<DMA register for burst accesses */



/******************************************************************************/
/*                                                                            */
/*                             Real-Time Clock                                */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for RTC_CTRLH register  ********************/
#define  RTC_CTRLH_PACEIEN                  ((uint8_t)0x01)               /*!< Second Interrupt Enable */
#define  RTC_CTRLH_ALAIEN                   ((uint8_t)0x02)               /*!< Alarm Interrupt Enable */
#define  RTC_CTRLH_OVIEN                    ((uint8_t)0x04)               /*!< OverfloW Interrupt Enable */

/*******************  Bit definition for RTC_CTRLL register  ********************/
#define  RTC_CTRLL_PACEF                    ((uint8_t)0x01)               /*!< Second Flag */
#define  RTC_CTRLL_ALAF                     ((uint8_t)0x02)               /*!< Alarm Flag */
#define  RTC_CTRLL_OVF                      ((uint8_t)0x04)               /*!< OverfloW Flag */
#define  RTC_CTRLL_RSYNF                    ((uint8_t)0x08)               /*!< Registers Synchronized Flag */
#define  RTC_CTRLL_CMF                      ((uint8_t)0x10)               /*!< Configuration Flag */
#define  RTC_CTRLL_RTF                      ((uint8_t)0x20)               /*!< RTC operation OFF */

/*******************  Bit definition for RTC_DIVH register  *******************/
#define  RTC_DIVH_DIV                       ((uint16_t)0x000F)            /*!< RTC Prescaler Reload Value High */

/*******************  Bit definition for RTC_DIVL register  *******************/
#define  RTC_DIVL_DIV                       ((uint16_t)0xFFFF)            /*!< RTC Prescaler Reload Value Low */

/*******************  Bit definition for RTC_DIVCNTH register  *******************/
#define  RTC_DIVCNTH_RTC_DIVCNT             ((uint16_t)0x000F)            /*!< RTC Clock Divider High */

/*******************  Bit definition for RTC_DIVCNTL register  *******************/
#define  RTC_DIVCNTL_RTC_DIVCNT             ((uint16_t)0xFFFF)            /*!< RTC Clock Divider Low */

/*******************  Bit definition for RTC_CNTH register  *******************/
#define  RTC_CNTH_CNT                       ((uint16_t)0xFFFF)            /*!< RTC Counter High */

/*******************  Bit definition for RTC_CNTL register  *******************/
#define  RTC_CNTL_CNT                       ((uint16_t)0xFFFF)            /*!< RTC Counter Low */

/*******************  Bit definition for RTC_ALAH register  *******************/
#define  RTC_ALAH_ALA                       ((uint16_t)0xFFFF)            /*!< RTC Alarm High */

/*******************  Bit definition for RTC_ALAL register  *******************/
#define  RTC_ALAL_ALA                       ((uint16_t)0xFFFF)            /*!< RTC Alarm Low */

/******************************************************************************/
/*                                                                            */
/*                           Independent WATCHDOG                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for IWDG_KEY register  ********************/
#define  IWDG_KEY_KEY                       ((uint16_t)0xFFFF)            /*!< Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PSC register  ********************/
#define  IWDG_PSC_PSC                       ((uint8_t)0x07)               /*!< PR[2:0] (Prescaler divider) */
#define  IWDG_PSC_PSC_0                     ((uint8_t)0x01)               /*!< Bit 0 */
#define  IWDG_PSC_PSC_1                     ((uint8_t)0x02)               /*!< Bit 1 */
#define  IWDG_PSC_PSC_2                     ((uint8_t)0x04)               /*!< Bit 2 */

/*******************  Bit definition for IWDG_RLD register  *******************/
#define  IWDG_RLD_RLD                       ((uint16_t)0x0FFF)            /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_STS register  ********************/
#define  IWDG_STS_PSCF                      ((uint8_t)0x01)               /*!< Watchdog prescaler value update */
#define  IWDG_STS_RLDF                      ((uint8_t)0x02)               /*!< Watchdog counter reload value update */

/******************************************************************************/
/*                                                                            */
/*                            Window WATCHDOG                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for WWDG_CTRL register  ********************/
#define  WWDG_CTRL_CNTR                     ((uint8_t)0x7F)               /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define  WWDG_CTRL_CNTR0                    ((uint8_t)0x01)               /*!< Bit 0 */
#define  WWDG_CTRL_CNTR1                    ((uint8_t)0x02)               /*!< Bit 1 */
#define  WWDG_CTRL_CNTR2                    ((uint8_t)0x04)               /*!< Bit 2 */
#define  WWDG_CTRL_CNTR3                    ((uint8_t)0x08)               /*!< Bit 3 */
#define  WWDG_CTRL_CNTR4                    ((uint8_t)0x10)               /*!< Bit 4 */
#define  WWDG_CTRL_CNTR5                    ((uint8_t)0x20)               /*!< Bit 5 */
#define  WWDG_CTRL_CNTR6                    ((uint8_t)0x40)               /*!< Bit 6 */

#define  WWDG_CTRL_EN                       ((uint8_t)0x80)               /*!< Activation bit */

/*******************  Bit definition for WWDG_CFG register  *******************/
#define  WWDG_CFG_WCNTR                     ((uint16_t)0x007F)            /*!< W[6:0] bits (7-bit window value) */
#define  WWDG_CFG_WCNTR0                    ((uint16_t)0x0001)            /*!< Bit 0 */
#define  WWDG_CFG_WCNTR1                    ((uint16_t)0x0002)            /*!< Bit 1 */
#define  WWDG_CFG_WCNTR2                    ((uint16_t)0x0004)            /*!< Bit 2 */
#define  WWDG_CFG_WCNTR3                    ((uint16_t)0x0008)            /*!< Bit 3 */
#define  WWDG_CFG_WCNTR4                    ((uint16_t)0x0010)            /*!< Bit 4 */
#define  WWDG_CFG_WCNTR5                    ((uint16_t)0x0020)            /*!< Bit 5 */
#define  WWDG_CFG_WCNTR6                    ((uint16_t)0x0040)            /*!< Bit 6 */

#define  WWDG_CFG_PSC                       ((uint16_t)0x0180)            /*!< WDGTB[1:0] bits (Timer Base) */
#define  WWDG_CFG_PSC0                      ((uint16_t)0x0080)            /*!< Bit 0 */
#define  WWDG_CFG_PSC1                      ((uint16_t)0x0100)            /*!< Bit 1 */

#define  WWDG_CFG_EWIEN                     ((uint16_t)0x0200)            /*!< Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_STS register  ********************/
#define  WWDG_STS_EWIF                      ((uint8_t)0x01)               /*!< Early Wakeup Interrupt Flag */

/******************************************************************************/
/*                                                                            */
/*                       eXternal Memory Controller                    		  */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for XMC_BK1CTRL1 register  *******************/
#define  XMC_BK1CTRL1_EN                    ((uint32_t)0x00000001)        /*!< Memory bank enable bit */
#define  XMC_BK1CTRL1_MUXEN                 ((uint32_t)0x00000002)        /*!< Address/data multiplexing enable bit */

#define  XMC_BK1CTRL1_DEV                   ((uint32_t)0x0000000C)        /*!< MTYP[1:0] bits (Memory type) */
#define  XMC_BK1CTRL1_DEV_0                 ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  XMC_BK1CTRL1_DEV_1                 ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  XMC_BK1CTRL1_BUSTYPE               ((uint32_t)0x00000030)        /*!< MWID[1:0] bits (Memory data bus width) */
#define  XMC_BK1CTRL1_BUSTYPE_0             ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1CTRL1_BUSTYPE_1             ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  XMC_BK1CTRL1_NOREN                 ((uint32_t)0x00000040)        /*!< Flash access enable */
#define  XMC_BK1CTRL1_BURSTEN               ((uint32_t)0x00000100)        /*!< Burst enable bit */
#define  XMC_BK1CTRL1_WAITALV               ((uint32_t)0x00000200)        /*!< Wait signal polarity bit */
#define  XMC_BK1CTRL1_BRSTSPLTEN            ((uint32_t)0x00000400)        /*!< Wrapped burst mode support */
#define  XMC_BK1CTRL1_WAITCFG               ((uint32_t)0x00000800)        /*!< Wait timing configuration */
#define  XMC_BK1CTRL1_WREN                  ((uint32_t)0x00001000)        /*!< Write enable bit */
#define  XMC_BK1CTRL1_WAITEN                ((uint32_t)0x00002000)        /*!< Wait enable bit */
#define  XMC_BK1CTRL1_TMGWREN               ((uint32_t)0x00004000)        /*!< Extended mode enable */
#define  XMC_BK1CTRL1_WAITASYNC             ((uint32_t)0x00008000)       /*!< Asynchronous wait */
#define  XMC_BK1CTRL1_BURSTWRSYN            ((uint32_t)0x00080000)        /*!< Write burst enable */

/******************  Bit definition for XMC_BK1CTRL2 register  *******************/
#define  XMC_BK1CTRL2_EN                    ((uint32_t)0x00000001)        /*!< Memory bank enable bit */
#define  XMC_BK1CTRL2_MUXEN                 ((uint32_t)0x00000002)        /*!< Address/data multiplexing enable bit */

#define  XMC_BK1CTRL2_DEV                   ((uint32_t)0x0000000C)        /*!< MTYP[1:0] bits (Memory type) */
#define  XMC_BK1CTRL2_DEV_0                 ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  XMC_BK1CTRL2_DEV_1                 ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  XMC_BK1CTRL2_BUSTYPE               ((uint32_t)0x00000030)        /*!< MWID[1:0] bits (Memory data bus width) */
#define  XMC_BK1CTRL2_BUSTYPE_0             ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1CTRL2_BUSTYPE_1             ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  XMC_BK1CTRL2_NOREN                 ((uint32_t)0x00000040)        /*!< Flash access enable */
#define  XMC_BK1CTRL2_BURSTEN               ((uint32_t)0x00000100)        /*!< Burst enable bit */
#define  XMC_BK1CTRL2_WAITALV               ((uint32_t)0x00000200)        /*!< Wait signal polarity bit */
#define  XMC_BK1CTRL2_BRSTSPLTEN            ((uint32_t)0x00000400)        /*!< Wrapped burst mode support */
#define  XMC_BK1CTRL2_WAITCFG               ((uint32_t)0x00000800)        /*!< Wait timing configuration */
#define  XMC_BK1CTRL2_WREN                  ((uint32_t)0x00001000)        /*!< Write enable bit */
#define  XMC_BK1CTRL2_WAITEN                ((uint32_t)0x00002000)        /*!< Wait enable bit */
#define  XMC_BK1CTRL2_TMGWREN               ((uint32_t)0x00004000)        /*!< Extended mode enable */
#define  XMC_BK1CTRL2_WAITASYNC             ((uint32_t)0x00008000)       /*!< Asynchronous wait */
#define  XMC_BK1CTRL2_BURSTWRSYN            ((uint32_t)0x00080000)        /*!< Write burst enable */

/******************  Bit definition for XMC_BK1CTRL3 register  *******************/
#define  XMC_BK1CTRL3_EN                    ((uint32_t)0x00000001)        /*!< Memory bank enable bit */
#define  XMC_BK1CTRL3_MUXEN                 ((uint32_t)0x00000002)        /*!< Address/data multiplexing enable bit */

#define  XMC_BK1CTRL3_DEV                   ((uint32_t)0x0000000C)        /*!< MTYP[1:0] bits (Memory type) */
#define  XMC_BK1CTRL3_DEV_0                 ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  XMC_BK1CTRL3_DEV_1                 ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  XMC_BK1CTRL3_BUSTYPE               ((uint32_t)0x00000030)        /*!< MWID[1:0] bits (Memory data bus width) */
#define  XMC_BK1CTRL3_BUSTYPE_0             ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1CTRL3_BUSTYPE_1             ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  XMC_BK1CTRL3_NOREN                 ((uint32_t)0x00000040)        /*!< Flash access enable */
#define  XMC_BK1CTRL3_BURSTEN               ((uint32_t)0x00000100)        /*!< Burst enable bit */
#define  XMC_BK1CTRL3_WAITALV               ((uint32_t)0x00000200)        /*!< Wait signal polarity bit. */
#define  XMC_BK1CTRL3_BRSTSPLTEN            ((uint32_t)0x00000400)        /*!< Wrapped burst mode support */
#define  XMC_BK1CTRL3_WAITCFG               ((uint32_t)0x00000800)        /*!< Wait timing configuration */
#define  XMC_BK1CTRL3_WREN                  ((uint32_t)0x00001000)        /*!< Write enable bit */
#define  XMC_BK1CTRL3_WAITEN                ((uint32_t)0x00002000)        /*!< Wait enable bit */
#define  XMC_BK1CTRL3_TMGWREN               ((uint32_t)0x00004000)        /*!< Extended mode enable */
#define  XMC_BK1CTRL3_WAITASYNC             ((uint32_t)0x00008000)       /*!< Asynchronous wait */
#define  XMC_BK1CTRL3_BURSTWRSYN            ((uint32_t)0x00080000)        /*!< Write burst enable */

/******************  Bit definition for XMC_BK1CTRL4 register  *******************/
#define  XMC_BK1CTRL4_EN                    ((uint32_t)0x00000001)        /*!< Memory bank enable bit */
#define  XMC_BK1CTRL4_MUXEN                 ((uint32_t)0x00000002)        /*!< Address/data multiplexing enable bit */

#define  XMC_BK1CTRL4_DEV                   ((uint32_t)0x0000000C)        /*!< MTYP[1:0] bits (Memory type) */
#define  XMC_BK1CTRL4_DEV_0                 ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  XMC_BK1CTRL4_DEV_1                 ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  XMC_BK1CTRL4_BUSTYPE               ((uint32_t)0x00000030)        /*!< MWID[1:0] bits (Memory data bus width) */
#define  XMC_BK1CTRL4_BUSTYPE_0             ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1CTRL4_BUSTYPE_1             ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  XMC_BK1CTRL4_NOREN                 ((uint32_t)0x00000040)        /*!< Flash access enable */
#define  XMC_BK1CTRL4_BURSTEN               ((uint32_t)0x00000100)        /*!< Burst enable bit */
#define  XMC_BK1CTRL4_WAITALV               ((uint32_t)0x00000200)        /*!< Wait signal polarity bit */
#define  XMC_BK1CTRL4_BRSTSPLTEN            ((uint32_t)0x00000400)        /*!< Wrapped burst mode support */
#define  XMC_BK1CTRL4_WAITCFG               ((uint32_t)0x00000800)        /*!< Wait timing configuration */
#define  XMC_BK1CTRL4_WREN                  ((uint32_t)0x00001000)        /*!< Write enable bit */
#define  XMC_BK1CTRL4_WAITEN                ((uint32_t)0x00002000)        /*!< Wait enable bit */
#define  XMC_BK1CTRL4_TMGWREN               ((uint32_t)0x00004000)        /*!< Extended mode enable */
#define  XMC_BK1CTRL4_WAITASYNC             ((uint32_t)0x00008000)       /*!< Asynchronous wait */
#define  XMC_BK1CTRL4_BURSTWRSYN            ((uint32_t)0x00080000)        /*!< Write burst enable */

/******************  Bit definition for XMC_BK1TMG1 register  ******************/
#define  XMC_BK1TMG1_ADROP                  ((uint32_t)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  XMC_BK1TMG1_ADROP_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK1TMG1_ADROP_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK1TMG1_ADROP_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK1TMG1_ADROP_3                ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  XMC_BK1TMG1_ADRHLD                 ((uint32_t)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  XMC_BK1TMG1_ADRHLD_0               ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1TMG1_ADRHLD_1               ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  XMC_BK1TMG1_ADRHLD_2               ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  XMC_BK1TMG1_ADRHLD_3               ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  XMC_BK1TMG1_DTOP                   ((uint32_t)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  XMC_BK1TMG1_DTOP_0                 ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK1TMG1_DTOP_1                 ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK1TMG1_DTOP_2                 ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK1TMG1_DTOP_3                 ((uint32_t)0x00000800)        /*!< Bit 3 */

#define  XMC_BK1TMG1_INTVLOP                ((uint32_t)0x000F0000)        /*!< BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  XMC_BK1TMG1_INTVLOP_0              ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK1TMG1_INTVLOP_1              ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK1TMG1_INTVLOP_2              ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK1TMG1_INTVLOP_3              ((uint32_t)0x00080000)        /*!< Bit 3 */

#define  XMC_BK1TMG1_CLKPSC                 ((uint32_t)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  XMC_BK1TMG1_CLKPSC_0               ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  XMC_BK1TMG1_CLKPSC_1               ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  XMC_BK1TMG1_CLKPSC_2               ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  XMC_BK1TMG1_CLKPSC_3               ((uint32_t)0x00800000)        /*!< Bit 3 */

#define  XMC_BK1TMG1_DTSTBL                 ((uint32_t)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  XMC_BK1TMG1_DTSTBL_0               ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK1TMG1_DTSTBL_1               ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK1TMG1_DTSTBL_2               ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK1TMG1_DTSTBL_3               ((uint32_t)0x08000000)        /*!< Bit 3 */

#define  XMC_BK1TMG1_MODE                   ((uint32_t)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  XMC_BK1TMG1_MODE_0                 ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  XMC_BK1TMG1_MODE_1                 ((uint32_t)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for XMC_BK1TMG2 register  *******************/
#define  XMC_BK1TMG2_ADROP                  ((uint32_t)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  XMC_BK1TMG2_ADROP_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK1TMG2_ADROP_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK1TMG2_ADROP_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK1TMG2_ADROP_3                ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  XMC_BK1TMG2_ADRHLD                 ((uint32_t)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  XMC_BK1TMG2_ADRHLD_0               ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1TMG2_ADRHLD_1               ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  XMC_BK1TMG2_ADRHLD_2               ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  XMC_BK1TMG2_ADRHLD_3               ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  XMC_BK1TMG2_DTOP                   ((uint32_t)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  XMC_BK1TMG2_DTOP_0                 ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK1TMG2_DTOP_1                 ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK1TMG2_DTOP_2                 ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK1TMG2_DTOP_3                 ((uint32_t)0x00000800)        /*!< Bit 3 */

#define  XMC_BK1TMG2_INTVLOP                ((uint32_t)0x000F0000)        /*!< BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  XMC_BK1TMG2_INTVLOP_0              ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK1TMG2_INTVLOP_1              ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK1TMG2_INTVLOP_2              ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK1TMG2_INTVLOP_3              ((uint32_t)0x00080000)        /*!< Bit 3 */

#define  XMC_BK1TMG2_CLKPSC                 ((uint32_t)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  XMC_BK1TMG2_CLKPSC_0               ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  XMC_BK1TMG2_CLKPSC_1               ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  XMC_BK1TMG2_CLKPSC_2               ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  XMC_BK1TMG2_CLKPSC_3               ((uint32_t)0x00800000)        /*!< Bit 3 */

#define  XMC_BK1TMG2_DTSTBL                 ((uint32_t)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  XMC_BK1TMG2_DTSTBL_0               ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK1TMG2_DTSTBL_1               ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK1TMG2_DTSTBL_2               ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK1TMG2_DTSTBL_3               ((uint32_t)0x08000000)        /*!< Bit 3 */

#define  XMC_BK1TMG2_MODE                   ((uint32_t)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  XMC_BK1TMG2_MODE_0                 ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  XMC_BK1TMG2_MODE_1                 ((uint32_t)0x20000000)        /*!< Bit 1 */

/*******************  Bit definition for XMC_BK1TMG3 register  *******************/
#define  XMC_BK1TMG3_ADROP                  ((uint32_t)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  XMC_BK1TMG3_ADROP_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK1TMG3_ADROP_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK1TMG3_ADROP_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK1TMG3_ADROP_3                ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  XMC_BK1TMG3_ADRHLD                 ((uint32_t)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  XMC_BK1TMG3_ADRHLD_0               ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1TMG3_ADRHLD_1               ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  XMC_BK1TMG3_ADRHLD_2               ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  XMC_BK1TMG3_ADRHLD_3               ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  XMC_BK1TMG3_DTOP                   ((uint32_t)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  XMC_BK1TMG3_DTOP_0                 ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK1TMG3_DTOP_1                 ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK1TMG3_DTOP_2                 ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK1TMG3_DTOP_3                 ((uint32_t)0x00000800)        /*!< Bit 3 */

#define  XMC_BK1TMG3_INTVLOP                ((uint32_t)0x000F0000)        /*!< BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  XMC_BK1TMG3_INTVLOP_0              ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK1TMG3_INTVLOP_1              ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK1TMG3_INTVLOP_2              ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK1TMG3_INTVLOP_3              ((uint32_t)0x00080000)        /*!< Bit 3 */

#define  XMC_BK1TMG3_CLKPSC                 ((uint32_t)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  XMC_BK1TMG3_CLKPSC_0               ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  XMC_BK1TMG3_CLKPSC_1               ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  XMC_BK1TMG3_CLKPSC_2               ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  XMC_BK1TMG3_CLKPSC_3               ((uint32_t)0x00800000)        /*!< Bit 3 */

#define  XMC_BK1TMG3_DTSTBL                 ((uint32_t)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  XMC_BK1TMG3_DTSTBL_0               ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK1TMG3_DTSTBL_1               ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK1TMG3_DTSTBL_2               ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK1TMG3_DTSTBL_3               ((uint32_t)0x08000000)        /*!< Bit 3 */

#define  XMC_BK1TMG3_MODE                   ((uint32_t)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  XMC_BK1TMG3_MODE_0                 ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  XMC_BK1TMG3_MODE_1                 ((uint32_t)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for XMC_BK1TMG4 register  *******************/
#define  XMC_BK1TMG4_ADROP                  ((uint32_t)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  XMC_BK1TMG4_ADROP_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK1TMG4_ADROP_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK1TMG4_ADROP_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK1TMG4_ADROP_3                ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  XMC_BK1TMG4_ADRHLD                 ((uint32_t)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  XMC_BK1TMG4_ADRHLD_0               ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1TMG4_ADRHLD_1               ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  XMC_BK1TMG4_ADRHLD_2               ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  XMC_BK1TMG4_ADRHLD_3               ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  XMC_BK1TMG4_DTOP                   ((uint32_t)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  XMC_BK1TMG4_DTOP_0                 ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK1TMG4_DTOP_1                 ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK1TMG4_DTOP_2                 ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK1TMG4_DTOP_3                 ((uint32_t)0x00000800)        /*!< Bit 3 */

#define  XMC_BK1TMG4_INTVLOP                ((uint32_t)0x000F0000)        /*!< BUSTURN[3:0] bits (Bus turnaround phase duration) */
#define  XMC_BK1TMG4_INTVLOP_0              ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK1TMG4_INTVLOP_1              ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK1TMG4_INTVLOP_2              ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK1TMG4_INTVLOP_3              ((uint32_t)0x00080000)        /*!< Bit 3 */

#define  XMC_BK1TMG4_CLKPSC                 ((uint32_t)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  XMC_BK1TMG4_CLKPSC_0               ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  XMC_BK1TMG4_CLKPSC_1               ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  XMC_BK1TMG4_CLKPSC_2               ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  XMC_BK1TMG4_CLKPSC_3               ((uint32_t)0x00800000)        /*!< Bit 3 */

#define  XMC_BK1TMG4_DTSTBL                 ((uint32_t)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  XMC_BK1TMG4_DTSTBL_0               ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK1TMG4_DTSTBL_1               ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK1TMG4_DTSTBL_2               ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK1TMG4_DTSTBL_3               ((uint32_t)0x08000000)        /*!< Bit 3 */

#define  XMC_BK1TMG4_MODE                   ((uint32_t)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  XMC_BK1TMG4_MODE_0                 ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  XMC_BK1TMG4_MODE_1                 ((uint32_t)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for XMC_BK1TMGWR1 register  ******************/
#define  XMC_BK1TMGWR1_ADROP                ((uint32_t)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  XMC_BK1TMGWR1_ADROP_0              ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK1TMGWR1_ADROP_1              ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK1TMGWR1_ADROP_2              ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK1TMGWR1_ADROP_3              ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  XMC_BK1TMGWR1_ADRHLD               ((uint32_t)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  XMC_BK1TMGWR1_ADRHLD_0             ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1TMGWR1_ADRHLD_1             ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  XMC_BK1TMGWR1_ADRHLD_2             ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  XMC_BK1TMGWR1_ADRHLD_3             ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  XMC_BK1TMGWR1_DTOP                 ((uint32_t)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  XMC_BK1TMGWR1_DTOP_0               ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK1TMGWR1_DTOP_1               ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK1TMGWR1_DTOP_2               ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK1TMGWR1_DTOP_3               ((uint32_t)0x00000800)        /*!< Bit 3 */

#define  XMC_BK1TMGWR1_CLKPSC               ((uint32_t)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  XMC_BK1TMGWR1_CLKPSC_0             ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR1_CLKPSC_1             ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  XMC_BK1TMGWR1_CLKPSC_2             ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  XMC_BK1TMGWR1_CLKPSC_3             ((uint32_t)0x00800000)        /*!< Bit 3 */

#define  XMC_BK1TMGWR1_DTSTBL               ((uint32_t)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  XMC_BK1TMGWR1_DTSTBL_0             ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR1_DTSTBL_1             ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK1TMGWR1_DTSTBL_2             ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK1TMGWR1_DTSTBL_3             ((uint32_t)0x08000000)        /*!< Bit 3 */

#define  XMC_BK1TMGWR1_MODE                 ((uint32_t)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  XMC_BK1TMGWR1_MODE_0               ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR1_MODE_1               ((uint32_t)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for XMC_BK1TMGWR2 register  ******************/
#define  XMC_BK1TMGWR2_ADROP                ((uint32_t)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  XMC_BK1TMGWR2_ADROP_0              ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK1TMGWR2_ADROP_1              ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK1TMGWR2_ADROP_2              ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK1TMGWR2_ADROP_3              ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  XMC_BK1TMGWR2_ADRHLD               ((uint32_t)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  XMC_BK1TMGWR2_ADRHLD_0             ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1TMGWR2_ADRHLD_1             ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  XMC_BK1TMGWR2_ADRHLD_2             ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  XMC_BK1TMGWR2_ADRHLD_3             ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  XMC_BK1TMGWR2_DTOP                 ((uint32_t)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  XMC_BK1TMGWR2_DTOP_0               ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK1TMGWR2_DTOP_1               ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK1TMGWR2_DTOP_2               ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK1TMGWR2_DTOP_3               ((uint32_t)0x00000800)        /*!< Bit 3 */

#define  XMC_BK1TMGWR2_CLKPSC               ((uint32_t)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  XMC_BK1TMGWR2_CLKPSC_0             ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR2_CLKPSC_1             ((uint32_t)0x00200000)        /*!< Bit 1*/
#define  XMC_BK1TMGWR2_CLKPSC_2             ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  XMC_BK1TMGWR2_CLKPSC_3             ((uint32_t)0x00800000)        /*!< Bit 3 */

#define  XMC_BK1TMGWR2_DTSTBL               ((uint32_t)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  XMC_BK1TMGWR2_DTSTBL_0             ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR2_DTSTBL_1             ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK1TMGWR2_DTSTBL_2             ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK1TMGWR2_DTSTBL_3             ((uint32_t)0x08000000)        /*!< Bit 3 */

#define  XMC_BK1TMGWR2_MODE                 ((uint32_t)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  XMC_BK1TMGWR2_MODE_0               ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR2_MODE_1               ((uint32_t)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for XMC_BK1TMGWR3 register  ******************/
#define  XMC_BK1TMGWR3_ADROP                ((uint32_t)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  XMC_BK1TMGWR3_ADROP_0              ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK1TMGWR3_ADROP_1              ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK1TMGWR3_ADROP_2              ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK1TMGWR3_ADROP_3              ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  XMC_BK1TMGWR3_ADRHLD               ((uint32_t)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  XMC_BK1TMGWR3_ADRHLD_0             ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1TMGWR3_ADRHLD_1             ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  XMC_BK1TMGWR3_ADRHLD_2             ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  XMC_BK1TMGWR3_ADRHLD_3             ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  XMC_BK1TMGWR3_DTOP                 ((uint32_t)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  XMC_BK1TMGWR3_DTOP_0               ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK1TMGWR3_DTOP_1               ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK1TMGWR3_DTOP_2               ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK1TMGWR3_DTOP_3               ((uint32_t)0x00000800)        /*!< Bit 3 */

#define  XMC_BK1TMGWR3_CLKPSC               ((uint32_t)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  XMC_BK1TMGWR3_CLKPSC_0             ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR3_CLKPSC_1             ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  XMC_BK1TMGWR3_CLKPSC_2             ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  XMC_BK1TMGWR3_CLKPSC_3             ((uint32_t)0x00800000)        /*!< Bit 3 */

#define  XMC_BK1TMGWR3_DTSTBL               ((uint32_t)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  XMC_BK1TMGWR3_DTSTBL_0             ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR3_DTSTBL_1             ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK1TMGWR3_DTSTBL_2             ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK1TMGWR3_DTSTBL_3             ((uint32_t)0x08000000)        /*!< Bit 3 */

#define  XMC_BK1TMGWR3_MODE                 ((uint32_t)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  XMC_BK1TMGWR3_MODE_0               ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR3_MODE_1               ((uint32_t)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for XMC_BK1TMGWR4 register  ******************/
#define  XMC_BK1TMGWR4_ADROP                ((uint32_t)0x0000000F)        /*!< ADDSET[3:0] bits (Address setup phase duration) */
#define  XMC_BK1TMGWR4_ADROP_0              ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK1TMGWR4_ADROP_1              ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK1TMGWR4_ADROP_2              ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK1TMGWR4_ADROP_3              ((uint32_t)0x00000008)        /*!< Bit 3 */

#define  XMC_BK1TMGWR4_ADRHLD               ((uint32_t)0x000000F0)        /*!< ADDHLD[3:0] bits (Address-hold phase duration) */
#define  XMC_BK1TMGWR4_ADRHLD_0             ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK1TMGWR4_ADRHLD_1             ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  XMC_BK1TMGWR4_ADRHLD_2             ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  XMC_BK1TMGWR4_ADRHLD_3             ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  XMC_BK1TMGWR4_DTOP                 ((uint32_t)0x0000FF00)        /*!< DATAST [3:0] bits (Data-phase duration) */
#define  XMC_BK1TMGWR4_DTOP_0               ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK1TMGWR4_DTOP_1               ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK1TMGWR4_DTOP_2               ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK1TMGWR4_DTOP_3               ((uint32_t)0x00000800)        /*!< Bit 3 */

#define  XMC_BK1TMGWR4_CLKPSC               ((uint32_t)0x00F00000)        /*!< CLKDIV[3:0] bits (Clock divide ratio) */
#define  XMC_BK1TMGWR4_CLKPSC_0             ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR4_CLKPSC_1             ((uint32_t)0x00200000)        /*!< Bit 1 */
#define  XMC_BK1TMGWR4_CLKPSC_2             ((uint32_t)0x00400000)        /*!< Bit 2 */
#define  XMC_BK1TMGWR4_CLKPSC_3             ((uint32_t)0x00800000)        /*!< Bit 3 */

#define  XMC_BK1TMGWR4_DTSTBL               ((uint32_t)0x0F000000)        /*!< DATLA[3:0] bits (Data latency) */
#define  XMC_BK1TMGWR4_DTSTBL_0             ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR4_DTSTBL_1             ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK1TMGWR4_DTSTBL_2             ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK1TMGWR4_DTSTBL_3             ((uint32_t)0x08000000)        /*!< Bit 3 */

#define  XMC_BK1TMGWR4_MODE                 ((uint32_t)0x30000000)        /*!< ACCMOD[1:0] bits (Access mode) */
#define  XMC_BK1TMGWR4_MODE_0               ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  XMC_BK1TMGWR4_MODE_1               ((uint32_t)0x20000000)        /*!< Bit 1 */

/******************  Bit definition for XMC_BK2CTRL register  *******************/
#define  XMC_BK2CTRL_WAITEN                 ((uint32_t)0x00000002)        /*!< Wait feature enable bit */
#define  XMC_BK2CTRL_EN                     ((uint32_t)0x00000004)        /*!< PC Card/NAND Flash memory bank enable bit */
#define  XMC_BK2CTRL_DEV                    ((uint32_t)0x00000008)        /*!< Memory type */

#define  XMC_BK2CTRL_BUSTYPE                ((uint32_t)0x00000030)        /*!< PWID[1:0] bits (NAND Flash databus width) */
#define  XMC_BK2CTRL_BUSTYPE_0              ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK2CTRL_BUSTYPE_1              ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  XMC_BK2CTRL_ECCEN                  ((uint32_t)0x00000040)        /*!< ECC computation logic enable bit */

#define  XMC_BK2CTRL_DLYCR                  ((uint32_t)0x00001E00)        /*!< TCLR[3:0] bits (CLE to RE delay) */
#define  XMC_BK2CTRL_DLYCR_0                ((uint32_t)0x00000200)        /*!< Bit 0 */
#define  XMC_BK2CTRL_DLYCR_1                ((uint32_t)0x00000400)        /*!< Bit 1 */
#define  XMC_BK2CTRL_DLYCR_2                ((uint32_t)0x00000800)        /*!< Bit 2 */
#define  XMC_BK2CTRL_DLYCR_3                ((uint32_t)0x00001000)        /*!< Bit 3 */

#define  XMC_BK2CTRL_DLYAR                  ((uint32_t)0x0001E000)        /*!< TAR[3:0] bits (ALE to RE delay) */
#define  XMC_BK2CTRL_DLYAR_0                ((uint32_t)0x00002000)        /*!< Bit 0 */
#define  XMC_BK2CTRL_DLYAR_1                ((uint32_t)0x00004000)        /*!< Bit 1 */
#define  XMC_BK2CTRL_DLYAR_2                ((uint32_t)0x00008000)        /*!< Bit 2 */
#define  XMC_BK2CTRL_DLYAR_3                ((uint32_t)0x00010000)        /*!< Bit 3 */

#define  XMC_BK2CTRL_ECCPGSIZE              ((uint32_t)0x000E0000)        /*!< ECCPS[1:0] bits (ECC page size) */
#define  XMC_BK2CTRL_ECCPGSIZE_0            ((uint32_t)0x00020000)        /*!< Bit 0 */
#define  XMC_BK2CTRL_ECCPGSIZE_1            ((uint32_t)0x00040000)        /*!< Bit 1 */
#define  XMC_BK2CTRL_ECCPGSIZE_2            ((uint32_t)0x00080000)        /*!< Bit 2 */

/******************  Bit definition for XMC_BK3CTRL register  *******************/
#define  XMC_BK3CTRL_WAITEN                 ((uint32_t)0x00000002)        /*!< Wait feature enable bit */
#define  XMC_BK3CTRL_EN                     ((uint32_t)0x00000004)        /*!< PC Card/NAND Flash memory bank enable bit */
#define  XMC_BK3CTRL_DEV                    ((uint32_t)0x00000008)        /*!< Memory type */

#define  XMC_BK3CTRL_BUSTYPE                ((uint32_t)0x00000030)        /*!< PWID[1:0] bits (NAND Flash databus width) */
#define  XMC_BK3CTRL_BUSTYPE_0              ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK3CTRL_BUSTYPE_1              ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  XMC_BK3CTRL_ECCEN                  ((uint32_t)0x00000040)        /*!< ECC computation logic enable bit */

#define  XMC_BK3CTRL_DLYCR                  ((uint32_t)0x00001E00)        /*!< TCLR[3:0] bits (CLE to RE delay) */
#define  XMC_BK3CTRL_DLYCR_0                ((uint32_t)0x00000200)        /*!< Bit 0 */
#define  XMC_BK3CTRL_DLYCR_1                ((uint32_t)0x00000400)        /*!< Bit 1 */
#define  XMC_BK3CTRL_DLYCR_2                ((uint32_t)0x00000800)        /*!< Bit 2 */
#define  XMC_BK3CTRL_DLYCR_3                ((uint32_t)0x00001000)        /*!< Bit 3 */

#define  XMC_BK3CTRL_DLYAR                  ((uint32_t)0x0001E000)        /*!< TAR[3:0] bits (ALE to RE delay) */
#define  XMC_BK3CTRL_DLYAR_0                ((uint32_t)0x00002000)        /*!< Bit 0 */
#define  XMC_BK3CTRL_DLYAR_1                ((uint32_t)0x00004000)        /*!< Bit 1 */
#define  XMC_BK3CTRL_DLYAR_2                ((uint32_t)0x00008000)        /*!< Bit 2 */
#define  XMC_BK3CTRL_DLYAR_3                ((uint32_t)0x00010000)        /*!< Bit 3 */

#define  XMC_BK3CTRL_ECCPGSIZE              ((uint32_t)0x000E0000)        /*!< ECCPS[2:0] bits (ECC page size) */
#define  XMC_BK3CTRL_ECCPGSIZE_0            ((uint32_t)0x00020000)        /*!< Bit 0 */
#define  XMC_BK3CTRL_ECCPGSIZE_1            ((uint32_t)0x00040000)        /*!< Bit 1 */
#define  XMC_BK3CTRL_ECCPGSIZE_2            ((uint32_t)0x00080000)        /*!< Bit 2 */

/******************  Bit definition for XMC_BK4CTRL register  *******************/
#define  XMC_BK4CTRL_WAITEN                 ((uint32_t)0x00000002)        /*!< Wait feature enable bit */
#define  XMC_BK4CTRL_EN                     ((uint32_t)0x00000004)        /*!< PC Card/NAND Flash memory bank enable bit */
#define  XMC_BK4CTRL_DEV                    ((uint32_t)0x00000008)        /*!< Memory type */

#define  XMC_BK4CTRL_BUSTYPE                ((uint32_t)0x00000030)        /*!< PWID[1:0] bits (NAND Flash databus width) */
#define  XMC_BK4CTRL_BUSTYPE_0              ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  XMC_BK4CTRL_BUSTYPE_1              ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  XMC_BK4CTRL_ECCEN                  ((uint32_t)0x00000040)        /*!< ECC computation logic enable bit */

#define  XMC_BK4CTRL_DLYCR                  ((uint32_t)0x00001E00)        /*!< TCLR[3:0] bits (CLE to RE delay) */
#define  XMC_BK4CTRL_DLYCR_0                ((uint32_t)0x00000200)        /*!< Bit 0 */
#define  XMC_BK4CTRL_DLYCR_1                ((uint32_t)0x00000400)        /*!< Bit 1 */
#define  XMC_BK4CTRL_DLYCR_2                ((uint32_t)0x00000800)        /*!< Bit 2 */
#define  XMC_BK4CTRL_DLYCR_3                ((uint32_t)0x00001000)        /*!< Bit 3 */

#define  XMC_BK4CTRL_DLYAR                  ((uint32_t)0x0001E000)        /*!< TAR[3:0] bits (ALE to RE delay) */
#define  XMC_BK4CTRL_DLYAR_0                ((uint32_t)0x00002000)        /*!< Bit 0 */
#define  XMC_BK4CTRL_DLYAR_1                ((uint32_t)0x00004000)        /*!< Bit 1 */
#define  XMC_BK4CTRL_DLYAR_2                ((uint32_t)0x00008000)        /*!< Bit 2 */
#define  XMC_BK4CTRL_DLYAR_3                ((uint32_t)0x00010000)        /*!< Bit 3 */

#define  XMC_BK4CTRL_ECCPGSIZE              ((uint32_t)0x000E0000)        /*!< ECCPS[2:0] bits (ECC page size) */
#define  XMC_BK4CTRL_ECCPGSIZE_0            ((uint32_t)0x00020000)        /*!< Bit 0 */
#define  XMC_BK4CTRL_ECCPGSIZE_1            ((uint32_t)0x00040000)        /*!< Bit 1 */
#define  XMC_BK4CTRL_ECCPGSIZE_2            ((uint32_t)0x00080000)        /*!< Bit 2 */

/*******************  Bit definition for XMC_BK2STS register  *******************/
#define  XMC_BK2STS_IRF                     ((uint8_t)0x01)               /*!< Interrupt Rising Edge status */
#define  XMC_BK2STS_IHLF                    ((uint8_t)0x02)               /*!< Interrupt Level status */
#define  XMC_BK2STS_IFF                     ((uint8_t)0x04)               /*!< Interrupt Falling Edge status */
#define  XMC_BK2STS_IREN                    ((uint8_t)0x08)               /*!< Interrupt Rising Edge detection Enable bit */
#define  XMC_BK2STS_IHLEN                   ((uint8_t)0x10)               /*!< Interrupt Level detection Enable bit */
#define  XMC_BK2STS_IFEN                    ((uint8_t)0x20)               /*!< Interrupt Falling Edge detection Enable bit */
#define  XMC_BK2STS_FIFOE                   ((uint8_t)0x40)               /*!< FIFO empty */

/*******************  Bit definition for XMC_BK3STS register  *******************/
#define  XMC_BK3STS_IRF                     ((uint8_t)0x01)               /*!< Interrupt Rising Edge status */
#define  XMC_BK3STS_IHLF                    ((uint8_t)0x02)               /*!< Interrupt Level status */
#define  XMC_BK3STS_IFF                     ((uint8_t)0x04)               /*!< Interrupt Falling Edge status */
#define  XMC_BK3STS_IREN                    ((uint8_t)0x08)               /*!< Interrupt Rising Edge detection Enable bit */
#define  XMC_BK3STS_IHLEN                   ((uint8_t)0x10)               /*!< Interrupt Level detection Enable bit */
#define  XMC_BK3STS_IFEN                    ((uint8_t)0x20)               /*!< Interrupt Falling Edge detection Enable bit */
#define  XMC_BK3STS_FIFOE                   ((uint8_t)0x40)               /*!< FIFO empty */

/*******************  Bit definition for XMC_BK4STS register  *******************/
#define  XMC_BK4STS_IRF                     ((uint8_t)0x01)               /*!< Interrupt Rising Edge status */
#define  XMC_BK4STS_IHLF                    ((uint8_t)0x02)               /*!< Interrupt Level status */
#define  XMC_BK4STS_IFF                     ((uint8_t)0x04)               /*!< Interrupt Falling Edge status */
#define  XMC_BK4STS_IREN                    ((uint8_t)0x08)               /*!< Interrupt Rising Edge detection Enable bit */
#define  XMC_BK4STS_IHLEN                   ((uint8_t)0x10)               /*!< Interrupt Level detection Enable bit */
#define  XMC_BK4STS_IFEN                    ((uint8_t)0x20)               /*!< Interrupt Falling Edge detection Enable bit */
#define  XMC_BK4STS_FIFOE                   ((uint8_t)0x40)               /*!< FIFO empty */

/******************  Bit definition for XMC_BK2TMGMEM register  ******************/
#define  XMC_BK2TMGMEM_STP                  ((uint32_t)0x000000FF)        /*!< MEMSET2[7:0] bits (Common memory 2 setup time) */
#define  XMC_BK2TMGMEM_STP_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK2TMGMEM_STP_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK2TMGMEM_STP_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK2TMGMEM_STP_3                ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  XMC_BK2TMGMEM_STP_4                ((uint32_t)0x00000010)        /*!< Bit 4 */
#define  XMC_BK2TMGMEM_STP_5                ((uint32_t)0x00000020)        /*!< Bit 5 */
#define  XMC_BK2TMGMEM_STP_6                ((uint32_t)0x00000040)        /*!< Bit 6 */
#define  XMC_BK2TMGMEM_STP_7                ((uint32_t)0x00000080)        /*!< Bit 7 */

#define  XMC_BK2TMGMEM_OP                   ((uint32_t)0x0000FF00)        /*!< MEMWAIT2[7:0] bits (Common memory 2 wait time) */
#define  XMC_BK2TMGMEM_OP_0                 ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK2TMGMEM_OP_1                 ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK2TMGMEM_OP_2                 ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK2TMGMEM_OP_3                 ((uint32_t)0x00000800)        /*!< Bit 3 */
#define  XMC_BK2TMGMEM_OP_4                 ((uint32_t)0x00001000)        /*!< Bit 4 */
#define  XMC_BK2TMGMEM_OP_5                 ((uint32_t)0x00002000)        /*!< Bit 5 */
#define  XMC_BK2TMGMEM_OP_6                 ((uint32_t)0x00004000)        /*!< Bit 6 */
#define  XMC_BK2TMGMEM_OP_7                 ((uint32_t)0x00008000)        /*!< Bit 7 */

#define  XMC_BK2TMGMEM_HLD                  ((uint32_t)0x00FF0000)        /*!< MEMHOLD2[7:0] bits (Common memory 2 hold time) */
#define  XMC_BK2TMGMEM_HLD_0                ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK2TMGMEM_HLD_1                ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK2TMGMEM_HLD_2                ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK2TMGMEM_HLD_3                ((uint32_t)0x00080000)        /*!< Bit 3 */
#define  XMC_BK2TMGMEM_HLD_4                ((uint32_t)0x00100000)        /*!< Bit 4 */
#define  XMC_BK2TMGMEM_HLD_5                ((uint32_t)0x00200000)        /*!< Bit 5 */
#define  XMC_BK2TMGMEM_HLD_6                ((uint32_t)0x00400000)        /*!< Bit 6 */
#define  XMC_BK2TMGMEM_HLD_7                ((uint32_t)0x00800000)        /*!< Bit 7 */

#define  XMC_BK2TMGMEM_WRSTP                ((uint32_t)0xFF000000)        /*!< MEMHIZ2[7:0] bits (Common memory 2 databus HiZ time) */
#define  XMC_BK2TMGMEM_WRSTP_0              ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK2TMGMEM_WRSTP_1              ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK2TMGMEM_WRSTP_2              ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK2TMGMEM_WRSTP_3              ((uint32_t)0x08000000)        /*!< Bit 3 */
#define  XMC_BK2TMGMEM_WRSTP_4              ((uint32_t)0x10000000)        /*!< Bit 4 */
#define  XMC_BK2TMGMEM_WRSTP_5              ((uint32_t)0x20000000)        /*!< Bit 5 */
#define  XMC_BK2TMGMEM_WRSTP_6              ((uint32_t)0x40000000)        /*!< Bit 6 */
#define  XMC_BK2TMGMEM_WRSTP_7              ((uint32_t)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for XMC_BK3TMGMEM register  ******************/
#define  XMC_BK3TMGMEM_STP                  ((uint32_t)0x000000FF)        /*!< MEMSET3[7:0] bits (Common memory 3 setup time) */
#define  XMC_BK3TMGMEM_STP_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK3TMGMEM_STP_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK3TMGMEM_STP_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK3TMGMEM_STP_3                ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  XMC_BK3TMGMEM_STP_4                ((uint32_t)0x00000010)        /*!< Bit 4 */
#define  XMC_BK3TMGMEM_STP_5                ((uint32_t)0x00000020)        /*!< Bit 5 */
#define  XMC_BK3TMGMEM_STP_6                ((uint32_t)0x00000040)        /*!< Bit 6 */
#define  XMC_BK3TMGMEM_STP_7                ((uint32_t)0x00000080)        /*!< Bit 7 */

#define  XMC_BK3TMGMEM_OP                   ((uint32_t)0x0000FF00)        /*!< MEMWAIT3[7:0] bits (Common memory 3 wait time) */
#define  XMC_BK3TMGMEM_OP_0                 ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK3TMGMEM_OP_1                 ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK3TMGMEM_OP_2                 ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK3TMGMEM_OP_3                 ((uint32_t)0x00000800)        /*!< Bit 3 */
#define  XMC_BK3TMGMEM_OP_4                 ((uint32_t)0x00001000)        /*!< Bit 4 */
#define  XMC_BK3TMGMEM_OP_5                 ((uint32_t)0x00002000)        /*!< Bit 5 */
#define  XMC_BK3TMGMEM_OP_6                 ((uint32_t)0x00004000)        /*!< Bit 6 */
#define  XMC_BK3TMGMEM_OP_7                 ((uint32_t)0x00008000)        /*!< Bit 7 */

#define  XMC_BK3TMGMEM_HLD                  ((uint32_t)0x00FF0000)        /*!< MEMHOLD3[7:0] bits (Common memory 3 hold time) */
#define  XMC_BK3TMGMEM_HLD_0                ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK3TMGMEM_HLD_1                ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK3TMGMEM_HLD_2                ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK3TMGMEM_HLD_3                ((uint32_t)0x00080000)        /*!< Bit 3 */
#define  XMC_BK3TMGMEM_HLD_4                ((uint32_t)0x00100000)        /*!< Bit 4 */
#define  XMC_BK3TMGMEM_HLD_5                ((uint32_t)0x00200000)        /*!< Bit 5 */
#define  XMC_BK3TMGMEM_HLD_6                ((uint32_t)0x00400000)        /*!< Bit 6 */
#define  XMC_BK3TMGMEM_HLD_7                ((uint32_t)0x00800000)        /*!< Bit 7 */

#define  XMC_BK3TMGMEM_WRSTP                ((uint32_t)0xFF000000)        /*!< MEMHIZ3[7:0] bits (Common memory 3 databus HiZ time) */
#define  XMC_BK3TMGMEM_WRSTP_0              ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK3TMGMEM_WRSTP_1              ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK3TMGMEM_WRSTP_2              ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK3TMGMEM_WRSTP_3              ((uint32_t)0x08000000)        /*!< Bit 3 */
#define  XMC_BK3TMGMEM_WRSTP_4              ((uint32_t)0x10000000)        /*!< Bit 4 */
#define  XMC_BK3TMGMEM_WRSTP_5              ((uint32_t)0x20000000)        /*!< Bit 5 */
#define  XMC_BK3TMGMEM_WRSTP_6              ((uint32_t)0x40000000)        /*!< Bit 6 */
#define  XMC_BK3TMGMEM_WRSTP_7              ((uint32_t)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for XMC_BK4TMGMEM register  ******************/
#define  XMC_BK4TMGMEM_STP                  ((uint32_t)0x000000FF)        /*!< MEMSET4[7:0] bits (Common memory 4 setup time) */
#define  XMC_BK4TMGMEM_STP_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK4TMGMEM_STP_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK4TMGMEM_STP_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK4TMGMEM_STP_3                ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  XMC_BK4TMGMEM_STP_4                ((uint32_t)0x00000010)        /*!< Bit 4 */
#define  XMC_BK4TMGMEM_STP_5                ((uint32_t)0x00000020)        /*!< Bit 5 */
#define  XMC_BK4TMGMEM_STP_6                ((uint32_t)0x00000040)        /*!< Bit 6 */
#define  XMC_BK4TMGMEM_STP_7                ((uint32_t)0x00000080)        /*!< Bit 7 */

#define  XMC_BK4TMGMEM_OP                   ((uint32_t)0x0000FF00)        /*!< MEMWAIT4[7:0] bits (Common memory 4 wait time) */
#define  XMC_BK4TMGMEM_OP_0                 ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK4TMGMEM_OP_1                 ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK4TMGMEM_OP_2                 ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK4TMGMEM_OP_3                 ((uint32_t)0x00000800)        /*!< Bit 3 */
#define  XMC_BK4TMGMEM_OP_4                 ((uint32_t)0x00001000)        /*!< Bit 4 */
#define  XMC_BK4TMGMEM_OP_5                 ((uint32_t)0x00002000)        /*!< Bit 5 */
#define  XMC_BK4TMGMEM_OP_6                 ((uint32_t)0x00004000)        /*!< Bit 6 */
#define  XMC_BK4TMGMEM_OP_7                 ((uint32_t)0x00008000)        /*!< Bit 7 */

#define  XMC_BK4TMGMEM_HLD                  ((uint32_t)0x00FF0000)        /*!< MEMHOLD4[7:0] bits (Common memory 4 hold time) */
#define  XMC_BK4TMGMEM_HLD_0                ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK4TMGMEM_HLD_1                ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK4TMGMEM_HLD_2                ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK4TMGMEM_HLD_3                ((uint32_t)0x00080000)        /*!< Bit 3 */
#define  XMC_BK4TMGMEM_HLD_4                ((uint32_t)0x00100000)        /*!< Bit 4 */
#define  XMC_BK4TMGMEM_HLD_5                ((uint32_t)0x00200000)        /*!< Bit 5 */
#define  XMC_BK4TMGMEM_HLD_6                ((uint32_t)0x00400000)        /*!< Bit 6 */
#define  XMC_BK4TMGMEM_HLD_7                ((uint32_t)0x00800000)        /*!< Bit 7 */

#define  XMC_BK4TMGMEM_WRSTP                ((uint32_t)0xFF000000)        /*!< MEMHIZ4[7:0] bits (Common memory 4 databus HiZ time) */
#define  XMC_BK4TMGMEM_WRSTP_0              ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK4TMGMEM_WRSTP_1              ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK4TMGMEM_WRSTP_2              ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK4TMGMEM_WRSTP_3              ((uint32_t)0x08000000)        /*!< Bit 3 */
#define  XMC_BK4TMGMEM_WRSTP_4              ((uint32_t)0x10000000)        /*!< Bit 4 */
#define  XMC_BK4TMGMEM_WRSTP_5              ((uint32_t)0x20000000)        /*!< Bit 5 */
#define  XMC_BK4TMGMEM_WRSTP_6              ((uint32_t)0x40000000)        /*!< Bit 6 */
#define  XMC_BK4TMGMEM_WRSTP_7              ((uint32_t)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for XMC_BK2TMGATT register  ******************/
#define  XMC_BK2TMGATT_STP                  ((uint32_t)0x000000FF)        /*!< ATTSET2[7:0] bits (Attribute memory 2 setup time) */
#define  XMC_BK2TMGATT_STP_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK2TMGATT_STP_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK2TMGATT_STP_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK2TMGATT_STP_3                ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  XMC_BK2TMGATT_STP_4                ((uint32_t)0x00000010)        /*!< Bit 4 */
#define  XMC_BK2TMGATT_STP_5                ((uint32_t)0x00000020)        /*!< Bit 5 */
#define  XMC_BK2TMGATT_STP_6                ((uint32_t)0x00000040)        /*!< Bit 6 */
#define  XMC_BK2TMGATT_STP_7                ((uint32_t)0x00000080)        /*!< Bit 7 */

#define  XMC_BK2TMGATT_OP                   ((uint32_t)0x0000FF00)        /*!< ATTWAIT2[7:0] bits (Attribute memory 2 wait time) */
#define  XMC_BK2TMGATT_OP_0                 ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK2TMGATT_OP_1                 ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK2TMGATT_OP_2                 ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK2TMGATT_OP_3                 ((uint32_t)0x00000800)        /*!< Bit 3 */
#define  XMC_BK2TMGATT_OP_4                 ((uint32_t)0x00001000)        /*!< Bit 4 */
#define  XMC_BK2TMGATT_OP_5                 ((uint32_t)0x00002000)        /*!< Bit 5 */
#define  XMC_BK2TMGATT_OP_6                 ((uint32_t)0x00004000)        /*!< Bit 6 */
#define  XMC_BK2TMGATT_OP_7                 ((uint32_t)0x00008000)        /*!< Bit 7 */

#define  XMC_BK2TMGATT_HLD                  ((uint32_t)0x00FF0000)        /*!< ATTHOLD2[7:0] bits (Attribute memory 2 hold time) */
#define  XMC_BK2TMGATT_HLD_0                ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK2TMGATT_HLD_1                ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK2TMGATT_HLD_2                ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK2TMGATT_HLD_3                ((uint32_t)0x00080000)        /*!< Bit 3 */
#define  XMC_BK2TMGATT_HLD_4                ((uint32_t)0x00100000)        /*!< Bit 4 */
#define  XMC_BK2TMGATT_HLD_5                ((uint32_t)0x00200000)        /*!< Bit 5 */
#define  XMC_BK2TMGATT_HLD_6                ((uint32_t)0x00400000)        /*!< Bit 6 */
#define  XMC_BK2TMGATT_HLD_7                ((uint32_t)0x00800000)        /*!< Bit 7 */

#define  XMC_BK2TMGATT_WRSTP                ((uint32_t)0xFF000000)        /*!< ATTHIZ2[7:0] bits (Attribute memory 2 databus HiZ time) */
#define  XMC_BK2TMGATT_WRSTP_0              ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK2TMGATT_WRSTP_1              ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK2TMGATT_WRSTP_2              ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK2TMGATT_WRSTP_3              ((uint32_t)0x08000000)        /*!< Bit 3 */
#define  XMC_BK2TMGATT_WRSTP_4              ((uint32_t)0x10000000)        /*!< Bit 4 */
#define  XMC_BK2TMGATT_WRSTP_5              ((uint32_t)0x20000000)        /*!< Bit 5 */
#define  XMC_BK2TMGATT_WRSTP_6              ((uint32_t)0x40000000)        /*!< Bit 6 */
#define  XMC_BK2TMGATT_WRSTP_7              ((uint32_t)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for XMC_BK3TMGATT register  ******************/
#define  XMC_BK3TMGATT_STP                  ((uint32_t)0x000000FF)        /*!< ATTSET3[7:0] bits (Attribute memory 3 setup time) */
#define  XMC_BK3TMGATT_STP_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK3TMGATT_STP_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK3TMGATT_STP_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK3TMGATT_STP_3                ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  XMC_BK3TMGATT_STP_4                ((uint32_t)0x00000010)        /*!< Bit 4 */
#define  XMC_BK3TMGATT_STP_5                ((uint32_t)0x00000020)        /*!< Bit 5 */
#define  XMC_BK3TMGATT_STP_6                ((uint32_t)0x00000040)        /*!< Bit 6 */
#define  XMC_BK3TMGATT_STP_7                ((uint32_t)0x00000080)        /*!< Bit 7 */

#define  XMC_BK3TMGATT_OP                   ((uint32_t)0x0000FF00)        /*!< ATTWAIT3[7:0] bits (Attribute memory 3 wait time) */
#define  XMC_BK3TMGATT_OP_0                 ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK3TMGATT_OP_1                 ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK3TMGATT_OP_2                 ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK3TMGATT_OP_3                 ((uint32_t)0x00000800)        /*!< Bit 3 */
#define  XMC_BK3TMGATT_OP_4                 ((uint32_t)0x00001000)        /*!< Bit 4 */
#define  XMC_BK3TMGATT_OP_5                 ((uint32_t)0x00002000)        /*!< Bit 5 */
#define  XMC_BK3TMGATT_OP_6                 ((uint32_t)0x00004000)        /*!< Bit 6 */
#define  XMC_BK3TMGATT_OP_7                 ((uint32_t)0x00008000)        /*!< Bit 7 */

#define  XMC_BK3TMGATT_HLD                  ((uint32_t)0x00FF0000)        /*!< ATTHOLD3[7:0] bits (Attribute memory 3 hold time) */
#define  XMC_BK3TMGATT_HLD_0                ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK3TMGATT_HLD_1                ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK3TMGATT_HLD_2                ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK3TMGATT_HLD_3                ((uint32_t)0x00080000)        /*!< Bit 3 */
#define  XMC_BK3TMGATT_HLD_4                ((uint32_t)0x00100000)        /*!< Bit 4 */
#define  XMC_BK3TMGATT_HLD_5                ((uint32_t)0x00200000)        /*!< Bit 5 */
#define  XMC_BK3TMGATT_HLD_6                ((uint32_t)0x00400000)        /*!< Bit 6 */
#define  XMC_BK3TMGATT_HLD_7                ((uint32_t)0x00800000)        /*!< Bit 7 */

#define  XMC_BK3TMGATT_WRSTP                ((uint32_t)0xFF000000)        /*!< ATTHIZ3[7:0] bits (Attribute memory 3 databus HiZ time) */
#define  XMC_BK3TMGATT_WRSTP_0              ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK3TMGATT_WRSTP_1              ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK3TMGATT_WRSTP_2              ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK3TMGATT_WRSTP_3              ((uint32_t)0x08000000)        /*!< Bit 3 */
#define  XMC_BK3TMGATT_WRSTP_4              ((uint32_t)0x10000000)        /*!< Bit 4 */
#define  XMC_BK3TMGATT_WRSTP_5              ((uint32_t)0x20000000)        /*!< Bit 5 */
#define  XMC_BK3TMGATT_WRSTP_6              ((uint32_t)0x40000000)        /*!< Bit 6 */
#define  XMC_BK3TMGATT_WRSTP_7              ((uint32_t)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for XMC_BK4TMGATT register  ******************/
#define  XMC_BK4TMGATT_STP                  ((uint32_t)0x000000FF)        /*!< ATTSET4[7:0] bits (Attribute memory 4 setup time) */
#define  XMC_BK4TMGATT_STP_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK4TMGATT_STP_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK4TMGATT_STP_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK4TMGATT_STP_3                ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  XMC_BK4TMGATT_STP_4                ((uint32_t)0x00000010)        /*!< Bit 4 */
#define  XMC_BK4TMGATT_STP_5                ((uint32_t)0x00000020)        /*!< Bit 5 */
#define  XMC_BK4TMGATT_STP_6                ((uint32_t)0x00000040)        /*!< Bit 6 */
#define  XMC_BK4TMGATT_STP_7                ((uint32_t)0x00000080)        /*!< Bit 7 */

#define  XMC_BK4TMGATT_OP                   ((uint32_t)0x0000FF00)        /*!< ATTWAIT4[7:0] bits (Attribute memory 4 wait time) */
#define  XMC_BK4TMGATT_OP_0                 ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK4TMGATT_OP_1                 ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK4TMGATT_OP_2                 ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK4TMGATT_OP_3                 ((uint32_t)0x00000800)        /*!< Bit 3 */
#define  XMC_BK4TMGATT_OP_4                 ((uint32_t)0x00001000)        /*!< Bit 4 */
#define  XMC_BK4TMGATT_OP_5                 ((uint32_t)0x00002000)        /*!< Bit 5 */
#define  XMC_BK4TMGATT_OP_6                 ((uint32_t)0x00004000)        /*!< Bit 6 */
#define  XMC_BK4TMGATT_OP_7                 ((uint32_t)0x00008000)        /*!< Bit 7 */

#define  XMC_BK4TMGATT_HLD                  ((uint32_t)0x00FF0000)        /*!< ATTHOLD4[7:0] bits (Attribute memory 4 hold time) */
#define  XMC_BK4TMGATT_HLD_0                ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK4TMGATT_HLD_1                ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK4TMGATT_HLD_2                ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK4TMGATT_HLD_3                ((uint32_t)0x00080000)        /*!< Bit 3 */
#define  XMC_BK4TMGATT_HLD_4                ((uint32_t)0x00100000)        /*!< Bit 4 */
#define  XMC_BK4TMGATT_HLD_5                ((uint32_t)0x00200000)        /*!< Bit 5 */
#define  XMC_BK4TMGATT_HLD_6                ((uint32_t)0x00400000)        /*!< Bit 6 */
#define  XMC_BK4TMGATT_HLD_7                ((uint32_t)0x00800000)        /*!< Bit 7 */

#define  XMC_BK4TMGATT_WRSTP                ((uint32_t)0xFF000000)        /*!< ATTHIZ4[7:0] bits (Attribute memory 4 databus HiZ time) */
#define  XMC_BK4TMGATT_WRSTP_0              ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK4TMGATT_WRSTP_1              ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK4TMGATT_WRSTP_2              ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK4TMGATT_WRSTP_3              ((uint32_t)0x08000000)        /*!< Bit 3 */
#define  XMC_BK4TMGATT_WRSTP_4              ((uint32_t)0x10000000)        /*!< Bit 4 */
#define  XMC_BK4TMGATT_WRSTP_5              ((uint32_t)0x20000000)        /*!< Bit 5 */
#define  XMC_BK4TMGATT_WRSTP_6              ((uint32_t)0x40000000)        /*!< Bit 6 */
#define  XMC_BK4TMGATT_WRSTP_7              ((uint32_t)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for XMC_BK4TMGIO register  *******************/
#define  XMC_BK4TMGIO_STP                   ((uint32_t)0x000000FF)        /*!< IOSET4[7:0] bits (I/O 4 setup time) */
#define  XMC_BK4TMGIO_STP_0                 ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  XMC_BK4TMGIO_STP_1                 ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  XMC_BK4TMGIO_STP_2                 ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  XMC_BK4TMGIO_STP_3                 ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  XMC_BK4TMGIO_STP_4                 ((uint32_t)0x00000010)        /*!< Bit 4 */
#define  XMC_BK4TMGIO_STP_5                 ((uint32_t)0x00000020)        /*!< Bit 5 */
#define  XMC_BK4TMGIO_STP_6                 ((uint32_t)0x00000040)        /*!< Bit 6 */
#define  XMC_BK4TMGIO_STP_7                 ((uint32_t)0x00000080)        /*!< Bit 7 */

#define  XMC_BK4TMGIO_OP                    ((uint32_t)0x0000FF00)        /*!< IOWAIT4[7:0] bits (I/O 4 wait time) */
#define  XMC_BK4TMGIO_OP_0                  ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  XMC_BK4TMGIO_OP_1                  ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  XMC_BK4TMGIO_OP_2                  ((uint32_t)0x00000400)        /*!< Bit 2 */
#define  XMC_BK4TMGIO_OP_3                  ((uint32_t)0x00000800)        /*!< Bit 3 */
#define  XMC_BK4TMGIO_OP_4                  ((uint32_t)0x00001000)        /*!< Bit 4 */
#define  XMC_BK4TMGIO_OP_5                  ((uint32_t)0x00002000)        /*!< Bit 5 */
#define  XMC_BK4TMGIO_OP_6                  ((uint32_t)0x00004000)        /*!< Bit 6 */
#define  XMC_BK4TMGIO_OP_7                  ((uint32_t)0x00008000)        /*!< Bit 7 */

#define  XMC_BK4TMGIO_HLD                   ((uint32_t)0x00FF0000)        /*!< IOHOLD4[7:0] bits (I/O 4 hold time) */
#define  XMC_BK4TMGIO_HLD_0                 ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  XMC_BK4TMGIO_HLD_1                 ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  XMC_BK4TMGIO_HLD_2                 ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  XMC_BK4TMGIO_HLD_3                 ((uint32_t)0x00080000)        /*!< Bit 3 */
#define  XMC_BK4TMGIO_HLD_4                 ((uint32_t)0x00100000)        /*!< Bit 4 */
#define  XMC_BK4TMGIO_HLD_5                 ((uint32_t)0x00200000)        /*!< Bit 5 */
#define  XMC_BK4TMGIO_HLD_6                 ((uint32_t)0x00400000)        /*!< Bit 6 */
#define  XMC_BK4TMGIO_HLD_7                 ((uint32_t)0x00800000)        /*!< Bit 7 */

#define  XMC_BK4TMGIO_WRSTP                 ((uint32_t)0xFF000000)        /*!< IOHIZ4[7:0] bits (I/O 4 databus HiZ time) */
#define  XMC_BK4TMGIO_WRSTP_0               ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  XMC_BK4TMGIO_WRSTP_1               ((uint32_t)0x02000000)        /*!< Bit 1 */
#define  XMC_BK4TMGIO_WRSTP_2               ((uint32_t)0x04000000)        /*!< Bit 2 */
#define  XMC_BK4TMGIO_WRSTP_3               ((uint32_t)0x08000000)        /*!< Bit 3 */
#define  XMC_BK4TMGIO_WRSTP_4               ((uint32_t)0x10000000)        /*!< Bit 4 */
#define  XMC_BK4TMGIO_WRSTP_5               ((uint32_t)0x20000000)        /*!< Bit 5 */
#define  XMC_BK4TMGIO_WRSTP_6               ((uint32_t)0x40000000)        /*!< Bit 6 */
#define  XMC_BK4TMGIO_WRSTP_7               ((uint32_t)0x80000000)        /*!< Bit 7 */

/******************  Bit definition for XMC_BK2ECC register  ******************/
#define  XMC_BK2ECC_ECC                     ((uint32_t)0xFFFFFFFF)        /*!< ECC result */

/******************  Bit definition for XMC_BK3ECC register  ******************/
#define  XMC_BK3ECC_ECC                     ((uint32_t)0xFFFFFFFF)        /*!< ECC result */

/******************************************************************************/
/*                                                                            */
/*                          SD host Interface                                 */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for SDIO_POWER register  ******************/
#define  SDIO_POWER_PWRCTRL                 ((uint8_t)0x03)               /*!< PWRCTRL[1:0] bits (Power supply control bits) */
#define  SDIO_POWER_PWRCTRL_0               ((uint8_t)0x01)               /*!< Bit 0 */
#define  SDIO_POWER_PWRCTRL_1               ((uint8_t)0x02)               /*!< Bit 1 */

/******************  Bit definition for SDIO_CLKCTRL register  ******************/
#define  SDIO_CLKCTRL_CLKPSC                ((uint16_t)0x00FF)            /*!< Clock divide factor */
#define  SDIO_CLKCTRL_CLKEN                 ((uint16_t)0x0100)            /*!< Clock enable bit */
#define  SDIO_CLKCTRL_PWRSVG                ((uint16_t)0x0200)            /*!< Power saving configuration bit */
#define  SDIO_CLKCTRL_BYPS                  ((uint16_t)0x0400)            /*!< Clock divider bypass enable bit */

#define  SDIO_CLKCTRL_BUSWIDTH              ((uint16_t)0x1800)            /*!< WIDBUS[1:0] bits (Wide bus mode enable bit) */
#define  SDIO_CLKCR_WIDBUS_0                ((uint16_t)0x0800)            /*!< Bit 0 */
#define  SDIO_CLKCR_WIDBUS_1                ((uint16_t)0x1000)            /*!< Bit 1 */

#define  SDIO_CLKCTRL_CLKEDG                ((uint16_t)0x2000)            /*!< SDIO_CK dephasing selection bit */
#define  SDIO_CLKCTRL_FLWCTRLEN             ((uint16_t)0x4000)            /*!< HW Flow Control enable */

/*******************  Bit definition for SDIO_ARG register  *******************/
#define  SDIO_ARG_ARG                       ((uint32_t)0xFFFFFFFF)            /*!< Command argument */

/*******************  Bit definition for SDIO_CMD register  *******************/
#define  SDIO_CMD_CMDIDX                    ((uint16_t)0x003F)            /*!< Command Index */

#define  SDIO_CMD_RSPWT                     ((uint16_t)0x00C0)            /*!< WAITRESP[1:0] bits (Wait for response bits) */
#define  SDIO_CMD_WAITRESP_0                ((uint16_t)0x0040)            /*!<  Bit 0 */
#define  SDIO_CMD_WAITRESP_1                ((uint16_t)0x0080)            /*!<  Bit 1 */

#define  SDIO_CMD_INTWT                     ((uint16_t)0x0100)            /*!< CPSM Waits for Interrupt Request */
#define  SDIO_CMD_PNDWT                     ((uint16_t)0x0200)            /*!< CPSM Waits for ends of data transfer (CmdPend internal signal) */
#define  SDIO_CMD_CMDMEN                    ((uint16_t)0x0400)            /*!< Command path state machine (CPSM) Enable bit */
#define  SDIO_CMD_SDIOSUSP                  ((uint16_t)0x0800)            /*!< SD I/O suspend command */
#define  SDIO_CMD_CMPLSGNLEN                ((uint16_t)0x1000)            /*!< Enable CMD completion */
#define  SDIO_CMD_INTDIS                    ((uint16_t)0x2000)            /*!< Interrupt Disable */
#define  SDIO_CMD_ATACMD                    ((uint16_t)0x4000)            /*!< CE-ATA command */

/*****************  Bit definition for SDIO_RSPCMD register  *****************/
#define  SDIO_RSPCMD_RSPCMD                 ((uint8_t)0x3F)               /*!< Response command index */

/******************  Bit definition for SDIO_RESP0 register  ******************/
#define  SDIO_RESP0_CARDSTATUS0             ((uint32_t)0xFFFFFFFF)        /*!< Card Status */

/******************  Bit definition for SDIO_RSP1 register  ******************/
#define  SDIO_RSP1_CARDSTS1                 ((uint32_t)0xFFFFFFFF)        /*!< Card Status */

/******************  Bit definition for SDIO_RSP2 register  ******************/
#define  SDIO_RSP2_CARDSTS2                 ((uint32_t)0xFFFFFFFF)        /*!< Card Status */

/******************  Bit definition for SDIO_RSP3 register  ******************/
#define  SDIO_RSP3_CARDSTS3                 ((uint32_t)0xFFFFFFFF)        /*!< Card Status */

/******************  Bit definition for SDIO_RSP4 register  ******************/
#define  SDIO_RSP4_CARDSTS4                 ((uint32_t)0xFFFFFFFF)        /*!< Card Status */

/******************  Bit definition for SDIO_DTTMR register  *****************/
#define  SDIO_DTTMR_TIMEOUT                 ((uint32_t)0xFFFFFFFF)        /*!< Data timeout period. */

/******************  Bit definition for SDIO_DTLEN register  *******************/
#define  SDIO_DTLEN_DTLEN                   ((uint32_t)0x01FFFFFF)        /*!< Data length value */

/******************  Bit definition for SDIO_DTCTRL register  ******************/
#define  SDIO_DTCTRL_TFREN                  ((uint16_t)0x0001)            /*!< Data transfer enabled bit */
#define  SDIO_DTCTRL_TFRDIR                 ((uint16_t)0x0002)            /*!< Data transfer direction selection */
#define  SDIO_DTCTRL_TFRMODE                ((uint16_t)0x0004)            /*!< Data transfer mode selection */
#define  SDIO_DTCTRL_DMAEN                  ((uint16_t)0x0008)            /*!< DMA enabled bit */

#define  SDIO_DTCTRL_BLKSIZE                ((uint16_t)0x00F0)            /*!< DBLOCKSIZE[3:0] bits (Data block size) */
#define  SDIO_DTCTRL_DBLOCKSIZE_0           ((uint16_t)0x0010)            /*!< Bit 0 */
#define  SDIO_DTCTRL_DBLOCKSIZE_1           ((uint16_t)0x0020)            /*!< Bit 1 */
#define  SDIO_DTCTRL_DBLOCKSIZE_2           ((uint16_t)0x0040)            /*!< Bit 2 */
#define  SDIO_DTCTRL_DBLOCKSIZE_3           ((uint16_t)0x0080)            /*!< Bit 3 */

#define  SDIO_DTCTRL_RDWTSTART              ((uint16_t)0x0100)            /*!< Read wait start */
#define  SDIO_DTCTRL_RDWTSTOP               ((uint16_t)0x0200)            /*!< Read wait stop */
#define  SDIO_DTCTRL_RDWTMODE               ((uint16_t)0x0400)            /*!< Read wait mode */
#define  SDIO_DTCTRL_SDIOEN                 ((uint16_t)0x0800)            /*!< SD I/O enable functions */

/******************  Bit definition for SDIO_DTCNTR register  *****************/
#define  SDIO_DTCNTR_DTCNT                  ((uint32_t)0x01FFFFFF)        /*!< Data count value */

/******************  Bit definition for SDIO_STS register  ********************/
#define  SDIO_STS_CMDFAIL                   ((uint32_t)0x00000001)        /*!< Command response received (CRC check failed) */
#define  SDIO_STS_DTFAIL                    ((uint32_t)0x00000002)        /*!< Data block sent/received (CRC check failed) */
#define  SDIO_STS_CMDTIMEOUT                ((uint32_t)0x00000004)        /*!< Command response timeout */
#define  SDIO_STS_DTTIMEOUT                 ((uint32_t)0x00000008)        /*!< Data timeout */
#define  SDIO_STS_TXERRU                    ((uint32_t)0x00000010)        /*!< Transmit FIFO underrun error */
#define  SDIO_STS_RXERRO                    ((uint32_t)0x00000020)        /*!< Received FIFO overrun error */
#define  SDIO_STS_CMDRSPCMPL                ((uint32_t)0x00000040)        /*!< Command response received (CRC check passed) */
#define  SDIO_STS_CMDCMPL                   ((uint32_t)0x00000080)        /*!< Command sent (no response required) */
#define  SDIO_STS_DTCMPL                    ((uint32_t)0x00000100)        /*!< Data end (data counter, SDIDCOUNT, is zero) */
#define  SDIO_STS_SBITERR                   ((uint32_t)0x00000200)        /*!< Start bit not detected on all data signals in wide bus mode */
#define  SDIO_STS_DTBLKCMPL                 ((uint32_t)0x00000400)        /*!< Data block sent/received (CRC check passed) */
#define  SDIO_STS_DOCMD                     ((uint32_t)0x00000800)        /*!< Command transfer in progress */
#define  SDIO_STS_DOTX                      ((uint32_t)0x00001000)        /*!< Data transmit in progress */
#define  SDIO_STS_DORX                      ((uint32_t)0x00002000)        /*!< Data receive in progress */
#define  SDIO_STS_TXBUF_H                   ((uint32_t)0x00004000)        /*!< Transmit FIFO Half Empty: at least 8 words can be written into the FIFO */
#define  SDIO_STS_RXBUF_H                   ((uint32_t)0x00008000)        /*!< Receive FIFO Half Full: there are at least 8 words in the FIFO */
#define  SDIO_STS_TXBUF_F                   ((uint32_t)0x00010000)        /*!< Transmit FIFO full */
#define  SDIO_STS_RXBUF_F                   ((uint32_t)0x00020000)        /*!< Receive FIFO full */
#define  SDIO_STS_TXBUF_E                   ((uint32_t)0x00040000)        /*!< Transmit FIFO empty */
#define  SDIO_STS_RXBUF_E                   ((uint32_t)0x00080000)        /*!< Receive FIFO empty */
#define  SDIO_STS_TXBUF                     ((uint32_t)0x00100000)        /*!< Data available in transmit FIFO */
#define  SDIO_STS_RXBUF                     ((uint32_t)0x00200000)        /*!< Data available in receive FIFO */
#define  SDIO_STS_SDIOIF                    ((uint32_t)0x00400000)        /*!< SDIO interrupt received */
#define  SDIO_STS_ATACMPL                   ((uint32_t)0x00800000)        /*!< CE-ATA command completion signal received for CMD61 */

/*******************  Bit definition for SDIO_INTCLR register  *******************/
#define  SDIO_INTCLR_CMDFAIL                ((uint32_t)0x00000001)        /*!< CCRCFAIL flag clear bit */
#define  SDIO_INTCLR_DTFAIL                 ((uint32_t)0x00000002)        /*!< DCRCFAIL flag clear bit */
#define  SDIO_INTCLR_CMDTIMEOUT             ((uint32_t)0x00000004)        /*!< CTIMEOUT flag clear bit */
#define  SDIO_INTCLR_DTTIMEOUT              ((uint32_t)0x00000008)        /*!< DTIMEOUT flag clear bit */
#define  SDIO_INTCLR_TXERRU                 ((uint32_t)0x00000010)        /*!< TXUNDERR flag clear bit */
#define  SDIO_INTCLR_RXERRO                 ((uint32_t)0x00000020)        /*!< RXOVERR flag clear bit */
#define  SDIO_INTCLR_CMDRSPCMPL             ((uint32_t)0x00000040)        /*!< CMDREND flag clear bit */
#define  SDIO_INTCLR_CMDCMPL                ((uint32_t)0x00000080)        /*!< CMDSENT flag clear bit */
#define  SDIO_INTCLR_DTCMPL                 ((uint32_t)0x00000100)        /*!< DATAEND flag clear bit */
#define  SDIO_INTCLR_SBITERR                ((uint32_t)0x00000200)        /*!< STBITERR flag clear bit */
#define  SDIO_INTCLR_DTBLKCMPL              ((uint32_t)0x00000400)        /*!< DBCKEND flag clear bit */
#define  SDIO_INTCLR_SDIOIF                 ((uint32_t)0x00400000)        /*!< SDIOIT flag clear bit */
#define  SDIO_INTCLR_ATACMPL                ((uint32_t)0x00800000)        /*!< CEATAEND flag clear bit */

/******************  Bit definition for SDIO_MASK register  *******************/
#define  SDIO_MASK_CCRCFAILIE               ((uint32_t)0x00000001)        /*!< Command CRC Fail Interrupt Enable */
#define  SDIO_INTEN_DTFAIL                  ((uint32_t)0x00000002)        /*!< Data CRC Fail Interrupt Enable */
#define  SDIO_INTEN_CMDTIMEOUT              ((uint32_t)0x00000004)        /*!< Command TimeOut Interrupt Enable */
#define  SDIO_INTEN_DTTIMEOUT               ((uint32_t)0x00000008)        /*!< Data TimeOut Interrupt Enable */
#define  SDIO_INTEN_TXERRU                  ((uint32_t)0x00000010)        /*!< Tx FIFO UnderRun Error Interrupt Enable */
#define  SDIO_INTEN_RXERRO                  ((uint32_t)0x00000020)        /*!< Rx FIFO OverRun Error Interrupt Enable */
#define  SDIO_INTEN_CMDRSPCMPL              ((uint32_t)0x00000040)        /*!< Command Response Received Interrupt Enable */
#define  SDIO_INTEN_CMDCMPL                 ((uint32_t)0x00000080)        /*!< Command Sent Interrupt Enable */
#define  SDIO_INTEN_DTCMPL                  ((uint32_t)0x00000100)        /*!< Data End Interrupt Enable */
#define  SDIO_INTEN_SBITERR                 ((uint32_t)0x00000200)        /*!< Start Bit Error Interrupt Enable */
#define  SDIO_INTEN_DTBLKCMPL               ((uint32_t)0x00000400)        /*!< Data Block End Interrupt Enable */
#define  SDIO_INTEN_DOCMD                   ((uint32_t)0x00000800)        /*!< Command Acting Interrupt Enable */
#define  SDIO_INTEN_DOTX                    ((uint32_t)0x00001000)        /*!< Data Transmit Acting Interrupt Enable */
#define  SDIO_INTEN_DORX                    ((uint32_t)0x00002000)        /*!< Data receive acting interrupt enabled */
#define  SDIO_INTEN_TXBUF_H                 ((uint32_t)0x00004000)        /*!< Tx FIFO Half Empty interrupt Enable */
#define  SDIO_INTEN_RXBUF_H                 ((uint32_t)0x00008000)        /*!< Rx FIFO Half Full interrupt Enable */
#define  SDIO_INTEN_TXBUF_F                 ((uint32_t)0x00010000)        /*!< Tx FIFO Full interrupt Enable */
#define  SDIO_INTEN_RXBUF_F                 ((uint32_t)0x00020000)        /*!< Rx FIFO Full interrupt Enable */
#define  SDIO_INTEN_TXBUF_E                 ((uint32_t)0x00040000)        /*!< Tx FIFO Empty interrupt Enable */
#define  SDIO_INTEN_RXBUF_E                 ((uint32_t)0x00080000)        /*!< Rx FIFO Empty interrupt Enable */
#define  SDIO_INTEN_TXBUF                   ((uint32_t)0x00100000)        /*!< Data available in Tx FIFO interrupt Enable */
#define  SDIO_INTEN_RXBUF                   ((uint32_t)0x00200000)        /*!< Data available in Rx FIFO interrupt Enable */
#define  SDIO_INTEN_SDIOIF                  ((uint32_t)0x00400000)        /*!< SDIO Mode Interrupt Received interrupt Enable */
#define  SDIO_INTEN_ATACMPL                 ((uint32_t)0x00800000)        /*!< CE-ATA command completion signal received Interrupt Enable */

/*****************  Bit definition for SDIO_BUFCNTR register  *****************/
#define  SDIO_BUFCNTR_CNT                   ((uint32_t)0x00FFFFFF)        /*!< Remaining number of words to be written to or read from the FIFO */

/******************  Bit definition for SDIO_BUF register  *******************/
#define  SDIO_BUF_DT                        ((uint32_t)0xFFFFFFFF)        /*!< Receive and transmit BUF data */

/******************************************************************************/
/*                                                                            */
/*                                   USB Device FS                            */
/*                                                                            */
/******************************************************************************/

/** Endpoint-specific registers */
/*******************  Bit definition for USB_EPT0 register  *******************/
#define  USB_EPT0_EPTADR                    ((uint16_t)0x000F)            /*!< Endpoint Address */

#define  USB_EPT0_STS_TX                    ((uint16_t)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EPT0_STS_TX_0                  ((uint16_t)0x0010)            /*!< Bit 0 */
#define  USB_EPT0_STS_TX_1                  ((uint16_t)0x0020)            /*!< Bit 1 */

#define  USB_EPT0_DTOG_TX                   ((uint16_t)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EPT0_CTFR_TX                   ((uint16_t)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EPT0_EPT_SUBTYPE               ((uint16_t)0x0100)            /*!< Endpoint Kind */

#define  USB_EPT0_EPT_TYPE                  ((uint16_t)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EPT0_EPT_TYPE_0                ((uint16_t)0x0200)            /*!< Bit 0 */
#define  USB_EPT0_EPT_TYPE_1                ((uint16_t)0x0400)            /*!< Bit 1 */

#define  USB_EPT0_SETUP                     ((uint16_t)0x0800)            /*!< Setup transaction completed */

#define  USB_EPT0_STS_RX                    ((uint16_t)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EPT0_STS_RX_0                  ((uint16_t)0x1000)            /*!< Bit 0 */
#define  USB_EPT0_STS_RX_1                  ((uint16_t)0x2000)            /*!< Bit 1 */

#define  USB_EPT0_DTOG_RX                   ((uint16_t)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EPT0_CTFR_RX                   ((uint16_t)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EPT1 register  *******************/
#define  USB_EPT1_EPTADR                    ((uint16_t)0x000F)            /*!< Endpoint Address */

#define  USB_EPT1_STS_TX                    ((uint16_t)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EPT1_STS_TX_0                  ((uint16_t)0x0010)            /*!< Bit 0 */
#define  USB_EPT1_STS_TX_1                  ((uint16_t)0x0020)            /*!< Bit 1 */

#define  USB_EPT1_DTOG_TX                   ((uint16_t)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EPT1_CTFR_TX                   ((uint16_t)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EPT1_EPT_SUBTYPE               ((uint16_t)0x0100)            /*!< Endpoint Kind */

#define  USB_EPT1_EPT_TYPE                  ((uint16_t)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EPT1_EPT_TYPE_0                ((uint16_t)0x0200)            /*!< Bit 0 */
#define  USB_EPT1_EPT_TYPE_1                ((uint16_t)0x0400)            /*!< Bit 1 */

#define  USB_EPT1_SETUP                     ((uint16_t)0x0800)            /*!< Setup transaction completed */

#define  USB_EPT1_STS_RX                    ((uint16_t)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EPT1_STS_RX_0                  ((uint16_t)0x1000)            /*!< Bit 0 */
#define  USB_EPT1_STS_RX_1                  ((uint16_t)0x2000)            /*!< Bit 1 */

#define  USB_EPT1_DTOG_RX                   ((uint16_t)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EPT1_CTFR_RX                   ((uint16_t)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EPT2 register  *******************/
#define  USB_EPT2_EPTADR                    ((uint16_t)0x000F)            /*!< Endpoint Address */

#define  USB_EPT2_STS_TX                    ((uint16_t)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EPT2_STS_TX_0                  ((uint16_t)0x0010)            /*!< Bit 0 */
#define  USB_EPT2_STS_TX_1                  ((uint16_t)0x0020)            /*!< Bit 1 */

#define  USB_EPT2_DTOG_TX                   ((uint16_t)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EPT2_CTFR_TX                   ((uint16_t)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EPT2_EPT_SUBTYPE               ((uint16_t)0x0100)            /*!< Endpoint Kind */

#define  USB_EPT2_EPT_TYPE                  ((uint16_t)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EPT2_EPT_TYPE_0                ((uint16_t)0x0200)            /*!< Bit 0 */
#define  USB_EPT2_EPT_TYPE_1                ((uint16_t)0x0400)            /*!< Bit 1 */

#define  USB_EPT2_SETUP                     ((uint16_t)0x0800)            /*!< Setup transaction completed */

#define  USB_EPT2_STS_RX                    ((uint16_t)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EPT2_STS_RX_0                  ((uint16_t)0x1000)            /*!< Bit 0 */
#define  USB_EPT2_STS_RX_1                  ((uint16_t)0x2000)            /*!< Bit 1 */

#define  USB_EPT2_DTOG_RX                   ((uint16_t)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EPT2_CTFR_RX                   ((uint16_t)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EPT3 register  *******************/
#define  USB_EPT3_EPTADR                    ((uint16_t)0x000F)            /*!< Endpoint Address */

#define  USB_EPT3_STS_TX                    ((uint16_t)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EPT3_STS_TX_0                  ((uint16_t)0x0010)            /*!< Bit 0 */
#define  USB_EPT3_STS_TX_1                  ((uint16_t)0x0020)            /*!< Bit 1 */

#define  USB_EPT3_DTOG_TX                   ((uint16_t)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EPT3_CTFR_TX                   ((uint16_t)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EPT3_EPT_SUBTYPE               ((uint16_t)0x0100)            /*!< Endpoint Kind */

#define  USB_EPT3_EPT_TYPE                  ((uint16_t)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EPT3_EPT_TYPE_0                ((uint16_t)0x0200)            /*!< Bit 0 */
#define  USB_EPT3_EPT_TYPE_1                ((uint16_t)0x0400)            /*!< Bit 1 */

#define  USB_EPT3_SETUP                     ((uint16_t)0x0800)            /*!< Setup transaction completed */

#define  USB_EPT3_STS_RX                    ((uint16_t)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EPT3_STS_RX_0                  ((uint16_t)0x1000)            /*!< Bit 0 */
#define  USB_EPT3_STS_RX_1                  ((uint16_t)0x2000)            /*!< Bit 1 */

#define  USB_EPT3_DTOG_RX                   ((uint16_t)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EPT3_CTFR_RX                   ((uint16_t)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EPT4 register  *******************/
#define  USB_EPT4_EPTADR                    ((uint16_t)0x000F)            /*!< Endpoint Address */

#define  USB_EPT4_STS_TX                    ((uint16_t)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EPT4_STS_TX_0                  ((uint16_t)0x0010)            /*!< Bit 0 */
#define  USB_EPT4_STS_TX_1                  ((uint16_t)0x0020)            /*!< Bit 1 */

#define  USB_EPT4_DTOG_TX                   ((uint16_t)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EPT4_CTFR_TX                   ((uint16_t)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EPT4_EPT_SUBTYPE               ((uint16_t)0x0100)            /*!< Endpoint Kind */

#define  USB_EPT4_EPT_TYPE                  ((uint16_t)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EPT4_EPT_TYPE_0                ((uint16_t)0x0200)            /*!< Bit 0 */
#define  USB_EPT4_EPT_TYPE_1                ((uint16_t)0x0400)            /*!< Bit 1 */

#define  USB_EPT4_SETUP                     ((uint16_t)0x0800)            /*!< Setup transaction completed */

#define  USB_EPT4_STS_RX                    ((uint16_t)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EPT4_STS_RX_0                  ((uint16_t)0x1000)            /*!< Bit 0 */
#define  USB_EPT4_STS_RX_1                  ((uint16_t)0x2000)            /*!< Bit 1 */

#define  USB_EPT4_DTOG_RX                   ((uint16_t)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EPT4_CTFR_RX                   ((uint16_t)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EPT5 register  *******************/
#define  USB_EPT5_EPTADR                    ((uint16_t)0x000F)            /*!< Endpoint Address */

#define  USB_EPT5_STS_TX                    ((uint16_t)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EPT5_STS_TX_0                  ((uint16_t)0x0010)            /*!< Bit 0 */
#define  USB_EPT5_STS_TX_1                  ((uint16_t)0x0020)            /*!< Bit 1 */

#define  USB_EPT5_DTOG_TX                   ((uint16_t)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EPT5_CTFR_TX                   ((uint16_t)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EPT5_EPT_SUBTYPE               ((uint16_t)0x0100)            /*!< Endpoint Kind */

#define  USB_EPT5_EPT_TYPE                  ((uint16_t)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EPT5_EPT_TYPE_0                ((uint16_t)0x0200)            /*!< Bit 0 */
#define  USB_EPT5_EPT_TYPE_1                ((uint16_t)0x0400)            /*!< Bit 1 */

#define  USB_EPT5_SETUP                     ((uint16_t)0x0800)            /*!< Setup transaction completed */

#define  USB_EPT5_STS_RX                    ((uint16_t)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EPT5_STS_RX_0                  ((uint16_t)0x1000)            /*!< Bit 0 */
#define  USB_EPT5_STS_RX_1                  ((uint16_t)0x2000)            /*!< Bit 1 */

#define  USB_EPT5_DTOG_RX                   ((uint16_t)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EPT5_CTFR_RX                   ((uint16_t)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EPT6 register  *******************/
#define  USB_EPT6_EPTADR                    ((uint16_t)0x000F)            /*!< Endpoint Address */

#define  USB_EPT6_STS_TX                    ((uint16_t)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EPT6_STS_TX_0                  ((uint16_t)0x0010)            /*!< Bit 0 */
#define  USB_EPT6_STS_TX_1                  ((uint16_t)0x0020)            /*!< Bit 1 */

#define  USB_EPT6_DTOG_TX                   ((uint16_t)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EPT6_CTFR_TX                   ((uint16_t)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EPT6_EPT_SUBTYPE               ((uint16_t)0x0100)            /*!< Endpoint Kind */

#define  USB_EPT6_EPT_TYPE                  ((uint16_t)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EPT6_EPT_TYPE_0                ((uint16_t)0x0200)            /*!< Bit 0 */
#define  USB_EPT6_EPT_TYPE_1                ((uint16_t)0x0400)            /*!< Bit 1 */

#define  USB_EPT6_SETUP                     ((uint16_t)0x0800)            /*!< Setup transaction completed */

#define  USB_EPT6_STS_RX                    ((uint16_t)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EPT6_STS_RX_0                  ((uint16_t)0x1000)            /*!< Bit 0 */
#define  USB_EPT6_STS_RX_1                  ((uint16_t)0x2000)            /*!< Bit 1 */

#define  USB_EPT6_DTOG_RX                   ((uint16_t)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EPT6_CTFR_RX                   ((uint16_t)0x8000)            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EPT7 register  *******************/
#define  USB_EPT7_EPTADR                    ((uint16_t)0x000F)            /*!< Endpoint Address */

#define  USB_EPT7_STS_TX                    ((uint16_t)0x0030)            /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define  USB_EPT7_STS_TX_0                  ((uint16_t)0x0010)            /*!< Bit 0 */
#define  USB_EPT7_STS_TX_1                  ((uint16_t)0x0020)            /*!< Bit 1 */

#define  USB_EPT7_DTOG_TX                   ((uint16_t)0x0040)            /*!< Data Toggle, for transmission transfers */
#define  USB_EPT7_CTFR_TX                   ((uint16_t)0x0080)            /*!< Correct Transfer for transmission */
#define  USB_EPT7_EPT_SUBTYPE               ((uint16_t)0x0100)            /*!< Endpoint Kind */

#define  USB_EPT7_EPT_TYPE                  ((uint16_t)0x0600)            /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define  USB_EPT7_EPT_TYPE_0                ((uint16_t)0x0200)            /*!< Bit 0 */
#define  USB_EPT7_EPT_TYPE_1                ((uint16_t)0x0400)            /*!< Bit 1 */

#define  USB_EPT7_SETUP                     ((uint16_t)0x0800)            /*!< Setup transaction completed */

#define  USB_EPT7_STS_RX                    ((uint16_t)0x3000)            /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define  USB_EPT7_STS_RX_0                  ((uint16_t)0x1000)            /*!< Bit 0 */
#define  USB_EPT7_STS_RX_1                  ((uint16_t)0x2000)            /*!< Bit 1 */

#define  USB_EPT7_DTOG_RX                   ((uint16_t)0x4000)            /*!< Data Toggle, for reception transfers */
#define  USB_EPT7_CTFR_RX                   ((uint16_t)0x8000)            /*!< Correct Transfer for reception */

/** Common registers */
/*******************  Bit definition for USB_CTRL register  *******************/
#define  USB_CTRL_FRST                      ((uint16_t)0x0001)            /*!< Force USB Reset */
#define  USB_CTRL_PDWN                      ((uint16_t)0x0002)            /*!< Power down */
#define  USB_CTRL_LPWR                      ((uint16_t)0x0004)            /*!< Low-power mode */
#define  USB_CTRL_FSUSP                     ((uint16_t)0x0008)            /*!< Force suspend */
#define  USB_CTRL_RESUME                    ((uint16_t)0x0010)            /*!< Resume request */
#define  USB_CTRL_ESOF_IEN                  ((uint16_t)0x0100)            /*!< Expected Start Of Frame Interrupt Mask */
#define  USB_CTRL_SOF_IEN                   ((uint16_t)0x0200)            /*!< Start Of Frame Interrupt Mask */
#define  USB_CTRL_RST_IEN                   ((uint16_t)0x0400)            /*!< RESET Interrupt Mask */
#define  USB_CTRL_SUSP_IEN                  ((uint16_t)0x0800)            /*!< Suspend mode Interrupt Mask */
#define  USB_CTRL_WKUP_IEN                  ((uint16_t)0x1000)            /*!< Wakeup Interrupt Mask */
#define  USB_CTRL_ERR_IEN                   ((uint16_t)0x2000)            /*!< Error Interrupt Mask */
#define  USB_CTRL_PMOVR_IEN                 ((uint16_t)0x4000)            /*!< Packet Memory Area Over / Underrun Interrupt Mask */
#define  USB_CTRL_CTFR_IEN                  ((uint16_t)0x8000)            /*!< Correct Transfer Interrupt Mask */

/*******************  Bit definition for USB_INTSTS register  *******************/
#define  USB_INTSTS_EPT_ID                  ((uint16_t)0x000F)            /*!< Endpoint Identifier */
#define  USB_INTSTS_DIR                     ((uint16_t)0x0010)            /*!< Direction of transaction */
#define  USB_INTSTS_ESOF                    ((uint16_t)0x0100)            /*!< Expected Start Of Frame */
#define  USB_INTSTS_SOFF                    ((uint16_t)0x0200)            /*!< Start Of Frame */
#define  USB_INTSTS_RSTF                    ((uint16_t)0x0400)            /*!< USB RESET request */
#define  USB_INTSTS_SUSPF                   ((uint16_t)0x0800)            /*!< Suspend mode request */
#define  USB_INTSTS_WKUPF                   ((uint16_t)0x1000)            /*!< Wake up */
#define  USB_INTSTS_ERRF                    ((uint16_t)0x2000)            /*!< Error */
#define  USB_INTSTS_PMOVERF                 ((uint16_t)0x4000)            /*!< Packet Memory Area Over / Underrun */
#define  USB_INTSTS_CTFRF                   ((uint16_t)0x8000)            /*!< Correct Transfer */

/*******************  Bit definition for USB_FRNUM register  ********************/
#define  USB_FRNUM_FRNUM                    ((uint16_t)0x07FF)            /*!< Frame Number */
#define  USB_FRNUM_LSOF                     ((uint16_t)0x1800)            /*!< Lost SOF */
#define  USB_FRNUM_LCK                      ((uint16_t)0x2000)            /*!< Locked */
#define  USB_FRNUM_DMSTS                    ((uint16_t)0x4000)            /*!< Receive Data - Line Status */
#define  USB_FRNUM_DPSTS                    ((uint16_t)0x8000)            /*!< Receive Data + Line Status */

/******************  Bit definition for USB_DEVADR register  *******************/
#define  USB_DEVADR_ADR                     ((uint8_t)0x7F)               /*!< ADD[6:0] bits (Device Address) */
#define  USB_DEVADR_ADR_B0                  ((uint8_t)0x01)               /*!< Bit 0 */
#define  USB_DEVADR_ADR_B1                  ((uint8_t)0x02)               /*!< Bit 1 */
#define  USB_DEVADR_ADR_B2                  ((uint8_t)0x04)               /*!< Bit 2 */
#define  USB_DEVADR_ADR_B3                  ((uint8_t)0x08)               /*!< Bit 3 */
#define  USB_DEVADR_ADR_B4                  ((uint8_t)0x10)               /*!< Bit 4 */
#define  USB_DEVADR_ADR_B5                  ((uint8_t)0x20)               /*!< Bit 5 */
#define  USB_DEVADR_ADR_B6                  ((uint8_t)0x40)               /*!< Bit 6 */

#define  USB_DEVADR_EN                      ((uint8_t)0x80)               /*!< Enable Function */

/******************  Bit definition for USB_BUFTBL register  ******************/
#define  USB_BUFTBL_ADR                     ((uint16_t)0xFFF8)            /*!< Buffer Table */

/** Buffer descriptor table */
/*****************  Bit definition for USB_ADR0_TX register  *****************/
#define  USB_ADR0_TX_ADR0_TX                ((uint16_t)0xFFFE)            /*!< Transmission Buffer Address 0 */

/*****************  Bit definition for USB_ADR1_TX register  *****************/
#define  USB_ADR1_TX_ADR1_TX                ((uint16_t)0xFFFE)            /*!< Transmission Buffer Address 1 */

/*****************  Bit definition for USB_ADR2_TX register  *****************/
#define  USB_ADR2_TX_ADR2_TX                ((uint16_t)0xFFFE)            /*!< Transmission Buffer Address 2 */

/*****************  Bit definition for USB_ADR3_TX register  *****************/
#define  USB_ADR3_TX_ADR3_TX                ((uint16_t)0xFFFE)            /*!< Transmission Buffer Address 3 */

/*****************  Bit definition for USB_ADR4_TX register  *****************/
#define  USB_ADR4_TX_ADR4_TX                ((uint16_t)0xFFFE)            /*!< Transmission Buffer Address 4 */

/*****************  Bit definition for USB_ADR5_TX register  *****************/
#define  USB_ADR5_TX_ADR5_TX                ((uint16_t)0xFFFE)            /*!< Transmission Buffer Address 5 */

/*****************  Bit definition for USB_ADR6_TX register  *****************/
#define  USB_ADR6_TX_ADR6_TX                ((uint16_t)0xFFFE)            /*!< Transmission Buffer Address 6 */

/*****************  Bit definition for USB_ADR7_TX register  *****************/
#define  USB_ADR7_TX_ADR7_TX                ((uint16_t)0xFFFE)            /*!< Transmission Buffer Address 7 */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_CNT0_TX register  ****************/
#define  USB_CNT0_TX_CNT0_TX                ((uint16_t)0x03FF)            /*!< Transmission Byte Count 0 */

/*****************  Bit definition for USB_CNT1_TX register  ****************/
#define  USB_CNT1_TX_CNT1_TX                ((uint16_t)0x03FF)            /*!< Transmission Byte Count 1 */

/*****************  Bit definition for USB_CNT2_TX register  ****************/
#define  USB_CNT2_TX_CNT2_TX                ((uint16_t)0x03FF)            /*!< Transmission Byte Count 2 */

/*****************  Bit definition for USB_CNT3_TX register  ****************/
#define  USB_CNT3_TX_CNT3_TX                ((uint16_t)0x03FF)            /*!< Transmission Byte Count 3 */

/*****************  Bit definition for USB_CNT4_TX register  ****************/
#define  USB_CNT4_TX_CNT4_TX                ((uint16_t)0x03FF)            /*!< Transmission Byte Count 4 */

/*****************  Bit definition for USB_CNT5_TX register  ****************/
#define  USB_CNT5_TX_CNT5_TX                ((uint16_t)0x03FF)            /*!< Transmission Byte Count 5 */

/*****************  Bit definition for USB_CNT6_TX register  ****************/
#define  USB_CNT6_TX_CNT6_TX                ((uint16_t)0x03FF)            /*!< Transmission Byte Count 6 */

/*****************  Bit definition for USB_CNT7_TX register  ****************/
#define  USB_CNT7_TX_CNT7_TX                ((uint16_t)0x03FF)            /*!< Transmission Byte Count 7 */

/*----------------------------------------------------------------------------*/

/****************  Bit definition for USB_CNT0_TX_0 register  ***************/
#define  USB_CNT0_TX_0_CNT0_TX_0            ((uint32_t)0x000003FF)        /*!< Transmission Byte Count 0 (low) */

/****************  Bit definition for USB_CNT0_TX_1 register  ***************/
#define  USB_CNT0_TX_1_CNT0_TX_1            ((uint32_t)0x03FF0000)        /*!< Transmission Byte Count 0 (high) */

/****************  Bit definition for USB_CNT1_TX_0 register  ***************/
#define  USB_CNT1_TX_0_CNT1_TX_0            ((uint32_t)0x000003FF)        /*!< Transmission Byte Count 1 (low) */

/****************  Bit definition for USB_CNT1_TX_1 register  ***************/
#define  USB_CNT1_TX_1_CNT1_TX_1            ((uint32_t)0x03FF0000)        /*!< Transmission Byte Count 1 (high) */

/****************  Bit definition for USB_CNT2_TX_0 register  ***************/
#define  USB_CNT2_TX_0_CNT2_TX_0            ((uint32_t)0x000003FF)        /*!< Transmission Byte Count 2 (low) */

/****************  Bit definition for USB_CNT2_TX_1 register  ***************/
#define  USB_CNT2_TX_1_CNT2_TX_1            ((uint32_t)0x03FF0000)        /*!< Transmission Byte Count 2 (high) */

/****************  Bit definition for USB_CNT3_TX_0 register  ***************/
#define  USB_CNT3_TX_0_CNT3_TX_0            ((uint32_t)0x000003FF)        /*!< Transmission Byte Count 3 (low) */

/****************  Bit definition for USB_CNT3_TX_1 register  ***************/
#define  USB_CNT3_TX_1_CNT3_TX_1            ((uint32_t)0x03FF0000)        /*!< Transmission Byte Count 3 (high) */

/****************  Bit definition for USB_CNT4_TX_0 register  ***************/
#define  USB_CNT4_TX_0_CNT4_TX_0            ((uint32_t)0x000003FF)        /*!< Transmission Byte Count 4 (low) */

/****************  Bit definition for USB_CNT4_TX_1 register  ***************/
#define  USB_CNT4_TX_1_CNT4_TX_1            ((uint32_t)0x03FF0000)        /*!< Transmission Byte Count 4 (high) */

/****************  Bit definition for USB_CNT5_TX_0 register  ***************/
#define  USB_CNT5_TX_0_CNT5_TX_0            ((uint32_t)0x000003FF)        /*!< Transmission Byte Count 5 (low) */

/****************  Bit definition for USB_CNT5_TX_1 register  ***************/
#define  USB_CNT5_TX_1_CNT5_TX_1            ((uint32_t)0x03FF0000)        /*!< Transmission Byte Count 5 (high) */

/****************  Bit definition for USB_CNT6_TX_0 register  ***************/
#define  USB_CNT6_TX_0_CNT6_TX_0            ((uint32_t)0x000003FF)        /*!< Transmission Byte Count 6 (low) */

/****************  Bit definition for USB_CNT6_TX_1 register  ***************/
#define  USB_CNT6_TX_1_CNT6_TX_1            ((uint32_t)0x03FF0000)        /*!< Transmission Byte Count 6 (high) */

/****************  Bit definition for USB_CNT7_TX_0 register  ***************/
#define  USB_CNT7_TX_0_CNT7_TX_0            ((uint32_t)0x000003FF)        /*!< Transmission Byte Count 7 (low) */

/****************  Bit definition for USB_CNT7_TX_1 register  ***************/
#define  USB_CNT7_TX_1_CNT7_TX_1            ((uint32_t)0x03FF0000)        /*!< Transmission Byte Count 7 (high) */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_ADR0_RX register  *****************/
#define  USB_ADR0_RX_ADR0_RX                ((uint16_t)0xFFFE)            /*!< Reception Buffer Address 0 */

/*****************  Bit definition for USB_ADR1_RX register  *****************/
#define  USB_ADR1_RX_ADR1_RX                ((uint16_t)0xFFFE)            /*!< Reception Buffer Address 1 */

/*****************  Bit definition for USB_ADR2_RX register  *****************/
#define  USB_ADR2_RX_ADR2_RX                ((uint16_t)0xFFFE)            /*!< Reception Buffer Address 2 */

/*****************  Bit definition for USB_ADR3_RX register  *****************/
#define  USB_ADR3_RX_ADR3_RX                ((uint16_t)0xFFFE)            /*!< Reception Buffer Address 3 */

/*****************  Bit definition for USB_ADR4_RX register  *****************/
#define  USB_ADR4_RX_ADR4_RX                ((uint16_t)0xFFFE)            /*!< Reception Buffer Address 4 */

/*****************  Bit definition for USB_ADR5_RX register  *****************/
#define  USB_ADR5_RX_ADR5_RX                ((uint16_t)0xFFFE)            /*!< Reception Buffer Address 5 */

/*****************  Bit definition for USB_ADR6_RX register  *****************/
#define  USB_ADR6_RX_ADR6_RX                ((uint16_t)0xFFFE)            /*!< Reception Buffer Address 6 */

/*****************  Bit definition for USB_ADR7_RX register  *****************/
#define  USB_ADR7_RX_ADR7_RX                ((uint16_t)0xFFFE)            /*!< Reception Buffer Address 7 */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_CNT0_RX register  ****************/
#define  USB_CNT0_RX_CNT0_RX                ((uint16_t)0x03FF)            /*!< Reception Byte Count */

#define  USB_CNT0_RX_NUM_BLK                ((uint16_t)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_CNT0_RX_NUM_BLK_0              ((uint16_t)0x0400)            /*!< Bit 0 */
#define  USB_CNT0_RX_NUM_BLK_1              ((uint16_t)0x0800)            /*!< Bit 1 */
#define  USB_CNT0_RX_NUM_BLK_2              ((uint16_t)0x1000)            /*!< Bit 2 */
#define  USB_CNT0_RX_NUM_BLK_3              ((uint16_t)0x2000)            /*!< Bit 3 */
#define  USB_CNT0_RX_NUM_BLK_4              ((uint16_t)0x4000)            /*!< Bit 4 */

#define  USB_CNT0_RX_BLKSIZE                ((uint16_t)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_CNT1_RX register  ****************/
#define  USB_CNT1_RX_CNT1_RX                ((uint16_t)0x03FF)            /*!< Reception Byte Count */

#define  USB_CNT1_RX_NUM_BLK                ((uint16_t)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_CNT1_RX_NUM_BLK_0              ((uint16_t)0x0400)            /*!< Bit 0 */
#define  USB_CNT1_RX_NUM_BLK_1              ((uint16_t)0x0800)            /*!< Bit 1 */
#define  USB_CNT1_RX_NUM_BLK_2              ((uint16_t)0x1000)            /*!< Bit 2 */
#define  USB_CNT1_RX_NUM_BLK_3              ((uint16_t)0x2000)            /*!< Bit 3 */
#define  USB_CNT1_RX_NUM_BLK_4              ((uint16_t)0x4000)            /*!< Bit 4 */

#define  USB_CNT1_RX_BLKSIZE                ((uint16_t)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_CNT2_RX register  ****************/
#define  USB_CNT2_RX_CNT2_RX                ((uint16_t)0x03FF)            /*!< Reception Byte Count */

#define  USB_CNT2_RX_NUM_BLK                ((uint16_t)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_CNT2_RX_NUM_BLK_0              ((uint16_t)0x0400)            /*!< Bit 0 */
#define  USB_CNT2_RX_NUM_BLK_1              ((uint16_t)0x0800)            /*!< Bit 1 */
#define  USB_CNT2_RX_NUM_BLK_2              ((uint16_t)0x1000)            /*!< Bit 2 */
#define  USB_CNT2_RX_NUM_BLK_3              ((uint16_t)0x2000)            /*!< Bit 3 */
#define  USB_CNT2_RX_NUM_BLK_4              ((uint16_t)0x4000)            /*!< Bit 4 */

#define  USB_CNT2_RX_BLKSIZE                ((uint16_t)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_CNT3_RX register  ****************/
#define  USB_CNT3_RX_CNT0_RX                ((uint16_t)0x03FF)            /*!< Reception Byte Count */

#define  USB_CNT3_RX_NUM_BLK                ((uint16_t)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_CNT3_RX_NUM_BLK_0              ((uint16_t)0x0400)            /*!< Bit 0 */
#define  USB_CNT3_RX_NUM_BLK_1              ((uint16_t)0x0800)            /*!< Bit 1 */
#define  USB_CNT3_RX_NUM_BLK_2              ((uint16_t)0x1000)            /*!< Bit 2 */
#define  USB_CNT3_RX_NUM_BLK_3              ((uint16_t)0x2000)            /*!< Bit 3 */
#define  USB_CNT3_RX_NUM_BLK_4              ((uint16_t)0x4000)            /*!< Bit 4 */

#define  USB_CNT3_RX_BLKSIZE                ((uint16_t)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_CNT4_RX register  ****************/
#define  USB_CNT4_RX_CNT0_RX                ((uint16_t)0x03FF)            /*!< Reception Byte Count */

#define  USB_CNT4_RX_NUM_BLK                ((uint16_t)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_CNT4_RX_NUM_BLK_0              ((uint16_t)0x0400)            /*!< Bit 0 */
#define  USB_CNT4_RX_NUM_BLK_1              ((uint16_t)0x0800)            /*!< Bit 1 */
#define  USB_CNT4_RX_NUM_BLK_2              ((uint16_t)0x1000)            /*!< Bit 2 */
#define  USB_CNT4_RX_NUM_BLK_3              ((uint16_t)0x2000)            /*!< Bit 3 */
#define  USB_CNT4_RX_NUM_BLK_4              ((uint16_t)0x4000)            /*!< Bit 4 */

#define  USB_CNT4_RX_BLKSIZE                ((uint16_t)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_CNT5_RX register  ****************/
#define  USB_CNT5_RX_CNT0_RX                ((uint16_t)0x03FF)            /*!< Reception Byte Count */

#define  USB_CNT5_RX_NUM_BLK                ((uint16_t)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_CNT5_RX_NUM_BLK_0              ((uint16_t)0x0400)            /*!< Bit 0 */
#define  USB_CNT5_RX_NUM_BLK_1              ((uint16_t)0x0800)            /*!< Bit 1 */
#define  USB_CNT5_RX_NUM_BLK_2              ((uint16_t)0x1000)            /*!< Bit 2 */
#define  USB_CNT5_RX_NUM_BLK_3              ((uint16_t)0x2000)            /*!< Bit 3 */
#define  USB_CNT5_RX_NUM_BLK_4              ((uint16_t)0x4000)            /*!< Bit 4 */

#define  USB_CNT5_RX_BLKSIZE                ((uint16_t)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_CNT6_RX register  ****************/
#define  USB_CNT6_RX_CNT0_RX                ((uint16_t)0x03FF)            /*!< Reception Byte Count */

#define  USB_CNT6_RX_NUM_BLK                ((uint16_t)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_CNT6_RX_NUM_BLK_0              ((uint16_t)0x0400)            /*!< Bit 0 */
#define  USB_CNT6_RX_NUM_BLK_1              ((uint16_t)0x0800)            /*!< Bit 1 */
#define  USB_CNT6_RX_NUM_BLK_2              ((uint16_t)0x1000)            /*!< Bit 2 */
#define  USB_CNT6_RX_NUM_BLK_3              ((uint16_t)0x2000)            /*!< Bit 3 */
#define  USB_CNT6_RX_NUM_BLK_4              ((uint16_t)0x4000)            /*!< Bit 4 */

#define  USB_CNT6_RX_BLKSIZE                ((uint16_t)0x8000)            /*!< BLock SIZE */

/*****************  Bit definition for USB_CNT7_RX register  ****************/
#define  USB_CNT7_RX_CNT0_RX                ((uint16_t)0x03FF)            /*!< Reception Byte Count */

#define  USB_CNT7_RX_NUM_BLK                ((uint16_t)0x7C00)            /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define  USB_CNT7_RX_NUM_BLK_0              ((uint16_t)0x0400)            /*!< Bit 0 */
#define  USB_CNT7_RX_NUM_BLK_1              ((uint16_t)0x0800)            /*!< Bit 1 */
#define  USB_CNT7_RX_NUM_BLK_2              ((uint16_t)0x1000)            /*!< Bit 2 */
#define  USB_CNT7_RX_NUM_BLK_3              ((uint16_t)0x2000)            /*!< Bit 3 */
#define  USB_CNT7_RX_NUM_BLK_4              ((uint16_t)0x4000)            /*!< Bit 4 */

#define  USB_CNT7_RX_BLKSIZE                ((uint16_t)0x8000)            /*!< BLock SIZE */

/*----------------------------------------------------------------------------*/

/****************  Bit definition for USB_CNT0_RX_0 register  ***************/
#define  USB_CNT0_RX_0_CNT0_RX_0            ((uint32_t)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_CNT0_RX_0_NUM_BLK_0            ((uint32_t)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_CNT0_RX_0_NUM_BLK_0_0          ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  USB_CNT0_RX_0_NUM_BLK_0_1          ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  USB_CNT0_RX_0_NUM_BLK_0_2          ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  USB_CNT0_RX_0_NUM_BLK_0_3          ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  USB_CNT0_RX_0_NUM_BLK_0_4          ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  USB_CNT0_RX_0_BLKSIZE_0            ((uint32_t)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_CNT0_RX_1 register  ***************/
#define  USB_CNT0_RX_1_CNT0_RX_1            ((uint32_t)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_CNT0_RX_1_NUM_BLK_1            ((uint32_t)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_CNT0_RX_1_NUM_BLK_1_0          ((uint32_t)0x04000000)        /*!< Bit 1 */
#define  USB_CNT0_RX_1_NUM_BLK_1_1          ((uint32_t)0x08000000)        /*!< Bit 1 */
#define  USB_CNT0_RX_1_NUM_BLK_1_2          ((uint32_t)0x10000000)        /*!< Bit 2 */
#define  USB_CNT0_RX_1_NUM_BLK_1_3          ((uint32_t)0x20000000)        /*!< Bit 3 */
#define  USB_CNT0_RX_1_NUM_BLK_1_4          ((uint32_t)0x40000000)        /*!< Bit 4 */

#define  USB_CNT0_RX_1_BLKSIZE_1            ((uint32_t)0x80000000)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_CNT1_RX_0 register  ***************/
#define  USB_CNT1_RX_0_CNT1_RX_0            ((uint32_t)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_CNT1_RX_0_NUM_BLK_0            ((uint32_t)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_CNT1_RX_0_NUM_BLK_0_0          ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  USB_CNT1_RX_0_NUM_BLK_0_1          ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  USB_CNT1_RX_0_NUM_BLK_0_2          ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  USB_CNT1_RX_0_NUM_BLK_0_3          ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  USB_CNT1_RX_0_NUM_BLK_0_4          ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  USB_CNT1_RX_0_BLKSIZE_0            ((uint32_t)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_CNT1_RX_1 register  ***************/
#define  USB_CNT1_RX_1_CNT1_RX_1            ((uint32_t)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_CNT1_RX_1_NUM_BLK_1            ((uint32_t)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_CNT1_RX_1_NUM_BLK_1_0          ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  USB_CNT1_RX_1_NUM_BLK_1_1          ((uint32_t)0x08000000)        /*!< Bit 1 */
#define  USB_CNT1_RX_1_NUM_BLK_1_2          ((uint32_t)0x10000000)        /*!< Bit 2 */
#define  USB_CNT1_RX_1_NUM_BLK_1_3          ((uint32_t)0x20000000)        /*!< Bit 3 */
#define  USB_CNT1_RX_1_NUM_BLK_1_4          ((uint32_t)0x40000000)        /*!< Bit 4 */

#define  USB_CNT1_RX_1_BLKSIZE_1            ((uint32_t)0x80000000)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_CNT2_RX_0 register  ***************/
#define  USB_CNT2_RX_0_CNT2_RX_0            ((uint32_t)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_CNT2_RX_0_NUM_BLK_0            ((uint32_t)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_CNT2_RX_0_NUM_BLK_0_0          ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  USB_CNT2_RX_0_NUM_BLK_0_1          ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  USB_CNT2_RX_0_NUM_BLK_0_2          ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  USB_CNT2_RX_0_NUM_BLK_0_3          ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  USB_CNT2_RX_0_NUM_BLK_0_4          ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  USB_CNT2_RX_0_BLKSIZE_0            ((uint32_t)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_CNT2_RX_1 register  ***************/
#define  USB_CNT2_RX_1_CNT2_RX_1            ((uint32_t)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_CNT2_RX_1_NUM_BLK_1            ((uint32_t)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_CNT2_RX_1_NUM_BLK_1_0          ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  USB_CNT2_RX_1_NUM_BLK_1_1          ((uint32_t)0x08000000)        /*!< Bit 1 */
#define  USB_CNT2_RX_1_NUM_BLK_1_2          ((uint32_t)0x10000000)        /*!< Bit 2 */
#define  USB_CNT2_RX_1_NUM_BLK_1_3          ((uint32_t)0x20000000)        /*!< Bit 3 */
#define  USB_CNT2_RX_1_NUM_BLK_1_4          ((uint32_t)0x40000000)        /*!< Bit 4 */

#define  USB_CNT2_RX_1_BLKSIZE_1            ((uint32_t)0x80000000)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_CNT3_RX_0 register  ***************/
#define  USB_CNT3_RX_0_CNT3_RX_0            ((uint32_t)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_CNT3_RX_0_NUM_BLK_0            ((uint32_t)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_CNT3_RX_0_NUM_BLK_0_0          ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  USB_CNT3_RX_0_NUM_BLK_0_1          ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  USB_CNT3_RX_0_NUM_BLK_0_2          ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  USB_CNT3_RX_0_NUM_BLK_0_3          ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  USB_CNT3_RX_0_NUM_BLK_0_4          ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  USB_CNT3_RX_0_BLKSIZE_0            ((uint32_t)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_CNT3_RX_1 register  ***************/
#define  USB_CNT3_RX_1_CNT3_RX_1            ((uint32_t)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_CNT3_RX_1_NUM_BLK_1            ((uint32_t)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_CNT3_RX_1_NUM_BLK_1_0          ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  USB_CNT3_RX_1_NUM_BLK_1_1          ((uint32_t)0x08000000)        /*!< Bit 1 */
#define  USB_CNT3_RX_1_NUM_BLK_1_2          ((uint32_t)0x10000000)        /*!< Bit 2 */
#define  USB_CNT3_RX_1_NUM_BLK_1_3          ((uint32_t)0x20000000)        /*!< Bit 3 */
#define  USB_CNT3_RX_1_NUM_BLK_1_4          ((uint32_t)0x40000000)        /*!< Bit 4 */

#define  USB_CNT3_RX_1_BLKSIZE_1            ((uint32_t)0x80000000)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_CNT4_RX_0 register  ***************/
#define  USB_CNT4_RX_0_CNT4_RX_0            ((uint32_t)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_CNT4_RX_0_NUM_BLK_0            ((uint32_t)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_CNT4_RX_0_NUM_BLK_0_0          ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  USB_CNT4_RX_0_NUM_BLK_0_1          ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  USB_CNT4_RX_0_NUM_BLK_0_2          ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  USB_CNT4_RX_0_NUM_BLK_0_3          ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  USB_CNT4_RX_0_NUM_BLK_0_4          ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  USB_CNT4_RX_0_BLKSIZE_0            ((uint32_t)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_CNT4_RX_1 register  ***************/
#define  USB_CNT4_RX_1_CNT4_RX_1            ((uint32_t)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_CNT4_RX_1_NUM_BLK_1            ((uint32_t)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_CNT4_RX_1_NUM_BLK_1_0          ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  USB_CNT4_RX_1_NUM_BLK_1_1          ((uint32_t)0x08000000)        /*!< Bit 1 */
#define  USB_CNT4_RX_1_NUM_BLK_1_2          ((uint32_t)0x10000000)        /*!< Bit 2 */
#define  USB_CNT4_RX_1_NUM_BLK_1_3          ((uint32_t)0x20000000)        /*!< Bit 3 */
#define  USB_CNT4_RX_1_NUM_BLK_1_4          ((uint32_t)0x40000000)        /*!< Bit 4 */

#define  USB_CNT4_RX_1_BLKSIZE_1            ((uint32_t)0x80000000)        /*!< BLock SIZE (high) */

/****************  Bit definition for USB_CNT5_RX_0 register  ***************/
#define  USB_CNT5_RX_0_CNT5_RX_0            ((uint32_t)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_CNT5_RX_0_NUM_BLK_0            ((uint32_t)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_CNT5_RX_0_NUM_BLK_0_0          ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  USB_CNT5_RX_0_NUM_BLK_0_1          ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  USB_CNT5_RX_0_NUM_BLK_0_2          ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  USB_CNT5_RX_0_NUM_BLK_0_3          ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  USB_CNT5_RX_0_NUM_BLK_0_4          ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  USB_CNT5_RX_0_BLKSIZE_0            ((uint32_t)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_CNT5_RX_1 register  ***************/
#define  USB_CNT5_RX_1_CNT5_RX_1            ((uint32_t)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_CNT5_RX_1_NUM_BLK_1            ((uint32_t)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_CNT5_RX_1_NUM_BLK_1_0          ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  USB_CNT5_RX_1_NUM_BLK_1_1          ((uint32_t)0x08000000)        /*!< Bit 1 */
#define  USB_CNT5_RX_1_NUM_BLK_1_2          ((uint32_t)0x10000000)        /*!< Bit 2 */
#define  USB_CNT5_RX_1_NUM_BLK_1_3          ((uint32_t)0x20000000)        /*!< Bit 3 */
#define  USB_CNT5_RX_1_NUM_BLK_1_4          ((uint32_t)0x40000000)        /*!< Bit 4 */

#define  USB_CNT5_RX_1_BLKSIZE_1            ((uint32_t)0x80000000)        /*!< BLock SIZE (high) */

/***************  Bit definition for USB_CNT6_RX_0  register  ***************/
#define  USB_CNT6_RX_0_CNT6_RX_0            ((uint32_t)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_CNT6_RX_0_NUM_BLK_0            ((uint32_t)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_CNT6_RX_0_NUM_BLK_0_0          ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  USB_CNT6_RX_0_NUM_BLK_0_1          ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  USB_CNT6_RX_0_NUM_BLK_0_2          ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  USB_CNT6_RX_0_NUM_BLK_0_3          ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  USB_CNT6_RX_0_NUM_BLK_0_4          ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  USB_CNT6_RX_0_BLKSIZE_0            ((uint32_t)0x00008000)        /*!< BLock SIZE (low) */

/****************  Bit definition for USB_CNT6_RX_1 register  ***************/
#define  USB_CNT6_RX_1_CNT6_RX_1            ((uint32_t)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_CNT6_RX_1_NUM_BLK_1            ((uint32_t)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_CNT6_RX_1_NUM_BLK_1_0          ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  USB_CNT6_RX_1_NUM_BLK_1_1          ((uint32_t)0x08000000)        /*!< Bit 1 */
#define  USB_CNT6_RX_1_NUM_BLK_1_2          ((uint32_t)0x10000000)        /*!< Bit 2 */
#define  USB_CNT6_RX_1_NUM_BLK_1_3          ((uint32_t)0x20000000)        /*!< Bit 3 */
#define  USB_CNT6_RX_1_NUM_BLK_1_4          ((uint32_t)0x40000000)        /*!< Bit 4 */

#define  USB_CNT6_RX_1_BLKSIZE_1            ((uint32_t)0x80000000)        /*!< BLock SIZE (high) */

/***************  Bit definition for USB_CNT7_RX_0 register  ****************/
#define  USB_CNT7_RX_0_CNT7_RX_0            ((uint32_t)0x000003FF)        /*!< Reception Byte Count (low) */

#define  USB_CNT7_RX_0_NUM_BLK_0            ((uint32_t)0x00007C00)        /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define  USB_CNT7_RX_0_NUM_BLK_0_0          ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  USB_CNT7_RX_0_NUM_BLK_0_1          ((uint32_t)0x00000800)        /*!< Bit 1 */
#define  USB_CNT7_RX_0_NUM_BLK_0_2          ((uint32_t)0x00001000)        /*!< Bit 2 */
#define  USB_CNT7_RX_0_NUM_BLK_0_3          ((uint32_t)0x00002000)        /*!< Bit 3 */
#define  USB_CNT7_RX_0_NUM_BLK_0_4          ((uint32_t)0x00004000)        /*!< Bit 4 */

#define  USB_CNT7_RX_0_BLKSIZE_0            ((uint32_t)0x00008000)        /*!< BLock SIZE (low) */

/***************  Bit definition for USB_CNT7_RX_1 register  ****************/
#define  USB_CNT7_RX_1_CNT7_RX_1            ((uint32_t)0x03FF0000)        /*!< Reception Byte Count (high) */

#define  USB_CNT7_RX_1_NUM_BLK_1            ((uint32_t)0x7C000000)        /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define  USB_CNT7_RX_1_NUM_BLK_1_0          ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  USB_CNT7_RX_1_NUM_BLK_1_1          ((uint32_t)0x08000000)        /*!< Bit 1 */
#define  USB_CNT7_RX_1_NUM_BLK_1_2          ((uint32_t)0x10000000)        /*!< Bit 2 */
#define  USB_CNT7_RX_1_NUM_BLK_1_3          ((uint32_t)0x20000000)        /*!< Bit 3 */
#define  USB_CNT7_RX_1_NUM_BLK_1_4          ((uint32_t)0x40000000)        /*!< Bit 4 */

#define  USB_CNT7_RX_1_BLKSIZE_1            ((uint32_t)0x80000000)        /*!< BLock SIZE (high) */

/******************************************************************************/
/*                                                                            */
/*                         Controller Area Network                            */
/*                                                                            */
/******************************************************************************/

/** CAN control and status registers */
/*******************  Bit definition for CAN_MCTRL register  ********************/
#define  CAN_MCTRL_INRQ                     ((uint16_t)0x0001)            /*!< Initialization Request */
#define  CAN_MCTRL_SLP                      ((uint16_t)0x0002)            /*!< Sleep Mode Request */
#define  CAN_MCTRL_TFP                      ((uint16_t)0x0004)            /*!< Transmit FIFO Priority */
#define  CAN_MCTRL_RFL                      ((uint16_t)0x0008)            /*!< Receive FIFO Locked Mode */
#define  CAN_MCTRL_NART                     ((uint16_t)0x0010)            /*!< No Automatic Retransmission */
#define  CAN_MCTRL_AWU                      ((uint16_t)0x0020)            /*!< Automatic Wakeup Mode */
#define  CAN_MCTRL_ABO                      ((uint16_t)0x0040)            /*!< Automatic Bus-Off Management */
#define  CAN_MCTRL_TTC                      ((uint16_t)0x0080)            /*!< Time Triggered Communication Mode */
#define  CAN_MCTRL_RST                      ((uint16_t)0x8000)            /*!< CAN software master reset */

/*******************  Bit definition for CAN_MSTS register  ********************/
#define  CAN_MSTS_IAK                       ((uint16_t)0x0001)            /*!< Initialization Acknowledge */
#define  CAN_MSTS_SAK                       ((uint16_t)0x0002)            /*!< Sleep Acknowledge */
#define  CAN_MSTS_ERIT                      ((uint16_t)0x0004)            /*!< Error Interrupt */
#define  CAN_MSTS_WKIT                      ((uint16_t)0x0008)            /*!< Wakeup Interrupt */
#define  CAN_MSTS_SAKIT                     ((uint16_t)0x0010)            /*!< Sleep Acknowledge Interrupt */
#define  CAN_MSTS_TX                        ((uint16_t)0x0100)            /*!< Transmit Mode */
#define  CAN_MSTS_RX                        ((uint16_t)0x0200)            /*!< Receive Mode */
#define  CAN_MSTS_LSAP                      ((uint16_t)0x0400)            /*!< Last Sample Point */
#define  CAN_MSTS_RXS                       ((uint16_t)0x0800)            /*!< CAN Rx Signal */

/*******************  Bit definition for CAN_TSTS register  ********************/
#define  CAN_TSTS_RQC0                      ((uint32_t)0x00000001)        /*!< Request Completed Mailbox0 */
#define  CAN_TSTS_TOK0                      ((uint32_t)0x00000002)        /*!< Transmission OK of Mailbox0 */
#define  CAN_TSTS_ALS0                      ((uint32_t)0x00000004)        /*!< Arbitration Lost for Mailbox0 */
#define  CAN_TSTS_TER0                      ((uint32_t)0x00000008)        /*!< Transmission Error of Mailbox0 */
#define  CAN_TSTS_ARQ0                      ((uint32_t)0x00000080)        /*!< Abort Request for Mailbox0 */
#define  CAN_TSTS_RQC1                      ((uint32_t)0x00000100)        /*!< Request Completed Mailbox1 */
#define  CAN_TSTS_TOK1                      ((uint32_t)0x00000200)        /*!< Transmission OK of Mailbox1 */
#define  CAN_TSTS_ALS1                      ((uint32_t)0x00000400)        /*!< Arbitration Lost for Mailbox1 */
#define  CAN_TSTS_TER1                      ((uint32_t)0x00000800)        /*!< Transmission Error of Mailbox1 */
#define  CAN_TSTS_ARQ1                      ((uint32_t)0x00008000)        /*!< Abort Request for Mailbox 1 */
#define  CAN_TSTS_RQC2                      ((uint32_t)0x00010000)        /*!< Request Completed Mailbox2 */
#define  CAN_TSTS_TOK2                      ((uint32_t)0x00020000)        /*!< Transmission OK of Mailbox 2 */
#define  CAN_TSTS_ALS2                      ((uint32_t)0x00040000)        /*!< Arbitration Lost for mailbox 2 */
#define  CAN_TSTS_TER2                      ((uint32_t)0x00080000)        /*!< Transmission Error of Mailbox 2 */
#define  CAN_TSTS_ARQ2                      ((uint32_t)0x00800000)        /*!< Abort Request for Mailbox 2 */
#define  CAN_TSTS_NTM                       ((uint32_t)0x03000000)        /*!< Mailbox Code */

#define  CAN_TSTS_TSME                      ((uint32_t)0x1C000000)        /*!< TME[2:0] bits */
#define  CAN_TSTS_TSME0                     ((uint32_t)0x04000000)        /*!< Transmit Mailbox 0 Empty */
#define  CAN_TSTS_TSME1                     ((uint32_t)0x08000000)        /*!< Transmit Mailbox 1 Empty */
#define  CAN_TSTS_TSME2                     ((uint32_t)0x10000000)        /*!< Transmit Mailbox 2 Empty */

#define  CAN_TSTS_LPM                       ((uint32_t)0xE0000000)        /*!< LOW[2:0] bits */
#define  CAN_TSTS_LPM0                      ((uint32_t)0x20000000)        /*!< Lowest Priority Flag for Mailbox 0 */
#define  CAN_TSTS_LPM1                      ((uint32_t)0x40000000)        /*!< Lowest Priority Flag for Mailbox 1 */
#define  CAN_TSTS_LPM2                      ((uint32_t)0x80000000)        /*!< Lowest Priority Flag for Mailbox 2 */

/*******************  Bit definition for CAN_RF0 register  *******************/
#define  CAN_RF0_RFP0                       ((uint8_t)0x03)               /*!< FIFO 0 Message Pending */
#define  CAN_RF0_RFFU0                      ((uint8_t)0x08)               /*!< FIFO 0 Full */
#define  CAN_RF0_RFOV0                      ((uint8_t)0x10)               /*!< FIFO 0 Overrun */
#define  CAN_RF0_RRFM0                      ((uint8_t)0x20)               /*!< Release FIFO 0 Output Mailbox */

/*******************  Bit definition for CAN_RF1 register  *******************/
#define  CAN_RF1_RFP1                       ((uint8_t)0x03)               /*!< FIFO 1 Message Pending */
#define  CAN_RF1_RFFU1                      ((uint8_t)0x08)               /*!< FIFO 1 Full */
#define  CAN_RF1_RFOV1                      ((uint8_t)0x10)               /*!< FIFO 1 Overrun */
#define  CAN_RF1_RRFM1                      ((uint8_t)0x20)               /*!< Release FIFO 1 Output Mailbox */

/********************  Bit definition for CAN_INTEN register  *******************/
#define  CAN_INTEN_TSMEIE                   ((uint32_t)0x00000001)        /*!< Transmit Mailbox Empty Interrupt Enable */
#define  CAN_INTEN_RFPIE0                   ((uint32_t)0x00000002)        /*!< FIFO Message Pending Interrupt Enable */
#define  CAN_INTEN_RFFUIE0                  ((uint32_t)0x00000004)        /*!< FIFO Full Interrupt Enable */
#define  CAN_INTEN_RFOVIE0                  ((uint32_t)0x00000008)        /*!< FIFO Overrun Interrupt Enable */
#define  CAN_INTEN_RFPIE1                   ((uint32_t)0x00000010)        /*!< FIFO Message Pending Interrupt Enable */
#define  CAN_INTEN_RFFUIE1                  ((uint32_t)0x00000020)        /*!< FIFO Full Interrupt Enable */
#define  CAN_INTEN_RFOVIE1                  ((uint32_t)0x00000040)        /*!< FIFO Overrun Interrupt Enable */
#define  CAN_INTEN_ERWIE                    ((uint32_t)0x00000100)        /*!< Error Warning Interrupt Enable */
#define  CAN_INTEN_ERPIE                    ((uint32_t)0x00000200)        /*!< Error Passive Interrupt Enable */
#define  CAN_INTEN_BUIE                     ((uint32_t)0x00000400)        /*!< Bus-Off Interrupt Enable */
#define  CAN_INTEN_ERCIE                    ((uint32_t)0x00000800)        /*!< Last Error Code Interrupt Enable */
#define  CAN_INTEN_ERIE                     ((uint32_t)0x00008000)        /*!< Error Interrupt Enable */
#define  CAN_INTEN_WKIE                     ((uint32_t)0x00010000)        /*!< Wakeup Interrupt Enable */
#define  CAN_INTEN_SAKIE                    ((uint32_t)0x00020000)        /*!< Sleep Interrupt Enable */

/********************  Bit definition for CAN_ESTS register  *******************/
#define  CAN_ESTS_ERWF                      ((uint32_t)0x00000001)        /*!< Error Warning Flag */
#define  CAN_ESTS_ERPF                      ((uint32_t)0x00000002)        /*!< Error Passive Flag */
#define  CAN_ESTS_BFF                       ((uint32_t)0x00000004)        /*!< Bus-Off Flag */

#define  CAN_ESTS_ERC                       ((uint32_t)0x00000070)        /*!< LEC[2:0] bits (Last Error Code) */
#define  CAN_ESTS_ERC_0                     ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  CAN_ESTS_ERC_1                     ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  CAN_ESTS_ERC_2                     ((uint32_t)0x00000040)        /*!< Bit 2 */

#define  CAN_ESTS_TEC                       ((uint32_t)0x00FF0000)        /*!< Least significant byte of the 9-bit Transmit Error Counter */
#define  CAN_ESTS_REC                       ((uint32_t)0xFF000000)        /*!< Receive Error Counter */

/*******************  Bit definition for CAN_BTMG register  ********************/
#define  CAN_BTMG_BRP                       ((uint32_t)0x000003FF)        /*!< Baud Rate Prescaler */
#define  CAN_BTMG_BS1                       ((uint32_t)0x000F0000)        /*!< Time Segment 1 */
#define  CAN_BTMG_BS2                       ((uint32_t)0x00700000)        /*!< Time Segment 2 */
#define  CAN_BTMG_SJW                       ((uint32_t)0x03000000)        /*!< Resynchronization Jump Width */
#define  CAN_BTMG_LBK                       ((uint32_t)0x40000000)        /*!< Loop Back Mode (Debug) */
#define  CAN_BTMG_SIL                       ((uint32_t)0x80000000)        /*!< Silent Mode */

/** Mailbox registers */
/******************  Bit definition for CAN_TI0R register  ********************/
#define  CAN_TMI0_TRQ                       ((uint32_t)0x00000001)        /*!< Transmit Mailbox Request */
#define  CAN_TMI0_RTR                       ((uint32_t)0x00000002)        /*!< Remote Transmission Request */
#define  CAN_TMI0_IDT                       ((uint32_t)0x00000004)        /*!< Identifier Extension */
#define  CAN_TMI0_EID                       ((uint32_t)0x001FFFF8)        /*!< Extended Identifier */
#define  CAN_TMI0_SID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */

/******************  Bit definition for CAN_TDT0 register  *******************/
#define  CAN_TDT0_DLC                       ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  CAN_TDT0_TMEN                      ((uint32_t)0x00000100)        /*!< Transmit Global Time */
#define  CAN_TDT0_TS                        ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */

/******************  Bit definition for CAN_TDL0 register  *******************/
#define  CAN_TDL0_D0                        ((uint32_t)0x000000FF)        /*!< Data byte 0 */
#define  CAN_TDL0_D1                        ((uint32_t)0x0000FF00)        /*!< Data byte 1 */
#define  CAN_TDL0_D2                        ((uint32_t)0x00FF0000)        /*!< Data byte 2 */
#define  CAN_TDL0_D3                        ((uint32_t)0xFF000000)        /*!< Data byte 3 */

/******************  Bit definition for CAN_TDH0 register  *******************/
#define  CAN_TDH0_D4                        ((uint32_t)0x000000FF)        /*!< Data byte 4 */
#define  CAN_TDH0_D5                        ((uint32_t)0x0000FF00)        /*!< Data byte 5 */
#define  CAN_TDH0_D6                        ((uint32_t)0x00FF0000)        /*!< Data byte 6 */
#define  CAN_TDH0_D7                        ((uint32_t)0xFF000000)        /*!< Data byte 7 */

/*******************  Bit definition for CAN_TMI1 register  *******************/
#define  CAN_TMI1_TRQ                       ((uint32_t)0x00000001)        /*!< Transmit Mailbox Request */
#define  CAN_TMI1_RTR                       ((uint32_t)0x00000002)        /*!< Remote Transmission Request */
#define  CAN_TMI1_IDT                       ((uint32_t)0x00000004)        /*!< Identifier Extension */
#define  CAN_TMI1_EID                       ((uint32_t)0x001FFFF8)        /*!< Extended Identifier */
#define  CAN_TMI1_SID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT1 register  ******************/
#define  CAN_TDT1_DLC                       ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  CAN_TDT1_TMEN                      ((uint32_t)0x00000100)        /*!< Transmit Global Time */
#define  CAN_TDT1_TS                        ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */

/*******************  Bit definition for CAN_TDL1 register  ******************/
#define  CAN_TDL1_D0                        ((uint32_t)0x000000FF)        /*!< Data byte 0 */
#define  CAN_TDL1_D1                        ((uint32_t)0x0000FF00)        /*!< Data byte 1 */
#define  CAN_TDL1_D2                        ((uint32_t)0x00FF0000)        /*!< Data byte 2 */
#define  CAN_TDL1_D3                        ((uint32_t)0xFF000000)        /*!< Data byte 3 */

/*******************  Bit definition for CAN_TDH1 register  ******************/
#define  CAN_TDH1_D4                        ((uint32_t)0x000000FF)        /*!< Data byte 4 */
#define  CAN_TDH1_D5                        ((uint32_t)0x0000FF00)        /*!< Data byte 5 */
#define  CAN_TDH1_D6                        ((uint32_t)0x00FF0000)        /*!< Data byte 6 */
#define  CAN_TDH1_D7                        ((uint32_t)0xFF000000)        /*!< Data byte 7 */

/*******************  Bit definition for CAN_TMI2 register  *******************/
#define  CAN_TMI2_TRQ                       ((uint32_t)0x00000001)        /*!< Transmit Mailbox Request */
#define  CAN_TMI2_RTR                       ((uint32_t)0x00000002)        /*!< Remote Transmission Request */
#define  CAN_TMI2_IDT                       ((uint32_t)0x00000004)        /*!< Identifier Extension */
#define  CAN_TMI2_EID                       ((uint32_t)0x001FFFF8)        /*!< Extended identifier */
#define  CAN_TMI2_SID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_TDT2 register  ******************/
#define  CAN_TDT2_DLC                       ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  CAN_TDT2_TMEN                      ((uint32_t)0x00000100)        /*!< Transmit Global Time */
#define  CAN_TDT2_TS                        ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */

/*******************  Bit definition for CAN_TDL2 register  ******************/
#define  CAN_TDL2_D0                        ((uint32_t)0x000000FF)        /*!< Data byte 0 */
#define  CAN_TDL2_D1                        ((uint32_t)0x0000FF00)        /*!< Data byte 1 */
#define  CAN_TDL2_D2                        ((uint32_t)0x00FF0000)        /*!< Data byte 2 */
#define  CAN_TDL2_D3                        ((uint32_t)0xFF000000)        /*!< Data byte 3 */

/*******************  Bit definition for CAN_TDH2 register  ******************/
#define  CAN_TDH2_D4                        ((uint32_t)0x000000FF)        /*!< Data byte 4 */
#define  CAN_TDH2_D5                        ((uint32_t)0x0000FF00)        /*!< Data byte 5 */
#define  CAN_TDH2_D6                        ((uint32_t)0x00FF0000)        /*!< Data byte 6 */
#define  CAN_TDH2_D7                        ((uint32_t)0xFF000000)        /*!< Data byte 7 */

/*******************  Bit definition for CAN_RFI0 register  *******************/
#define  CAN_RFI0_RTR                       ((uint32_t)0x00000002)        /*!< Remote Transmission Request */
#define  CAN_RFI0_IDT                       ((uint32_t)0x00000004)        /*!< Identifier Extension */
#define  CAN_RFI0_EID                       ((uint32_t)0x001FFFF8)        /*!< Extended Identifier */
#define  CAN_RFI0_SID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT0 register  ******************/
#define  CAN_RDT0_DLC                       ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  CAN_RDT0_FID                       ((uint32_t)0x0000FF00)        /*!< Filter Match Index */
#define  CAN_RDT0_TS                        ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */

/*******************  Bit definition for CAN_RDL0 register  ******************/
#define  CAN_RDL0_D0                        ((uint32_t)0x000000FF)        /*!< Data byte 0 */
#define  CAN_RDL0_D1                        ((uint32_t)0x0000FF00)        /*!< Data byte 1 */
#define  CAN_RDL0_D2                        ((uint32_t)0x00FF0000)        /*!< Data byte 2 */
#define  CAN_RDL0_D3                        ((uint32_t)0xFF000000)        /*!< Data byte 3 */

/*******************  Bit definition for CAN_RDH0 register  ******************/
#define  CAN_RDH0_D4                        ((uint32_t)0x000000FF)        /*!< Data byte 4 */
#define  CAN_RDH0_D5                        ((uint32_t)0x0000FF00)        /*!< Data byte 5 */
#define  CAN_RDH0_D6                        ((uint32_t)0x00FF0000)        /*!< Data byte 6 */
#define  CAN_RDH0_D7                        ((uint32_t)0xFF000000)        /*!< Data byte 7 */

/*******************  Bit definition for CAN_RFI1 register  *******************/
#define  CAN_RFI1_RTR                       ((uint32_t)0x00000002)        /*!< Remote Transmission Request */
#define  CAN_RFI1_IDT                       ((uint32_t)0x00000004)        /*!< Identifier Extension */
#define  CAN_RFI1_EID                       ((uint32_t)0x001FFFF8)        /*!< Extended identifier */
#define  CAN_RFI1_SID                       ((uint32_t)0xFFE00000)        /*!< Standard Identifier or Extended Identifier */

/*******************  Bit definition for CAN_RDT1 register  ******************/
#define  CAN_RDT1_DLC                       ((uint32_t)0x0000000F)        /*!< Data Length Code */
#define  CAN_RDT1_FID                       ((uint32_t)0x0000FF00)        /*!< Filter Match Index */
#define  CAN_RDT1_TS                        ((uint32_t)0xFFFF0000)        /*!< Message Time Stamp */

/*******************  Bit definition for CAN_RDL1 register  ******************/
#define  CAN_RDL1_D0                        ((uint32_t)0x000000FF)        /*!< Data byte 0 */
#define  CAN_RDL1_D1                        ((uint32_t)0x0000FF00)        /*!< Data byte 1 */
#define  CAN_RDL1_D2                        ((uint32_t)0x00FF0000)        /*!< Data byte 2 */
#define  CAN_RDL1_D3                        ((uint32_t)0xFF000000)        /*!< Data byte 3 */

/*******************  Bit definition for CAN_RDH1 register  ******************/
#define  CAN_RDH1_D4                        ((uint32_t)0x000000FF)        /*!< Data byte 4 */
#define  CAN_RDH1_D5                        ((uint32_t)0x0000FF00)        /*!< Data byte 5 */
#define  CAN_RDH1_D6                        ((uint32_t)0x00FF0000)        /*!< Data byte 6 */
#define  CAN_RDH1_D7                        ((uint32_t)0xFF000000)        /*!< Data byte 7 */

/** CAN filter registers */
/*******************  Bit definition for CAN_FM register  ********************/
#define  CAN_FM_FINT                        ((uint8_t)0x01)               /*!< Filter Init Mode */

/*******************  Bit definition for CAN_FM1 register  *******************/
#define  CAN_FM1_FMS                        ((uint16_t)0x3FFF)            /*!< Filter Mode */
#define  CAN_FM1_FMS0                       ((uint16_t)0x0001)            /*!< Filter Init Mode bit 0 */
#define  CAN_FM1_FMS1                       ((uint16_t)0x0002)            /*!< Filter Init Mode bit 1 */
#define  CAN_FM1_FMS2                       ((uint16_t)0x0004)            /*!< Filter Init Mode bit 2 */
#define  CAN_FM1_FMS3                       ((uint16_t)0x0008)            /*!< Filter Init Mode bit 3 */
#define  CAN_FM1_FMS4                       ((uint16_t)0x0010)            /*!< Filter Init Mode bit 4 */
#define  CAN_FM1_FMS5                       ((uint16_t)0x0020)            /*!< Filter Init Mode bit 5 */
#define  CAN_FM1_FMS6                       ((uint16_t)0x0040)            /*!< Filter Init Mode bit 6 */
#define  CAN_FM1_FMS7                       ((uint16_t)0x0080)            /*!< Filter Init Mode bit 7 */
#define  CAN_FM1_FMS8                       ((uint16_t)0x0100)            /*!< Filter Init Mode bit 8 */
#define  CAN_FM1_FMS9                       ((uint16_t)0x0200)            /*!< Filter Init Mode bit 9 */
#define  CAN_FM1_FMS10                      ((uint16_t)0x0400)            /*!< Filter Init Mode bit 10 */
#define  CAN_FM1_FMS11                      ((uint16_t)0x0800)            /*!< Filter Init Mode bit 11 */
#define  CAN_FM1_FMS12                      ((uint16_t)0x1000)            /*!< Filter Init Mode bit 12 */
#define  CAN_FM1_FMS13                      ((uint16_t)0x2000)            /*!< Filter Init Mode bit 13 */

/*******************  Bit definition for CAN_FS1 register  *******************/
#define  CAN_FS1_FBS                        ((uint16_t)0x3FFF)            /*!< Filter Scale Configuration */
#define  CAN_FS1_FBS0                       ((uint16_t)0x0001)            /*!< Filter Scale Configuration bit 0 */
#define  CAN_FS1_FBS1                       ((uint16_t)0x0002)            /*!< Filter Scale Configuration bit 1 */
#define  CAN_FS1_FBS2                       ((uint16_t)0x0004)            /*!< Filter Scale Configuration bit 2 */
#define  CAN_FS1_FBS3                       ((uint16_t)0x0008)            /*!< Filter Scale Configuration bit 3 */
#define  CAN_FS1_FBS4                       ((uint16_t)0x0010)            /*!< Filter Scale Configuration bit 4 */
#define  CAN_FS1_FBS5                       ((uint16_t)0x0020)            /*!< Filter Scale Configuration bit 5 */
#define  CAN_FS1_FBS6                       ((uint16_t)0x0040)            /*!< Filter Scale Configuration bit 6 */
#define  CAN_FS1_FBS7                       ((uint16_t)0x0080)            /*!< Filter Scale Configuration bit 7 */
#define  CAN_FS1_FBS8                       ((uint16_t)0x0100)            /*!< Filter Scale Configuration bit 8 */
#define  CAN_FS1_FBS9                       ((uint16_t)0x0200)            /*!< Filter Scale Configuration bit 9 */
#define  CAN_FS1_FBS10                      ((uint16_t)0x0400)            /*!< Filter Scale Configuration bit 10 */
#define  CAN_FS1_FBS11                      ((uint16_t)0x0800)            /*!< Filter Scale Configuration bit 11 */
#define  CAN_FS1_FBS12                      ((uint16_t)0x1000)            /*!< Filter Scale Configuration bit 12 */
#define  CAN_FS1_FBS13                      ((uint16_t)0x2000)            /*!< Filter Scale Configuration bit 13 */

/******************  Bit definition for CAN_FFA1 register  *******************/
#define  CAN_FFA1_FAF                       ((uint16_t)0x3FFF)            /*!< Filter FIFO Assignment */
#define  CAN_FFA1_FAF0                      ((uint16_t)0x0001)            /*!< Filter FIFO Assignment for Filter 0 */
#define  CAN_FFA1_FAF1                      ((uint16_t)0x0002)            /*!< Filter FIFO Assignment for Filter 1 */
#define  CAN_FFA1_FAF2                      ((uint16_t)0x0004)            /*!< Filter FIFO Assignment for Filter 2 */
#define  CAN_FFA1_FAF3                      ((uint16_t)0x0008)            /*!< Filter FIFO Assignment for Filter 3 */
#define  CAN_FFA1_FAF4                      ((uint16_t)0x0010)            /*!< Filter FIFO Assignment for Filter 4 */
#define  CAN_FFA1_FAF5                      ((uint16_t)0x0020)            /*!< Filter FIFO Assignment for Filter 5 */
#define  CAN_FFA1_FAF6                      ((uint16_t)0x0040)            /*!< Filter FIFO Assignment for Filter 6 */
#define  CAN_FFA1_FAF7                      ((uint16_t)0x0080)            /*!< Filter FIFO Assignment for Filter 7 */
#define  CAN_FFA1_FAF8                      ((uint16_t)0x0100)            /*!< Filter FIFO Assignment for Filter 8 */
#define  CAN_FFA1_FAF9                      ((uint16_t)0x0200)            /*!< Filter FIFO Assignment for Filter 9 */
#define  CAN_FFA1_FAF10                     ((uint16_t)0x0400)            /*!< Filter FIFO Assignment for Filter 10 */
#define  CAN_FFA1_FAF11                     ((uint16_t)0x0800)            /*!< Filter FIFO Assignment for Filter 11 */
#define  CAN_FFA1_FAF12                     ((uint16_t)0x1000)            /*!< Filter FIFO Assignment for Filter 12 */
#define  CAN_FFA1_FAF13                     ((uint16_t)0x2000)            /*!< Filter FIFO Assignment for Filter 13 */

/*******************  Bit definition for CAN_FA1 register  *******************/
#define  CAN_FA1_FEN                        ((uint16_t)0x3FFF)            /*!< Filter Active */
#define  CAN_FA1_FEN0                       ((uint16_t)0x0001)            /*!< Filter 0 Active */
#define  CAN_FA1_FEN1                       ((uint16_t)0x0002)            /*!< Filter 1 Active */
#define  CAN_FA1_FEN2                       ((uint16_t)0x0004)            /*!< Filter 2 Active */
#define  CAN_FA1_FEN3                       ((uint16_t)0x0008)            /*!< Filter 3 Active */
#define  CAN_FA1_FEN4                       ((uint16_t)0x0010)            /*!< Filter 4 Active */
#define  CAN_FA1_FEN5                       ((uint16_t)0x0020)            /*!< Filter 5 Active */
#define  CAN_FA1_FEN6                       ((uint16_t)0x0040)            /*!< Filter 6 Active */
#define  CAN_FA1_FEN7                       ((uint16_t)0x0080)            /*!< Filter 7 Active */
#define  CAN_FA1_FEN8                       ((uint16_t)0x0100)            /*!< Filter 8 Active */
#define  CAN_FA1_FEN9                       ((uint16_t)0x0200)            /*!< Filter 9 Active */
#define  CAN_FA1_FEN10                      ((uint16_t)0x0400)            /*!< Filter 10 Active */
#define  CAN_FA1_FEN11                      ((uint16_t)0x0800)            /*!< Filter 11 Active */
#define  CAN_FA1_FEN12                      ((uint16_t)0x1000)            /*!< Filter 12 Active */
#define  CAN_FA1_FEN13                      ((uint16_t)0x2000)            /*!< Filter 13 Active */

/*******************  Bit definition for CAN_FB0R1 register  *******************/
#define  CAN_FB0R1_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB0R1_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB0R1_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB0R1_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB0R1_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB0R1_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB0R1_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB0R1_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB0R1_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB0R1_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB0R1_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB0R1_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB0R1_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB0R1_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB0R1_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB0R1_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB0R1_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB0R1_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB0R1_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB0R1_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB0R1_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB0R1_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB0R1_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB0R1_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB0R1_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB0R1_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB0R1_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB0R1_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB0R1_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB0R1_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB0R1_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB0R1_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB1R1 register  *******************/
#define  CAN_FB1R1_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB1R1_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB1R1_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB1R1_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB1R1_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB1R1_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB1R1_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB1R1_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB1R1_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB1R1_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB1R1_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB1R1_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB1R1_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB1R1_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB1R1_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB1R1_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB1R1_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB1R1_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB1R1_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB1R1_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB1R1_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB1R1_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB1R1_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB1R1_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB1R1_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB1R1_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB1R1_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB1R1_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB1R1_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB1R1_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB1R1_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB1R1_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB2R1 register  *******************/
#define  CAN_FB2R1_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB2R1_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB2R1_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB2R1_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB2R1_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB2R1_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB2R1_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB2R1_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB2R1_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB2R1_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB2R1_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB2R1_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB2R1_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB2R1_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB2R1_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB2R1_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB2R1_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB2R1_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB2R1_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB2R1_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB2R1_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB2R1_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB2R1_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB2R1_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB2R1_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB2R1_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB2R1_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB2R1_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB2R1_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB2R1_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB2R1_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB2R1_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB3R1 register  *******************/
#define  CAN_FB3R1_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB3R1_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB3R1_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB3R1_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB3R1_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB3R1_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB3R1_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB3R1_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB3R1_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB3R1_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB3R1_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB3R1_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB3R1_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB3R1_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB3R1_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB3R1_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB3R1_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB3R1_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB3R1_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB3R1_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB3R1_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB3R1_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB3R1_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB3R1_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB3R1_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB3R1_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB3R1_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB3R1_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB3R1_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB3R1_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB3R1_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB3R1_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB4R1 register  *******************/
#define  CAN_FB4R1_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB4R1_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB4R1_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB4R1_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB4R1_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB4R1_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB4R1_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB4R1_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB4R1_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB4R1_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB4R1_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB4R1_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB4R1_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB4R1_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB4R1_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB4R1_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB4R1_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB4R1_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB4R1_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB4R1_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB4R1_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB4R1_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB4R1_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB4R1_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB4R1_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB4R1_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB4R1_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB4R1_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB4R1_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB4R1_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB4R1_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB4R1_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB5R1 register  *******************/
#define  CAN_FB5R1_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB5R1_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB5R1_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB5R1_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB5R1_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB5R1_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB5R1_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB5R1_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB5R1_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB5R1_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB5R1_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB5R1_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB5R1_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB5R1_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB5R1_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB5R1_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB5R1_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB5R1_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB5R1_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB5R1_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB5R1_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB5R1_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB5R1_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB5R1_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB5R1_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB5R1_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB5R1_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB5R1_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB5R1_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB5R1_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB5R1_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB5R1_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB6R1 register  *******************/
#define  CAN_FB6R1_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB6R1_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB6R1_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB6R1_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB6R1_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB6R1_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB6R1_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB6R1_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB6R1_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB6R1_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB6R1_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB6R1_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB6R1_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB6R1_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB6R1_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB6R1_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB6R1_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB6R1_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB6R1_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB6R1_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB6R1_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB6R1_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB6R1_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB6R1_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB6R1_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB6R1_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB6R1_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB6R1_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB6R1_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB6R1_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB6R1_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB6R1_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB7R1 register  *******************/
#define  CAN_FB7R1_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB7R1_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB7R1_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB7R1_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB7R1_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB7R1_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB7R1_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB7R1_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB7R1_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB7R1_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB7R1_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB7R1_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB7R1_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB7R1_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB7R1_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB7R1_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB7R1_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB7R1_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB7R1_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB7R1_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB7R1_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB7R1_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB7R1_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB7R1_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB7R1_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB7R1_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB7R1_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB7R1_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB7R1_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB7R1_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB7R1_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB7R1_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB8R1 register  *******************/
#define  CAN_FB8R1_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB8R1_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB8R1_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB8R1_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB8R1_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB8R1_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB8R1_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB8R1_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB8R1_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB8R1_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB8R1_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB8R1_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB8R1_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB8R1_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB8R1_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB8R1_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB8R1_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB8R1_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB8R1_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB8R1_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB8R1_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB8R1_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB8R1_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB8R1_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB8R1_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB8R1_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB8R1_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB8R1_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB8R1_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB8R1_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB8R1_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB8R1_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB9R1 register  *******************/
#define  CAN_FB9R1_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB9R1_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB9R1_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB9R1_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB9R1_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB9R1_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB9R1_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB9R1_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB9R1_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB9R1_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB9R1_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB9R1_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB9R1_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB9R1_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB9R1_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB9R1_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB9R1_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB9R1_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB9R1_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB9R1_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB9R1_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB9R1_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB9R1_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB9R1_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB9R1_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB9R1_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB9R1_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB9R1_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB9R1_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB9R1_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB9R1_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB9R1_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB10R1 register  ******************/
#define  CAN_FB10R1_FD0                     ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB10R1_FD1                     ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB10R1_FD2                     ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB10R1_FD3                     ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB10R1_FD4                     ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB10R1_FD5                     ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB10R1_FD6                     ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB10R1_FD7                     ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB10R1_FD8                     ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB10R1_FD9                     ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB10R1_FD10                    ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB10R1_FD11                    ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB10R1_FD12                    ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB10R1_FD13                    ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB10R1_FD14                    ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB10R1_FD15                    ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB10R1_FD16                    ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB10R1_FD17                    ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB10R1_FD18                    ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB10R1_FD19                    ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB10R1_FD20                    ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB10R1_FD21                    ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB10R1_FD22                    ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB10R1_FD23                    ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB10R1_FD24                    ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB10R1_FD25                    ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB10R1_FD26                    ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB10R1_FD27                    ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB10R1_FD28                    ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB10R1_FD29                    ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB10R1_FD30                    ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB10R1_FD31                    ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB11R1 register  ******************/
#define  CAN_FB11R1_FD0                     ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB11R1_FD1                     ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB11R1_FD2                     ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB11R1_FD3                     ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB11R1_FD4                     ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB11R1_FD5                     ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB11R1_FD6                     ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB11R1_FD7                     ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB11R1_FD8                     ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB11R1_FD9                     ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB11R1_FD10                    ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB11R1_FD11                    ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB11R1_FD12                    ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB11R1_FD13                    ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB11R1_FD14                    ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB11R1_FD15                    ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB11R1_FD16                    ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB11R1_FD17                    ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB11R1_FD18                    ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB11R1_FD19                    ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB11R1_FD20                    ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB11R1_FD21                    ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB11R1_FD22                    ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB11R1_FD23                    ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB11R1_FD24                    ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB11R1_FD25                    ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB11R1_FD26                    ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB11R1_FD27                    ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB11R1_FD28                    ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB11R1_FD29                    ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB11R1_FD30                    ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB11R1_FD31                    ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB12R1 register  ******************/
#define  CAN_FB12R1_FD0                     ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB12R1_FD1                     ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB12R1_FD2                     ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB12R1_FD3                     ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB12R1_FD4                     ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB12R1_FD5                     ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB12R1_FD6                     ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB12R1_FD7                     ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB12R1_FD8                     ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB12R1_FD9                     ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB12R1_FD10                    ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB12R1_FD11                    ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB12R1_FD12                    ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB12R1_FD13                    ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB12R1_FD14                    ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB12R1_FD15                    ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB12R1_FD16                    ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB12R1_FD17                    ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB12R1_FD18                    ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB12R1_FD19                    ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB12R1_FD20                    ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB12R1_FD21                    ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB12R1_FD22                    ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB12R1_FD23                    ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB12R1_FD24                    ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB12R1_FD25                    ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB12R1_FD26                    ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB12R1_FD27                    ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB12R1_FD28                    ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB12R1_FD29                    ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB12R1_FD30                    ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB12R1_FD31                    ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB13R1 register  ******************/
#define  CAN_FB13R1_FD0                     ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB13R1_FD1                     ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB13R1_FD2                     ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB13R1_FD3                     ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB13R1_FD4                     ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB13R1_FD5                     ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB13R1_FD6                     ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB13R1_FD7                     ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB13R1_FD8                     ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB13R1_FD9                     ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB13R1_FD10                    ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB13R1_FD11                    ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB13R1_FD12                    ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB13R1_FD13                    ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB13R1_FD14                    ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB13R1_FD15                    ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB13R1_FD16                    ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB13R1_FD17                    ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB13R1_FD18                    ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB13R1_FD19                    ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB13R1_FD20                    ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB13R1_FD21                    ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB13R1_FD22                    ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB13R1_FD23                    ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB13R1_FD24                    ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB13R1_FD25                    ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB13R1_FD26                    ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB13R1_FD27                    ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB13R1_FD28                    ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB13R1_FD29                    ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB13R1_FD30                    ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB13R1_FD31                    ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB0R2 register  *******************/
#define  CAN_FB0R2_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB0R2_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB0R2_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB0R2_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB0R2_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB0R2_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB0R2_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB0R2_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB0R2_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB0R2_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB0R2_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB0R2_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB0R2_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB0R2_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB0R2_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB0R2_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB0R2_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB0R2_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB0R2_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB0R2_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB0R2_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB0R2_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB0R2_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB0R2_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB0R2_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB0R2_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB0R2_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB0R2_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB0R2_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB0R2_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB0R2_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB0R2_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB1R2 register  *******************/
#define  CAN_FB1R2_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB1R2_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB1R2_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB1R2_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB1R2_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB1R2_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB1R2_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB1R2_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB1R2_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB1R2_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB1R2_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB1R2_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB1R2_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB1R2_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB1R2_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB1R2_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB1R2_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB1R2_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB1R2_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB1R2_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB1R2_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB1R2_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB1R2_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB1R2_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB1R2_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB1R2_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB1R2_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB1R2_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB1R2_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB1R2_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB1R2_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB1R2_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB2R2 register  *******************/
#define  CAN_FB2R2_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB2R2_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB2R2_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB2R2_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB2R2_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB2R2_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB2R2_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB2R2_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB2R2_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB2R2_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB2R2_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB2R2_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB2R2_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB2R2_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB2R2_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB2R2_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB2R2_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB2R2_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB2R2_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB2R2_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB2R2_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB2R2_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB2R2_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB2R2_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB2R2_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB2R2_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB2R2_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB2R2_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB2R2_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB2R2_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB2R2_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB2R2_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB3R2 register  *******************/
#define  CAN_FB3R2_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB3R2_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB3R2_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB3R2_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB3R2_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB3R2_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB3R2_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB3R2_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB3R2_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB3R2_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB3R2_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB3R2_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB3R2_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB3R2_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB3R2_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB3R2_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB3R2_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB3R2_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB3R2_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB3R2_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB3R2_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB3R2_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB3R2_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB3R2_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB3R2_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB3R2_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB3R2_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB3R2_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB3R2_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB3R2_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB3R2_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB3R2_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB4R2 register  *******************/
#define  CAN_FB4R2_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB4R2_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB4R2_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB4R2_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB4R2_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB4R2_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB4R2_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB4R2_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB4R2_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB4R2_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB4R2_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB4R2_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB4R2_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB4R2_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB4R2_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB4R2_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB4R2_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB4R2_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB4R2_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB4R2_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB4R2_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB4R2_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB4R2_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB4R2_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB4R2_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB4R2_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB4R2_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB4R2_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB4R2_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB4R2_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB4R2_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB4R2_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB5R2 register  *******************/
#define  CAN_FB5R2_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB5R2_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB5R2_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB5R2_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB5R2_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB5R2_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB5R2_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB5R2_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB5R2_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB5R2_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB5R2_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB5R2_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB5R2_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB5R2_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB5R2_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB5R2_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB5R2_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB5R2_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB5R2_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB5R2_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB5R2_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB5R2_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB5R2_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB5R2_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB5R2_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB5R2_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB5R2_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB5R2_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB5R2_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB5R2_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB5R2_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB5R2_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB6R2 register  *******************/
#define  CAN_FB6R2_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB6R2_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB6R2_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB6R2_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB6R2_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB6R2_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB6R2_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB6R2_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB6R2_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB6R2_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB6R2_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB6R2_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB6R2_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB6R2_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB6R2_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB6R2_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB6R2_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB6R2_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB6R2_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB6R2_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB6R2_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB6R2_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB6R2_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB6R2_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB6R2_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB6R2_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB6R2_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB6R2_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB6R2_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB6R2_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB6R2_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB6R2_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB7R2 register  *******************/
#define  CAN_FB7R2_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB7R2_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB7R2_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB7R2_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB7R2_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB7R2_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB7R2_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB7R2_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB7R2_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB7R2_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB7R2_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB7R2_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB7R2_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB7R2_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB7R2_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB7R2_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB7R2_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB7R2_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB7R2_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB7R2_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB7R2_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB7R2_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB7R2_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB7R2_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB7R2_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB7R2_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB7R2_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB7R2_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB7R2_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB7R2_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB7R2_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB7R2_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB8R2 register  *******************/
#define  CAN_FB8R2_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB8R2_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB8R2_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB8R2_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB8R2_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB8R2_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB8R2_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB8R2_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB8R2_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB8R2_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB8R2_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB8R2_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB8R2_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB8R2_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB8R2_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB8R2_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB8R2_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB8R2_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB8R2_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB8R2_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB8R2_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB8R2_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB8R2_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB8R2_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB8R2_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB8R2_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB8R2_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB8R2_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB8R2_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB8R2_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB8R2_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB8R2_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB9R2 register  *******************/
#define  CAN_FB9R2_FD0                      ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB9R2_FD1                      ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB9R2_FD2                      ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB9R2_FD3                      ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB9R2_FD4                      ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB9R2_FD5                      ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB9R2_FD6                      ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB9R2_FD7                      ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB9R2_FD8                      ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB9R2_FD9                      ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB9R2_FD10                     ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB9R2_FD11                     ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB9R2_FD12                     ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB9R2_FD13                     ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB9R2_FD14                     ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB9R2_FD15                     ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB9R2_FD16                     ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB9R2_FD17                     ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB9R2_FD18                     ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB9R2_FD19                     ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB9R2_FD20                     ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB9R2_FD21                     ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB9R2_FD22                     ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB9R2_FD23                     ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB9R2_FD24                     ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB9R2_FD25                     ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB9R2_FD26                     ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB9R2_FD27                     ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB9R2_FD28                     ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB9R2_FD29                     ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB9R2_FD30                     ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB9R2_FD31                     ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB10R2 register  ******************/
#define  CAN_FB10R2_FD0                     ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB10R2_FD1                     ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB10R2_FD2                     ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB10R2_FD3                     ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB10R2_FD4                     ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB10R2_FD5                     ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB10R2_FD6                     ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB10R2_FD7                     ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB10R2_FD8                     ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB10R2_FD9                     ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB10R2_FD10                    ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB10R2_FD11                    ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB10R2_FD12                    ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB10R2_FD13                    ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB10R2_FD14                    ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB10R2_FD15                    ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB10R2_FD16                    ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB10R2_FD17                    ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB10R2_FD18                    ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB10R2_FD19                    ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB10R2_FD20                    ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB10R2_FD21                    ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB10R2_FD22                    ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB10R2_FD23                    ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB10R2_FD24                    ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB10R2_FD25                    ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB10R2_FD26                    ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB10R2_FD27                    ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB10R2_FD28                    ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB10R2_FD29                    ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB10R2_FD30                    ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB10R2_FD31                    ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB11R2 register  ******************/
#define  CAN_FB11R2_FD0                     ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB11R2_FD1                     ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB11R2_FD2                     ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB11R2_FD3                     ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB11R2_FD4                     ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB11R2_FD5                     ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB11R2_FD6                     ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB11R2_FD7                     ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB11R2_FD8                     ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB11R2_FD9                     ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB11R2_FD10                    ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB11R2_FD11                    ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB11R2_FD12                    ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB11R2_FD13                    ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB11R2_FD14                    ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB11R2_FD15                    ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB11R2_FD16                    ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB11R2_FD17                    ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB11R2_FD18                    ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB11R2_FD19                    ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB11R2_FD20                    ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB11R2_FD21                    ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB11R2_FD22                    ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB11R2_FD23                    ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB11R2_FD24                    ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB11R2_FD25                    ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB11R2_FD26                    ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB11R2_FD27                    ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB11R2_FD28                    ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB11R2_FD29                    ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB11R2_FD30                    ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB11R2_FD31                    ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB12R2 register  ******************/
#define  CAN_FB12R2_FD0                     ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB12R2_FD1                     ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB12R2_FD2                     ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB12R2_FD3                     ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB12R2_FD4                     ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB12R2_FD5                     ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB12R2_FD6                     ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB12R2_FD7                     ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB12R2_FD8                     ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB12R2_FD9                     ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB12R2_FD10                    ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB12R2_FD11                    ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB12R2_FD12                    ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB12R2_FD13                    ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB12R2_FD14                    ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB12R2_FD15                    ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB12R2_FD16                    ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB12R2_FD17                    ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB12R2_FD18                    ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB12R2_FD19                    ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB12R2_FD20                    ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB12R2_FD21                    ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB12R2_FD22                    ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB12R2_FD23                    ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB12R2_FD24                    ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB12R2_FD25                    ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB12R2_FD26                    ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB12R2_FD27                    ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB12R2_FD28                    ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB12R2_FD29                    ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB12R2_FD30                    ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB12R2_FD31                    ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/*******************  Bit definition for CAN_FB13R2 register  ******************/
#define  CAN_FB13R2_FD0                     ((uint32_t)0x00000001)        /*!< Filter bit 0 */
#define  CAN_FB13R2_FD1                     ((uint32_t)0x00000002)        /*!< Filter bit 1 */
#define  CAN_FB13R2_FD2                     ((uint32_t)0x00000004)        /*!< Filter bit 2 */
#define  CAN_FB13R2_FD3                     ((uint32_t)0x00000008)        /*!< Filter bit 3 */
#define  CAN_FB13R2_FD4                     ((uint32_t)0x00000010)        /*!< Filter bit 4 */
#define  CAN_FB13R2_FD5                     ((uint32_t)0x00000020)        /*!< Filter bit 5 */
#define  CAN_FB13R2_FD6                     ((uint32_t)0x00000040)        /*!< Filter bit 6 */
#define  CAN_FB13R2_FD7                     ((uint32_t)0x00000080)        /*!< Filter bit 7 */
#define  CAN_FB13R2_FD8                     ((uint32_t)0x00000100)        /*!< Filter bit 8 */
#define  CAN_FB13R2_FD9                     ((uint32_t)0x00000200)        /*!< Filter bit 9 */
#define  CAN_FB13R2_FD10                    ((uint32_t)0x00000400)        /*!< Filter bit 10 */
#define  CAN_FB13R2_FD11                    ((uint32_t)0x00000800)        /*!< Filter bit 11 */
#define  CAN_FB13R2_FD12                    ((uint32_t)0x00001000)        /*!< Filter bit 12 */
#define  CAN_FB13R2_FD13                    ((uint32_t)0x00002000)        /*!< Filter bit 13 */
#define  CAN_FB13R2_FD14                    ((uint32_t)0x00004000)        /*!< Filter bit 14 */
#define  CAN_FB13R2_FD15                    ((uint32_t)0x00008000)        /*!< Filter bit 15 */
#define  CAN_FB13R2_FD16                    ((uint32_t)0x00010000)        /*!< Filter bit 16 */
#define  CAN_FB13R2_FD17                    ((uint32_t)0x00020000)        /*!< Filter bit 17 */
#define  CAN_FB13R2_FD18                    ((uint32_t)0x00040000)        /*!< Filter bit 18 */
#define  CAN_FB13R2_FD19                    ((uint32_t)0x00080000)        /*!< Filter bit 19 */
#define  CAN_FB13R2_FD20                    ((uint32_t)0x00100000)        /*!< Filter bit 20 */
#define  CAN_FB13R2_FD21                    ((uint32_t)0x00200000)        /*!< Filter bit 21 */
#define  CAN_FB13R2_FD22                    ((uint32_t)0x00400000)        /*!< Filter bit 22 */
#define  CAN_FB13R2_FD23                    ((uint32_t)0x00800000)        /*!< Filter bit 23 */
#define  CAN_FB13R2_FD24                    ((uint32_t)0x01000000)        /*!< Filter bit 24 */
#define  CAN_FB13R2_FD25                    ((uint32_t)0x02000000)        /*!< Filter bit 25 */
#define  CAN_FB13R2_FD26                    ((uint32_t)0x04000000)        /*!< Filter bit 26 */
#define  CAN_FB13R2_FD27                    ((uint32_t)0x08000000)        /*!< Filter bit 27 */
#define  CAN_FB13R2_FD28                    ((uint32_t)0x10000000)        /*!< Filter bit 28 */
#define  CAN_FB13R2_FD29                    ((uint32_t)0x20000000)        /*!< Filter bit 29 */
#define  CAN_FB13R2_FD30                    ((uint32_t)0x40000000)        /*!< Filter bit 30 */
#define  CAN_FB13R2_FD31                    ((uint32_t)0x80000000)        /*!< Filter bit 31 */

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for SPI_CTRL1 register  ********************/
#define  SPI_CTRL1_CPHA                     ((uint16_t)0x0001)            /*!< Clock Phase */
#define  SPI_CTRL1_CPOL                     ((uint16_t)0x0002)            /*!< Clock Polarity */
#define  SPI_CTRL1_MSTEN                    ((uint16_t)0x0004)            /*!< Master Selection */

#define  SPI_CTRL1_MCLKP                    ((uint16_t)0x0038)            /*!< BR[2:0] bits (Baud Rate Control) */
#define  SPI_CTRL1_MCLKP_0                  ((uint16_t)0x0008)            /*!< Bit 0 */
#define  SPI_CTRL1_MCLKP_1                  ((uint16_t)0x0010)            /*!< Bit 1 */
#define  SPI_CTRL1_MCLKP_2                  ((uint16_t)0x0020)            /*!< Bit 2 */

#define  SPI_CTRL1_SPIEN                    ((uint16_t)0x0040)            /*!< SPI Enable */
#define  SPI_CTRL1_LSBEN                    ((uint16_t)0x0080)            /*!< Frame Format */
#define  SPI_CTRL1_ISS                      ((uint16_t)0x0100)            /*!< Internal slave select */
#define  SPI_CTRL1_SWNSSEN                  ((uint16_t)0x0200)            /*!< Software slave management */
#define  SPI_CTRL1_RONLY                    ((uint16_t)0x0400)            /*!< Receive only */
#define  SPI_CTRL1_DFF16                    ((uint16_t)0x0800)            /*!< Data Frame Format */
#define  SPI_CTRL1_CTN                      ((uint16_t)0x1000)            /*!< Transmit CRC next */
#define  SPI_CTRL1_CCE                      ((uint16_t)0x2000)            /*!< Hardware CRC calculation enable */
#define  SPI_CTRL1_BDOE                     ((uint16_t)0x4000)            /*!< Output enable in bidirectional mode */
#define  SPI_CTRL1_BDMODE                   ((uint16_t)0x8000)            /*!< Bidirectional data mode enable */

/*******************  Bit definition for SPI_CTRL2 register  ********************/
#define  SPI_CTRL2_DMAREN                   ((uint16_t)0x0001)            /*!< Rx Buffer DMA Enable */
#define  SPI_CTRL2_DMATEN                   ((uint16_t)0x0002)            /*!< Tx Buffer DMA Enable */
#define  SPI_CTRL2_NSSOE                    ((uint16_t)0x0004)            /*!< SS Output Enable */
#define  SPI_CTRL2_ERRIE                    ((uint16_t)0x0020)            /*!< Error Interrupt Enable */
#define  SPI_CTRL2_RNEIE                    ((uint16_t)0x0040)            /*!< RX buffer Not Empty Interrupt Enable */
#define  SPI_CTRL2_TEIE                     ((uint16_t)0x0080)            /*!< Tx buffer Empty Interrupt Enable */
#define  SPI_CTRL2_MCLKP_EXT                ((uint16_t)0x0100)            /*!< BR[3] bits (Baud Rate Control) */
#define  SPI_CTRL2_MCLKP_3                  ((uint16_t)0x0100)            /*!< Bit 3 */

/********************  Bit definition for SPI_STS register  ********************/
#define  SPI_STS_RNE                        ((uint8_t)0x01)               /*!< Receive buffer Not Empty */
#define  SPI_STS_TE                         ((uint8_t)0x02)               /*!< Transmit buffer Empty */
#define  SPI_STS_I2SCS                      ((uint8_t)0x04)               /*!< Channel side */
#define  SPI_STS_UDR                        ((uint8_t)0x08)               /*!< Underrun flag */
#define  SPI_STS_CERR                       ((uint8_t)0x10)               /*!< CRC Error flag */
#define  SPI_STS_MODF                       ((uint8_t)0x20)               /*!< Mode fault */
#define  SPI_STS_OVR                        ((uint8_t)0x40)               /*!< Overrun flag */
#define  SPI_STS_BSY                        ((uint8_t)0x80)               /*!< Busy flag */

/********************  Bit definition for SPI_DT register  ********************/
#define  SPI_DT_DT                          ((uint16_t)0xFFFF)            /*!< Data Register */

/*******************  Bit definition for SPI_CPOLY register  ******************/
#define  SPI_CPOLY_CPOLY                    ((uint16_t)0xFFFF)            /*!< CRC polynomial register */

/******************  Bit definition for SPI_RCRC register  ******************/
#define  SPI_RCRC_RCRC                      ((uint16_t)0xFFFF)            /*!< Rx CRC Register */

/******************  Bit definition for SPI_TCRC register  ******************/
#define  SPI_TCRC_TCRC                      ((uint16_t)0xFFFF)            /*!< Tx CRC Register */

/******************  Bit definition for SPI_I2SCTRL register  *****************/
#define  SPI_I2SCTRL_CHLEN                  ((uint16_t)0x0001)            /*!< Channel length (number of bits per audio channel) */

#define  SPI_I2SCTRL_DLEN                   ((uint16_t)0x0006)            /*!< DATLEN[1:0] bits (Data length to be transferred) */
#define  SPI_I2SCTRL_DLEN_0                 ((uint16_t)0x0002)            /*!< Bit 0 */
#define  SPI_I2SCTRL_DLEN_1                 ((uint16_t)0x0004)            /*!< Bit 1 */

#define  SPI_I2SCTRL_CPOL                   ((uint16_t)0x0008)            /*!< steady state clock polarity */

#define  SPI_I2SCTRL_I2SAP                  ((uint16_t)0x0030)            /*!< I2SSTD[1:0] bits (I2S standard selection) */
#define  SPI_I2SCTRL_I2SAP_0                ((uint16_t)0x0010)            /*!< Bit 0 */
#define  SPI_I2SCTRL_I2SAP_1                ((uint16_t)0x0020)            /*!< Bit 1 */

#define  SPI_I2SCTRL_PCMSYNCSEL             ((uint16_t)0x0080)            /*!< PCM frame synchronization */

#define  SPI_I2SCTRL_I2SMOD                 ((uint16_t)0x0300)            /*!< I2SCFG[1:0] bits (I2S configuration mode) */
#define  SPI_I2SCTRL_I2SMOD_0               ((uint16_t)0x0100)            /*!< Bit 0 */
#define  SPI_I2SCTRL_I2SMOD_1               ((uint16_t)0x0200)            /*!< Bit 1 */

#define  SPI_I2SCTRL_I2SEN                  ((uint16_t)0x0400)            /*!< I2S Enable */
#define  SPI_I2SCTRL_I2SSEL                 ((uint16_t)0x0800)            /*!< I2S mode selection */

/******************  Bit definition for SPI_I2SCLKP register  *******************/
#define  SPI_I2SCLKP_I2SDIV                 ((uint16_t)0x0CFF)            /*!< I2S Linear prescaler */
#define  SPI_I2SCLKP_I2SODD                 ((uint16_t)0x0100)            /*!< Odd factor for the prescaler */
#define  SPI_I2SCLKP_I2SMCLKOE              ((uint16_t)0x0200)            /*!< Master Clock Output Enable */




/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for I2C_CTRL1 register  ********************/
#define  I2C_CTRL1_PEN                      ((uint16_t)0x0001)            /*!< Peripheral Enable */
#define  I2C_CTRL1_SMBMODE                  ((uint16_t)0x0002)            /*!< SMBus Mode */
#define  I2C_CTRL1_SMBTYPE                  ((uint16_t)0x0008)            /*!< SMBus Type */
#define  I2C_CTRL1_ARPEN                    ((uint16_t)0x0010)            /*!< ARP Enable */
#define  I2C_CTRL1_PECEN                    ((uint16_t)0x0020)            /*!< PEC Enable */
#define  I2C_CTRL1_GCEN                     ((uint16_t)0x0040)            /*!< General Call Enable */
#define  I2C_CTRL1_NOCLKSTRETCH             ((uint16_t)0x0080)            /*!< Clock Stretching Disable (Slave mode) */
#define  I2C_CTRL1_STARTGEN                 ((uint16_t)0x0100)            /*!< Start Generation */
#define  I2C_CTRL1_STOPGEN                  ((uint16_t)0x0200)            /*!< Stop Generation */
#define  I2C_CTRL1_ACKEN                    ((uint16_t)0x0400)            /*!< Acknowledge Enable */
#define  I2C_CTRL1_POSEN                    ((uint16_t)0x0800)            /*!< Acknowledge/PEC Position (for data reception) */
#define  I2C_CTRL1_PECTRA                   ((uint16_t)0x1000)            /*!< Packet Error Checking */
#define  I2C_CTRL1_SMBALERT                 ((uint16_t)0x2000)            /*!< SMBus Alert */
#define  I2C_CTRL1_SWRESET                  ((uint16_t)0x8000)            /*!< Software Reset */

/*******************  Bit definition for I2C_CTRL2 register  ********************/
#define  I2C_CTRL2_CLKFREQ                  ((uint16_t)0x003F)            /*!< FREQ[7:0] bits (Peripheral Clock Frequency) */
#define  I2C_CTRL2_CLKFREQ_B0               ((uint16_t)0x0001)            /*!< Bit 0 */
#define  I2C_CTRL2_CLKFREQ_B1               ((uint16_t)0x0002)            /*!< Bit 1 */
#define  I2C_CTRL2_CLKFREQ_B2               ((uint16_t)0x0004)            /*!< Bit 2 */
#define  I2C_CTRL2_CLKFREQ_B3               ((uint16_t)0x0008)            /*!< Bit 3 */
#define  I2C_CTRL2_CLKFREQ_B4               ((uint16_t)0x0010)            /*!< Bit 4 */
#define  I2C_CTRL2_CLKFREQ_B5               ((uint16_t)0x0020)            /*!< Bit 5 */
#define  I2C_CTRL2_CLKFREQ_B6               ((uint16_t)0x0040)            /*!< Bit 6 */
#define  I2C_CTRL2_CLKFREQ_B7               ((uint16_t)0x0080)            /*!< Bit 7 */

#define  I2C_CTRL2_ERRITEN                  ((uint16_t)0x0100)            /*!< Error Interrupt Enable */
#define  I2C_CTRL2_EVTITEN                  ((uint16_t)0x0200)            /*!< Event Interrupt Enable */
#define  I2C_CTRL2_BUFITEN                  ((uint16_t)0x0400)            /*!< Buffer Interrupt Enable */
#define  I2C_CTRL2_DMAEN                    ((uint16_t)0x0800)            /*!< DMA Requests Enable */
#define  I2C_CTRL2_DMALAST                  ((uint16_t)0x1000)            /*!< DMA Last Transfer */

/*******************  Bit definition for I2C_OADDR1 register  *******************/
#define  I2C_OADDR1_ADDR1_7                 ((uint16_t)0x00FE)            /*!< Interface Address */
#define  I2C_OADDR1_ADDR8_9                 ((uint16_t)0x0300)            /*!< Interface Address */

#define  I2C_OADDR1_ADDR_B0                 ((uint16_t)0x0001)            /*!< Bit 0 */
#define  I2C_OADDR1_ADDR_B1                 ((uint16_t)0x0002)            /*!< Bit 1 */
#define  I2C_OADDR1_ADDR_B2                 ((uint16_t)0x0004)            /*!< Bit 2 */
#define  I2C_OADDR1_ADDR_B3                 ((uint16_t)0x0008)            /*!< Bit 3 */
#define  I2C_OADDR1_ADDR_B4                 ((uint16_t)0x0010)            /*!< Bit 4 */
#define  I2C_OADDR1_ADDR_B5                 ((uint16_t)0x0020)            /*!< Bit 5 */
#define  I2C_OADDR1_ADDR_B6                 ((uint16_t)0x0040)            /*!< Bit 6 */
#define  I2C_OADDR1_ADDR_B7                 ((uint16_t)0x0080)            /*!< Bit 7 */
#define  I2C_OADDR1_ADDR_B8                 ((uint16_t)0x0100)            /*!< Bit 8 */
#define  I2C_OADDR1_ADDR_B9                 ((uint16_t)0x0200)            /*!< Bit 9 */

#define  I2C_OADDR1_ADDRMODE                ((uint16_t)0x8000)            /*!< Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OADDR2 register  *******************/
#define  I2C_OADDR2_DUALEN                  ((uint8_t)0x01)               /*!< Dual addressing mode enable */
#define  I2C_OADDR2_ADDR2                   ((uint8_t)0xFE)               /*!< Interface address */

/********************  Bit definition for I2C_DT register  ********************/
#define  I2C_DT_DT                          ((uint8_t)0xFF)               /*!< 8-bit Data Register */

/*******************  Bit definition for I2C_STS1 register  ********************/
#define  I2C_STS1_STARTF                    ((uint16_t)0x0001)            /*!< Start Bit (Master mode) */
#define  I2C_STS1_ADDRF                     ((uint16_t)0x0002)            /*!< Address sent (master mode)/matched (slave mode) */
#define  I2C_STS1_BTFF                      ((uint16_t)0x0004)            /*!< Byte Transfer Finished */
#define  I2C_STS1_ADDR10F                   ((uint16_t)0x0008)            /*!< 10-bit header sent (Master mode) */
#define  I2C_STS1_STOPF                     ((uint16_t)0x0010)            /*!< Stop detection (Slave mode) */
#define  I2C_STS1_RDNE                      ((uint16_t)0x0040)            /*!< Data Register not Empty (receivers) */
#define  I2C_STS1_TDE                       ((uint16_t)0x0080)            /*!< Data Register Empty (transmitters) */
#define  I2C_STS1_BUSERR                    ((uint16_t)0x0100)            /*!< Bus Error */
#define  I2C_STS1_ARLOST                    ((uint16_t)0x0200)            /*!< Arbitration Lost (master mode) */
#define  I2C_STS1_ACKFAIL                   ((uint16_t)0x0400)            /*!< Acknowledge Failure */
#define  I2C_STS1_OVRUN                     ((uint16_t)0x0800)            /*!< Overrun/Underrun */
#define  I2C_STS1_PECERR                    ((uint16_t)0x1000)            /*!< PEC Error in reception */
#define  I2C_STS1_TIMOUT                    ((uint16_t)0x4000)            /*!< Timeout or Tlow Error */
#define  I2C_STS1_SMBALERTF                 ((uint16_t)0x8000)            /*!< SMBus Alert */

/*******************  Bit definition for I2C_STS2 register  ********************/
#define  I2C_STS2_MSF                       ((uint16_t)0x0001)            /*!< Master/Slave */
#define  I2C_STS2_BUSYF                     ((uint16_t)0x0002)            /*!< Bus Busy */
#define  I2C_STS2_TRF                       ((uint16_t)0x0004)            /*!< Transmitter/Receiver */
#define  I2C_STS2_GCADDRF                   ((uint16_t)0x0010)            /*!< General Call Address (Slave mode) */
#define  I2C_STS2_SMBDEFTADDRF              ((uint16_t)0x0020)            /*!< SMBus Device Default Address (Slave mode) */
#define  I2C_STS2_SMBHOSTADDRF              ((uint16_t)0x0040)            /*!< SMBus Host Header (Slave mode) */
#define  I2C_STS2_DUALF                     ((uint16_t)0x0080)            /*!< Dual Flag (Slave mode) */
#define  I2C_STS2_PECVAL                    ((uint16_t)0xFF00)            /*!< Packet Error Checking Register */

/*******************  Bit definition for I2C_CLKCTRL register  ********************/
#define  I2C_CLKCTRL_CLKCTRL                ((uint16_t)0x0FFF)            /*!< Clock Control Register in Fast/Standard mode (Master mode) */
#define  I2C_CLKCTRL_FMDUTY                 ((uint16_t)0x4000)            /*!< Fast Mode Duty Cycle */
#define  I2C_CLKCTRL_FSMODE                 ((uint16_t)0x8000)            /*!< I2C Master Mode Selection */

/******************  Bit definition for I2C_TMRISE register  *******************/
#define  I2C_TMRISE_TMRISE                  ((uint8_t)0x3F)               /*!< Maximum Rise Time in Fast/Standard mode (Master mode) */

///////////////////////////////////////////////////////////////////////////////
// copied from stm file
/*******************  Bit definition for I2C_CR1 register  ********************/
#define I2C_CR1_PE_Pos                      (0U)                               
#define I2C_CR1_PE_Msk                      (0x1U << I2C_CR1_PE_Pos)           /*!< 0x00000001 */
#define I2C_CR1_PE                          I2C_CR1_PE_Msk                     /*!< Peripheral Enable */
#define I2C_CR1_SMBUS_Pos                   (1U)                               
#define I2C_CR1_SMBUS_Msk                   (0x1U << I2C_CR1_SMBUS_Pos)        /*!< 0x00000002 */
#define I2C_CR1_SMBUS                       I2C_CR1_SMBUS_Msk                  /*!< SMBus Mode */
#define I2C_CR1_SMBTYPE_Pos                 (3U)                               
#define I2C_CR1_SMBTYPE_Msk                 (0x1U << I2C_CR1_SMBTYPE_Pos)      /*!< 0x00000008 */
#define I2C_CR1_SMBTYPE                     I2C_CR1_SMBTYPE_Msk                /*!< SMBus Type */
#define I2C_CR1_ENARP_Pos                   (4U)                               
#define I2C_CR1_ENARP_Msk                   (0x1U << I2C_CR1_ENARP_Pos)        /*!< 0x00000010 */
#define I2C_CR1_ENARP                       I2C_CR1_ENARP_Msk                  /*!< ARP Enable */
#define I2C_CR1_ENPEC_Pos                   (5U)                               
#define I2C_CR1_ENPEC_Msk                   (0x1U << I2C_CR1_ENPEC_Pos)        /*!< 0x00000020 */
#define I2C_CR1_ENPEC                       I2C_CR1_ENPEC_Msk                  /*!< PEC Enable */
#define I2C_CR1_ENGC_Pos                    (6U)                               
#define I2C_CR1_ENGC_Msk                    (0x1U << I2C_CR1_ENGC_Pos)         /*!< 0x00000040 */
#define I2C_CR1_ENGC                        I2C_CR1_ENGC_Msk                   /*!< General Call Enable */
#define I2C_CR1_NOSTRETCH_Pos               (7U)                               
#define I2C_CR1_NOSTRETCH_Msk               (0x1U << I2C_CR1_NOSTRETCH_Pos)    /*!< 0x00000080 */
#define I2C_CR1_NOSTRETCH                   I2C_CR1_NOSTRETCH_Msk              /*!< Clock Stretching Disable (Slave mode) */
#define I2C_CR1_START_Pos                   (8U)                               
#define I2C_CR1_START_Msk                   (0x1U << I2C_CR1_START_Pos)        /*!< 0x00000100 */
#define I2C_CR1_START                       I2C_CR1_START_Msk                  /*!< Start Generation */
#define I2C_CR1_STOP_Pos                    (9U)                               
#define I2C_CR1_STOP_Msk                    (0x1U << I2C_CR1_STOP_Pos)         /*!< 0x00000200 */
#define I2C_CR1_STOP                        I2C_CR1_STOP_Msk                   /*!< Stop Generation */
#define I2C_CR1_ACK_Pos                     (10U)                              
#define I2C_CR1_ACK_Msk                     (0x1U << I2C_CR1_ACK_Pos)          /*!< 0x00000400 */
#define I2C_CR1_ACK                         I2C_CR1_ACK_Msk                    /*!< Acknowledge Enable */
#define I2C_CR1_POS_Pos                     (11U)                              
#define I2C_CR1_POS_Msk                     (0x1U << I2C_CR1_POS_Pos)          /*!< 0x00000800 */
#define I2C_CR1_POS                         I2C_CR1_POS_Msk                    /*!< Acknowledge/PEC Position (for data reception) */
#define I2C_CR1_PEC_Pos                     (12U)                              
#define I2C_CR1_PEC_Msk                     (0x1U << I2C_CR1_PEC_Pos)          /*!< 0x00001000 */
#define I2C_CR1_PEC                         I2C_CR1_PEC_Msk                    /*!< Packet Error Checking */
#define I2C_CR1_ALERT_Pos                   (13U)                              
#define I2C_CR1_ALERT_Msk                   (0x1U << I2C_CR1_ALERT_Pos)        /*!< 0x00002000 */
#define I2C_CR1_ALERT                       I2C_CR1_ALERT_Msk                  /*!< SMBus Alert */
#define I2C_CR1_SWRST_Pos                   (15U)                              
#define I2C_CR1_SWRST_Msk                   (0x1U << I2C_CR1_SWRST_Pos)        /*!< 0x00008000 */
#define I2C_CR1_SWRST                       I2C_CR1_SWRST_Msk                  /*!< Software Reset */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_FREQ_Pos                    (0U)                               
#define I2C_CR2_FREQ_Msk                    (0x3FU << I2C_CR2_FREQ_Pos)        /*!< 0x0000003F */
#define I2C_CR2_FREQ                        I2C_CR2_FREQ_Msk                   /*!< FREQ[5:0] bits (Peripheral Clock Frequency) */
#define I2C_CR2_FREQ_0                      (0x01U << I2C_CR2_FREQ_Pos)        /*!< 0x00000001 */
#define I2C_CR2_FREQ_1                      (0x02U << I2C_CR2_FREQ_Pos)        /*!< 0x00000002 */
#define I2C_CR2_FREQ_2                      (0x04U << I2C_CR2_FREQ_Pos)        /*!< 0x00000004 */
#define I2C_CR2_FREQ_3                      (0x08U << I2C_CR2_FREQ_Pos)        /*!< 0x00000008 */
#define I2C_CR2_FREQ_4                      (0x10U << I2C_CR2_FREQ_Pos)        /*!< 0x00000010 */
#define I2C_CR2_FREQ_5                      (0x20U << I2C_CR2_FREQ_Pos)        /*!< 0x00000020 */

#define I2C_CR2_ITERREN_Pos                 (8U)                               
#define I2C_CR2_ITERREN_Msk                 (0x1U << I2C_CR2_ITERREN_Pos)      /*!< 0x00000100 */
#define I2C_CR2_ITERREN                     I2C_CR2_ITERREN_Msk                /*!< Error Interrupt Enable */
#define I2C_CR2_ITEVTEN_Pos                 (9U)                               
#define I2C_CR2_ITEVTEN_Msk                 (0x1U << I2C_CR2_ITEVTEN_Pos)      /*!< 0x00000200 */
#define I2C_CR2_ITEVTEN                     I2C_CR2_ITEVTEN_Msk                /*!< Event Interrupt Enable */
#define I2C_CR2_ITBUFEN_Pos                 (10U)                              
#define I2C_CR2_ITBUFEN_Msk                 (0x1U << I2C_CR2_ITBUFEN_Pos)      /*!< 0x00000400 */
#define I2C_CR2_ITBUFEN                     I2C_CR2_ITBUFEN_Msk                /*!< Buffer Interrupt Enable */
#define I2C_CR2_DMAEN_Pos                   (11U)                              
#define I2C_CR2_DMAEN_Msk                   (0x1U << I2C_CR2_DMAEN_Pos)        /*!< 0x00000800 */
#define I2C_CR2_DMAEN                       I2C_CR2_DMAEN_Msk                  /*!< DMA Requests Enable */
#define I2C_CR2_LAST_Pos                    (12U)                              
#define I2C_CR2_LAST_Msk                    (0x1U << I2C_CR2_LAST_Pos)         /*!< 0x00001000 */
#define I2C_CR2_LAST                        I2C_CR2_LAST_Msk                   /*!< DMA Last Transfer */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define I2C_OAR1_ADD1_7                     0x000000FEU             /*!< Interface Address */
#define I2C_OAR1_ADD8_9                     0x00000300U             /*!< Interface Address */

#define I2C_OAR1_ADD0_Pos                   (0U)                               
#define I2C_OAR1_ADD0_Msk                   (0x1U << I2C_OAR1_ADD0_Pos)        /*!< 0x00000001 */
#define I2C_OAR1_ADD0                       I2C_OAR1_ADD0_Msk                  /*!< Bit 0 */
#define I2C_OAR1_ADD1_Pos                   (1U)                               
#define I2C_OAR1_ADD1_Msk                   (0x1U << I2C_OAR1_ADD1_Pos)        /*!< 0x00000002 */
#define I2C_OAR1_ADD1                       I2C_OAR1_ADD1_Msk                  /*!< Bit 1 */
#define I2C_OAR1_ADD2_Pos                   (2U)                               
#define I2C_OAR1_ADD2_Msk                   (0x1U << I2C_OAR1_ADD2_Pos)        /*!< 0x00000004 */
#define I2C_OAR1_ADD2                       I2C_OAR1_ADD2_Msk                  /*!< Bit 2 */
#define I2C_OAR1_ADD3_Pos                   (3U)                               
#define I2C_OAR1_ADD3_Msk                   (0x1U << I2C_OAR1_ADD3_Pos)        /*!< 0x00000008 */
#define I2C_OAR1_ADD3                       I2C_OAR1_ADD3_Msk                  /*!< Bit 3 */
#define I2C_OAR1_ADD4_Pos                   (4U)                               
#define I2C_OAR1_ADD4_Msk                   (0x1U << I2C_OAR1_ADD4_Pos)        /*!< 0x00000010 */
#define I2C_OAR1_ADD4                       I2C_OAR1_ADD4_Msk                  /*!< Bit 4 */
#define I2C_OAR1_ADD5_Pos                   (5U)                               
#define I2C_OAR1_ADD5_Msk                   (0x1U << I2C_OAR1_ADD5_Pos)        /*!< 0x00000020 */
#define I2C_OAR1_ADD5                       I2C_OAR1_ADD5_Msk                  /*!< Bit 5 */
#define I2C_OAR1_ADD6_Pos                   (6U)                               
#define I2C_OAR1_ADD6_Msk                   (0x1U << I2C_OAR1_ADD6_Pos)        /*!< 0x00000040 */
#define I2C_OAR1_ADD6                       I2C_OAR1_ADD6_Msk                  /*!< Bit 6 */
#define I2C_OAR1_ADD7_Pos                   (7U)                               
#define I2C_OAR1_ADD7_Msk                   (0x1U << I2C_OAR1_ADD7_Pos)        /*!< 0x00000080 */
#define I2C_OAR1_ADD7                       I2C_OAR1_ADD7_Msk                  /*!< Bit 7 */
#define I2C_OAR1_ADD8_Pos                   (8U)                               
#define I2C_OAR1_ADD8_Msk                   (0x1U << I2C_OAR1_ADD8_Pos)        /*!< 0x00000100 */
#define I2C_OAR1_ADD8                       I2C_OAR1_ADD8_Msk                  /*!< Bit 8 */
#define I2C_OAR1_ADD9_Pos                   (9U)                               
#define I2C_OAR1_ADD9_Msk                   (0x1U << I2C_OAR1_ADD9_Pos)        /*!< 0x00000200 */
#define I2C_OAR1_ADD9                       I2C_OAR1_ADD9_Msk                  /*!< Bit 9 */

#define I2C_OAR1_ADDMODE_Pos                (15U)                              
#define I2C_OAR1_ADDMODE_Msk                (0x1U << I2C_OAR1_ADDMODE_Pos)     /*!< 0x00008000 */
#define I2C_OAR1_ADDMODE                    I2C_OAR1_ADDMODE_Msk               /*!< Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define I2C_OAR2_ENDUAL_Pos                 (0U)                               
#define I2C_OAR2_ENDUAL_Msk                 (0x1U << I2C_OAR2_ENDUAL_Pos)      /*!< 0x00000001 */
#define I2C_OAR2_ENDUAL                     I2C_OAR2_ENDUAL_Msk                /*!< Dual addressing mode enable */
#define I2C_OAR2_ADD2_Pos                   (1U)                               
#define I2C_OAR2_ADD2_Msk                   (0x7FU << I2C_OAR2_ADD2_Pos)       /*!< 0x000000FE */
#define I2C_OAR2_ADD2                       I2C_OAR2_ADD2_Msk                  /*!< Interface address */

/********************  Bit definition for I2C_DR register  ********************/
#define I2C_DR_DR_Pos             (0U)                                         
#define I2C_DR_DR_Msk             (0xFFU << I2C_DR_DR_Pos)                     /*!< 0x000000FF */
#define I2C_DR_DR                 I2C_DR_DR_Msk                                /*!< 8-bit Data Register         */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define I2C_SR1_SB_Pos                      (0U)                               
#define I2C_SR1_SB_Msk                      (0x1U << I2C_SR1_SB_Pos)           /*!< 0x00000001 */
#define I2C_SR1_SB                          I2C_SR1_SB_Msk                     /*!< Start Bit (Master mode) */
#define I2C_SR1_ADDR_Pos                    (1U)                               
#define I2C_SR1_ADDR_Msk                    (0x1U << I2C_SR1_ADDR_Pos)         /*!< 0x00000002 */
#define I2C_SR1_ADDR                        I2C_SR1_ADDR_Msk                   /*!< Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_BTF_Pos                     (2U)                               
#define I2C_SR1_BTF_Msk                     (0x1U << I2C_SR1_BTF_Pos)          /*!< 0x00000004 */
#define I2C_SR1_BTF                         I2C_SR1_BTF_Msk                    /*!< Byte Transfer Finished */
#define I2C_SR1_ADD10_Pos                   (3U)                               
#define I2C_SR1_ADD10_Msk                   (0x1U << I2C_SR1_ADD10_Pos)        /*!< 0x00000008 */
#define I2C_SR1_ADD10                       I2C_SR1_ADD10_Msk                  /*!< 10-bit header sent (Master mode) */
#define I2C_SR1_STOPF_Pos                   (4U)                               
#define I2C_SR1_STOPF_Msk                   (0x1U << I2C_SR1_STOPF_Pos)        /*!< 0x00000010 */
#define I2C_SR1_STOPF                       I2C_SR1_STOPF_Msk                  /*!< Stop detection (Slave mode) */
#define I2C_SR1_RXNE_Pos                    (6U)                               
#define I2C_SR1_RXNE_Msk                    (0x1U << I2C_SR1_RXNE_Pos)         /*!< 0x00000040 */
#define I2C_SR1_RXNE                        I2C_SR1_RXNE_Msk                   /*!< Data Register not Empty (receivers) */
#define I2C_SR1_TXE_Pos                     (7U)                               
#define I2C_SR1_TXE_Msk                     (0x1U << I2C_SR1_TXE_Pos)          /*!< 0x00000080 */
#define I2C_SR1_TXE                         I2C_SR1_TXE_Msk                    /*!< Data Register Empty (transmitters) */
#define I2C_SR1_BERR_Pos                    (8U)                               
#define I2C_SR1_BERR_Msk                    (0x1U << I2C_SR1_BERR_Pos)         /*!< 0x00000100 */
#define I2C_SR1_BERR                        I2C_SR1_BERR_Msk                   /*!< Bus Error */
#define I2C_SR1_ARLO_Pos                    (9U)                               
#define I2C_SR1_ARLO_Msk                    (0x1U << I2C_SR1_ARLO_Pos)         /*!< 0x00000200 */
#define I2C_SR1_ARLO                        I2C_SR1_ARLO_Msk                   /*!< Arbitration Lost (master mode) */
#define I2C_SR1_AF_Pos                      (10U)                              
#define I2C_SR1_AF_Msk                      (0x1U << I2C_SR1_AF_Pos)           /*!< 0x00000400 */
#define I2C_SR1_AF                          I2C_SR1_AF_Msk                     /*!< Acknowledge Failure */
#define I2C_SR1_OVR_Pos                     (11U)                              
#define I2C_SR1_OVR_Msk                     (0x1U << I2C_SR1_OVR_Pos)          /*!< 0x00000800 */
#define I2C_SR1_OVR                         I2C_SR1_OVR_Msk                    /*!< Overrun/Underrun */
#define I2C_SR1_PECERR_Pos                  (12U)                              
#define I2C_SR1_PECERR_Msk                  (0x1U << I2C_SR1_PECERR_Pos)       /*!< 0x00001000 */
#define I2C_SR1_PECERR                      I2C_SR1_PECERR_Msk                 /*!< PEC Error in reception */
#define I2C_SR1_TIMEOUT_Pos                 (14U)                              
#define I2C_SR1_TIMEOUT_Msk                 (0x1U << I2C_SR1_TIMEOUT_Pos)      /*!< 0x00004000 */
#define I2C_SR1_TIMEOUT                     I2C_SR1_TIMEOUT_Msk                /*!< Timeout or Tlow Error */
#define I2C_SR1_SMBALERT_Pos                (15U)                              
#define I2C_SR1_SMBALERT_Msk                (0x1U << I2C_SR1_SMBALERT_Pos)     /*!< 0x00008000 */
#define I2C_SR1_SMBALERT                    I2C_SR1_SMBALERT_Msk               /*!< SMBus Alert */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define I2C_SR2_MSL_Pos                     (0U)                               
#define I2C_SR2_MSL_Msk                     (0x1U << I2C_SR2_MSL_Pos)          /*!< 0x00000001 */
#define I2C_SR2_MSL                         I2C_SR2_MSL_Msk                    /*!< Master/Slave */
#define I2C_SR2_BUSY_Pos                    (1U)                               
#define I2C_SR2_BUSY_Msk                    (0x1U << I2C_SR2_BUSY_Pos)         /*!< 0x00000002 */
#define I2C_SR2_BUSY                        I2C_SR2_BUSY_Msk                   /*!< Bus Busy */
#define I2C_SR2_TRA_Pos                     (2U)                               
#define I2C_SR2_TRA_Msk                     (0x1U << I2C_SR2_TRA_Pos)          /*!< 0x00000004 */
#define I2C_SR2_TRA                         I2C_SR2_TRA_Msk                    /*!< Transmitter/Receiver */
#define I2C_SR2_GENCALL_Pos                 (4U)                               
#define I2C_SR2_GENCALL_Msk                 (0x1U << I2C_SR2_GENCALL_Pos)      /*!< 0x00000010 */
#define I2C_SR2_GENCALL                     I2C_SR2_GENCALL_Msk                /*!< General Call Address (Slave mode) */
#define I2C_SR2_SMBDEFAULT_Pos              (5U)                               
#define I2C_SR2_SMBDEFAULT_Msk              (0x1U << I2C_SR2_SMBDEFAULT_Pos)   /*!< 0x00000020 */
#define I2C_SR2_SMBDEFAULT                  I2C_SR2_SMBDEFAULT_Msk             /*!< SMBus Device Default Address (Slave mode) */
#define I2C_SR2_SMBHOST_Pos                 (6U)                               
#define I2C_SR2_SMBHOST_Msk                 (0x1U << I2C_SR2_SMBHOST_Pos)      /*!< 0x00000040 */
#define I2C_SR2_SMBHOST                     I2C_SR2_SMBHOST_Msk                /*!< SMBus Host Header (Slave mode) */
#define I2C_SR2_DUALF_Pos                   (7U)                               
#define I2C_SR2_DUALF_Msk                   (0x1U << I2C_SR2_DUALF_Pos)        /*!< 0x00000080 */
#define I2C_SR2_DUALF                       I2C_SR2_DUALF_Msk                  /*!< Dual Flag (Slave mode) */
#define I2C_SR2_PEC_Pos                     (8U)                               
#define I2C_SR2_PEC_Msk                     (0xFFU << I2C_SR2_PEC_Pos)         /*!< 0x0000FF00 */
#define I2C_SR2_PEC                         I2C_SR2_PEC_Msk                    /*!< Packet Error Checking Register */

/*******************  Bit definition for I2C_CCR register  ********************/
#define I2C_CCR_CCR_Pos                     (0U)                               
#define I2C_CCR_CCR_Msk                     (0xFFFU << I2C_CCR_CCR_Pos)        /*!< 0x00000FFF */
#define I2C_CCR_CCR                         I2C_CCR_CCR_Msk                    /*!< Clock Control Register in Fast/Standard mode (Master mode) */
#define I2C_CCR_DUTY_Pos                    (14U)                              
#define I2C_CCR_DUTY_Msk                    (0x1U << I2C_CCR_DUTY_Pos)         /*!< 0x00004000 */
#define I2C_CCR_DUTY                        I2C_CCR_DUTY_Msk                   /*!< Fast Mode Duty Cycle */
#define I2C_CCR_FS_Pos                      (15U)                              
#define I2C_CCR_FS_Msk                      (0x1U << I2C_CCR_FS_Pos)           /*!< 0x00008000 */
#define I2C_CCR_FS                          I2C_CCR_FS_Msk                     /*!< I2C Master Mode Selection */

/******************  Bit definition for I2C_TRISE register  *******************/
#define I2C_TRISE_TRISE_Pos                 (0U)                               
#define I2C_TRISE_TRISE_Msk                 (0x3FU << I2C_TRISE_TRISE_Pos)     /*!< 0x0000003F */
#define I2C_TRISE_TRISE                     I2C_TRISE_TRISE_Msk                /*!< Maximum Rise Time in Fast/Standard mode (Master mode) */

/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for USART_STS register  *******************/
#define  USART_STS_PERR                     ((uint16_t)0x0001)            /*!< Parity Error */
#define  USART_STS_FERR                     ((uint16_t)0x0002)            /*!< Framing Error */
#define  USART_STS_NERR                     ((uint16_t)0x0004)            /*!< Noise Error Flag */
#define  USART_STS_ORERR                    ((uint16_t)0x0008)            /*!< OverRun Error */
#define  USART_STS_IDLEF                    ((uint16_t)0x0010)            /*!< IDLE line detected */
#define  USART_STS_RDNE                     ((uint16_t)0x0020)            /*!< Read Data Register Not Empty */
#define  USART_STS_TRAC                     ((uint16_t)0x0040)            /*!< Transmission Complete */
#define  USART_STS_TDE                      ((uint16_t)0x0080)            /*!< Transmit Data Register Empty */
#define  USART_STS_LBDF                     ((uint16_t)0x0100)            /*!< LIN Break Detection Flag */
#define  USART_STS_CTSF                     ((uint16_t)0x0200)            /*!< CTS Flag */

/*******************  Bit definition for USART_DT register  *******************/
#define  USART_DT_DT                        ((uint16_t)0x01FF)            /*!< Data value */

/******************  Bit definition for USART_BAUDR register  *******************/
#define  USART_BAUDR_DIV_Decimal            ((uint16_t)0x000F)            /*!< Fraction of USARTDIV */
#define  USART_BAUDR_DIV_Integer            ((uint16_t)0xFFF0)            /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_CTRL1 register  *******************/
#define  USART_CTRL1_SBRK                   ((uint16_t)0x0001)            /*!< Send Break */
#define  USART_CTRL1_RECMUTE                ((uint16_t)0x0002)            /*!< Receiver wakeup */
#define  USART_CTRL1_REN                    ((uint16_t)0x0004)            /*!< Receiver Enable */
#define  USART_CTRL1_TEN                    ((uint16_t)0x0008)            /*!< Transmitter Enable */
#define  USART_CTRL1_IDLEIEN                ((uint16_t)0x0010)            /*!< IDLE Interrupt Enable */
#define  USART_CTRL1_RDNEIEN                ((uint16_t)0x0020)            /*!< RXNE Interrupt Enable */
#define  USART_CTRL1_TRACIEN                ((uint16_t)0x0040)            /*!< Transmission Complete Interrupt Enable */
#define  USART_CTRL1_TDEIEN                 ((uint16_t)0x0080)            /*!< PE Interrupt Enable */
#define  USART_CTRL1_PERRIEN                ((uint16_t)0x0100)            /*!< PE Interrupt Enable */
#define  USART_CTRL1_PSEL                   ((uint16_t)0x0200)            /*!< Parity Selection */
#define  USART_CTRL1_PCEN                   ((uint16_t)0x0400)            /*!< Parity Control Enable */
#define  USART_CTRL1_WUMODE                 ((uint16_t)0x0800)            /*!< Wakeup method */
#define  USART_CTRL1_LEN                    ((uint16_t)0x1000)            /*!< Word length */
#define  USART_CTRL1_UEN                    ((uint16_t)0x2000)            /*!< USART Enable */
#define  USART_CTRL1_OVER8                  ((uint16_t)0x8000)            /*!< USART Oversmapling 8-bits */

/******************  Bit definition for USART_CTRL2 register  *******************/
#define  USART_CTRL2_ADDR                   ((uint16_t)0x000F)            /*!< Address of the USART node */
#define  USART_CTRL2_LBDLEN                 ((uint16_t)0x0020)            /*!< LIN Break Detection Length */
#define  USART_CTRL2_LBDIEN                 ((uint16_t)0x0040)            /*!< LIN Break Detection Interrupt Enable */
#define  USART_CTRL2_LBCP                   ((uint16_t)0x0100)            /*!< Last Bit Clock pulse */
#define  USART_CTRL2_CLKPHA                 ((uint16_t)0x0200)            /*!< Clock Phase */
#define  USART_CTRL2_CLKPOL                 ((uint16_t)0x0400)            /*!< Clock Polarity */
#define  USART_CTRL2_CLKEN                  ((uint16_t)0x0800)            /*!< Clock Enable */

#define  USART_CTRL2_STOPB                  ((uint16_t)0x3000)            /*!< STOP[1:0] bits (STOP bits) */
#define  USART_CTRL2_STOP_B0                ((uint16_t)0x1000)            /*!< Bit 0 */
#define  USART_CTRL2_STOP_B1                ((uint16_t)0x2000)            /*!< Bit 1 */

#define  USART_CTRL2_LINEN                  ((uint16_t)0x4000)            /*!< LIN mode enable */

/******************  Bit definition for USART_CTRL3 register  *******************/
#define  USART_CTRL3_ERRIEN                 ((uint16_t)0x0001)            /*!< Error Interrupt Enable */
#define  USART_CTRL3_IRDAEN                 ((uint16_t)0x0002)            /*!< IrDA mode Enable */
#define  USART_CTRL3_IRDALP                 ((uint16_t)0x0004)            /*!< IrDA Low-Power */
#define  USART_CTRL3_HALFSEL                ((uint16_t)0x0008)            /*!< Half-Duplex Selection */
#define  USART_CTRL3_NACKEN                 ((uint16_t)0x0010)            /*!< Smartcard NACK enable */
#define  USART_CTRL3_SCMEN                  ((uint16_t)0x0020)            /*!< Smartcard mode enable */
#define  USART_CTRL3_DMAREN                 ((uint16_t)0x0040)            /*!< DMA Enable Receiver */
#define  USART_CTRL3_DMATEN                 ((uint16_t)0x0080)            /*!< DMA Enable Transmitter */
#define  USART_CTRL3_RTSEN                  ((uint16_t)0x0100)            /*!< RTS Enable */
#define  USART_CTRL3_CTSEN                  ((uint16_t)0x0200)            /*!< CTS Enable */
#define  USART_CTRL3_CTSIEN                 ((uint16_t)0x0400)            /*!< CTS Interrupt Enable */
#define  USART_CTRL3_ONEBIT                 ((uint16_t)0x0800)            /*!< One Bit method */

/******************  Bit definition for USART_GTP register  ******************/
#define  USART_GTP_PSC                      ((uint16_t)0x00FF)            /*!< PSC[7:0] bits (Prescaler value) */
#define  USART_GTP_PSC_B0                   ((uint16_t)0x0001)            /*!< Bit 0 */
#define  USART_GTP_PSC_B1                   ((uint16_t)0x0002)            /*!< Bit 1 */
#define  USART_GTP_PSC_B2                   ((uint16_t)0x0004)            /*!< Bit 2 */
#define  USART_GTP_PSC_B3                   ((uint16_t)0x0008)            /*!< Bit 3 */
#define  USART_GTP_PSC_B4                   ((uint16_t)0x0010)            /*!< Bit 4 */
#define  USART_GTP_PSC_B5                   ((uint16_t)0x0020)            /*!< Bit 5 */
#define  USART_GTP_PSC_B6                   ((uint16_t)0x0040)            /*!< Bit 6 */
#define  USART_GTP_PSC_B7                   ((uint16_t)0x0080)            /*!< Bit 7 */

#define  USART_GTP_GTVAL                    ((uint16_t)0xFF00)            /*!< Guard time value */

////////////////////////////////////////////////////////////////////////////////
// copied from stm file

/*******************  Bit definition for USART_SR register  *******************/
#define USART_SR_PE_Pos                     (0U)                               
#define USART_SR_PE_Msk                     (0x1U << USART_SR_PE_Pos)          /*!< 0x00000001 */
#define USART_SR_PE                         USART_SR_PE_Msk                    /*!< Parity Error */
#define USART_SR_FE_Pos                     (1U)                               
#define USART_SR_FE_Msk                     (0x1U << USART_SR_FE_Pos)          /*!< 0x00000002 */
#define USART_SR_FE                         USART_SR_FE_Msk                    /*!< Framing Error */
#define USART_SR_NE_Pos                     (2U)                               
#define USART_SR_NE_Msk                     (0x1U << USART_SR_NE_Pos)          /*!< 0x00000004 */
#define USART_SR_NE                         USART_SR_NE_Msk                    /*!< Noise Error Flag */
#define USART_SR_ORE_Pos                    (3U)                               
#define USART_SR_ORE_Msk                    (0x1U << USART_SR_ORE_Pos)         /*!< 0x00000008 */
#define USART_SR_ORE                        USART_SR_ORE_Msk                   /*!< OverRun Error */
#define USART_SR_IDLE_Pos                   (4U)                               
#define USART_SR_IDLE_Msk                   (0x1U << USART_SR_IDLE_Pos)        /*!< 0x00000010 */
#define USART_SR_IDLE                       USART_SR_IDLE_Msk                  /*!< IDLE line detected */
#define USART_SR_RXNE_Pos                   (5U)                               
#define USART_SR_RXNE_Msk                   (0x1U << USART_SR_RXNE_Pos)        /*!< 0x00000020 */
#define USART_SR_RXNE                       USART_SR_RXNE_Msk                  /*!< Read Data Register Not Empty */
#define USART_SR_TC_Pos                     (6U)                               
#define USART_SR_TC_Msk                     (0x1U << USART_SR_TC_Pos)          /*!< 0x00000040 */
#define USART_SR_TC                         USART_SR_TC_Msk                    /*!< Transmission Complete */
#define USART_SR_TXE_Pos                    (7U)                               
#define USART_SR_TXE_Msk                    (0x1U << USART_SR_TXE_Pos)         /*!< 0x00000080 */
#define USART_SR_TXE                        USART_SR_TXE_Msk                   /*!< Transmit Data Register Empty */
#define USART_SR_LBD_Pos                    (8U)                               
#define USART_SR_LBD_Msk                    (0x1U << USART_SR_LBD_Pos)         /*!< 0x00000100 */
#define USART_SR_LBD                        USART_SR_LBD_Msk                   /*!< LIN Break Detection Flag */
#define USART_SR_CTS_Pos                    (9U)                               
#define USART_SR_CTS_Msk                    (0x1U << USART_SR_CTS_Pos)         /*!< 0x00000200 */
#define USART_SR_CTS                        USART_SR_CTS_Msk                   /*!< CTS Flag */

/*******************  Bit definition for USART_DR register  *******************/
#define USART_DR_DR_Pos                     (0U)                               
#define USART_DR_DR_Msk                     (0x1FFU << USART_DR_DR_Pos)        /*!< 0x000001FF */
#define USART_DR_DR                         USART_DR_DR_Msk                    /*!< Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_Fraction_Pos          (0U)                               
#define USART_BRR_DIV_Fraction_Msk          (0xFU << USART_BRR_DIV_Fraction_Pos) /*!< 0x0000000F */
#define USART_BRR_DIV_Fraction              USART_BRR_DIV_Fraction_Msk         /*!< Fraction of USARTDIV */
#define USART_BRR_DIV_Mantissa_Pos          (4U)                               
#define USART_BRR_DIV_Mantissa_Msk          (0xFFFU << USART_BRR_DIV_Mantissa_Pos) /*!< 0x0000FFF0 */
#define USART_BRR_DIV_Mantissa              USART_BRR_DIV_Mantissa_Msk         /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_SBK_Pos                   (0U)                               
#define USART_CR1_SBK_Msk                   (0x1U << USART_CR1_SBK_Pos)        /*!< 0x00000001 */
#define USART_CR1_SBK                       USART_CR1_SBK_Msk                  /*!< Send Break */
#define USART_CR1_RWU_Pos                   (1U)                               
#define USART_CR1_RWU_Msk                   (0x1U << USART_CR1_RWU_Pos)        /*!< 0x00000002 */
#define USART_CR1_RWU                       USART_CR1_RWU_Msk                  /*!< Receiver wakeup */
#define USART_CR1_RE_Pos                    (2U)                               
#define USART_CR1_RE_Msk                    (0x1U << USART_CR1_RE_Pos)         /*!< 0x00000004 */
#define USART_CR1_RE                        USART_CR1_RE_Msk                   /*!< Receiver Enable */
#define USART_CR1_TE_Pos                    (3U)                               
#define USART_CR1_TE_Msk                    (0x1U << USART_CR1_TE_Pos)         /*!< 0x00000008 */
#define USART_CR1_TE                        USART_CR1_TE_Msk                   /*!< Transmitter Enable */
#define USART_CR1_IDLEIE_Pos                (4U)                               
#define USART_CR1_IDLEIE_Msk                (0x1U << USART_CR1_IDLEIE_Pos)     /*!< 0x00000010 */
#define USART_CR1_IDLEIE                    USART_CR1_IDLEIE_Msk               /*!< IDLE Interrupt Enable */
#define USART_CR1_RXNEIE_Pos                (5U)                               
#define USART_CR1_RXNEIE_Msk                (0x1U << USART_CR1_RXNEIE_Pos)     /*!< 0x00000020 */
#define USART_CR1_RXNEIE                    USART_CR1_RXNEIE_Msk               /*!< RXNE Interrupt Enable */
#define USART_CR1_TCIE_Pos                  (6U)                               
#define USART_CR1_TCIE_Msk                  (0x1U << USART_CR1_TCIE_Pos)       /*!< 0x00000040 */
#define USART_CR1_TCIE                      USART_CR1_TCIE_Msk                 /*!< Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE_Pos                 (7U)                               
#define USART_CR1_TXEIE_Msk                 (0x1U << USART_CR1_TXEIE_Pos)      /*!< 0x00000080 */
#define USART_CR1_TXEIE                     USART_CR1_TXEIE_Msk                /*!< PE Interrupt Enable */
#define USART_CR1_PEIE_Pos                  (8U)                               
#define USART_CR1_PEIE_Msk                  (0x1U << USART_CR1_PEIE_Pos)       /*!< 0x00000100 */
#define USART_CR1_PEIE                      USART_CR1_PEIE_Msk                 /*!< PE Interrupt Enable */
#define USART_CR1_PS_Pos                    (9U)                               
#define USART_CR1_PS_Msk                    (0x1U << USART_CR1_PS_Pos)         /*!< 0x00000200 */
#define USART_CR1_PS                        USART_CR1_PS_Msk                   /*!< Parity Selection */
#define USART_CR1_PCE_Pos                   (10U)                              
#define USART_CR1_PCE_Msk                   (0x1U << USART_CR1_PCE_Pos)        /*!< 0x00000400 */
#define USART_CR1_PCE                       USART_CR1_PCE_Msk                  /*!< Parity Control Enable */
#define USART_CR1_WAKE_Pos                  (11U)                              
#define USART_CR1_WAKE_Msk                  (0x1U << USART_CR1_WAKE_Pos)       /*!< 0x00000800 */
#define USART_CR1_WAKE                      USART_CR1_WAKE_Msk                 /*!< Wakeup method */
#define USART_CR1_M_Pos                     (12U)                              
#define USART_CR1_M_Msk                     (0x1U << USART_CR1_M_Pos)          /*!< 0x00001000 */
#define USART_CR1_M                         USART_CR1_M_Msk                    /*!< Word length */
#define USART_CR1_UE_Pos                    (13U)                              
#define USART_CR1_UE_Msk                    (0x1U << USART_CR1_UE_Pos)         /*!< 0x00002000 */
#define USART_CR1_UE                        USART_CR1_UE_Msk                   /*!< USART Enable */

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADD_Pos                   (0U)                               
#define USART_CR2_ADD_Msk                   (0xFU << USART_CR2_ADD_Pos)        /*!< 0x0000000F */
#define USART_CR2_ADD                       USART_CR2_ADD_Msk                  /*!< Address of the USART node */
#define USART_CR2_LBDL_Pos                  (5U)                               
#define USART_CR2_LBDL_Msk                  (0x1U << USART_CR2_LBDL_Pos)       /*!< 0x00000020 */
#define USART_CR2_LBDL                      USART_CR2_LBDL_Msk                 /*!< LIN Break Detection Length */
#define USART_CR2_LBDIE_Pos                 (6U)                               
#define USART_CR2_LBDIE_Msk                 (0x1U << USART_CR2_LBDIE_Pos)      /*!< 0x00000040 */
#define USART_CR2_LBDIE                     USART_CR2_LBDIE_Msk                /*!< LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL_Pos                  (8U)                               
#define USART_CR2_LBCL_Msk                  (0x1U << USART_CR2_LBCL_Pos)       /*!< 0x00000100 */
#define USART_CR2_LBCL                      USART_CR2_LBCL_Msk                 /*!< Last Bit Clock pulse */
#define USART_CR2_CPHA_Pos                  (9U)                               
#define USART_CR2_CPHA_Msk                  (0x1U << USART_CR2_CPHA_Pos)       /*!< 0x00000200 */
#define USART_CR2_CPHA                      USART_CR2_CPHA_Msk                 /*!< Clock Phase */
#define USART_CR2_CPOL_Pos                  (10U)                              
#define USART_CR2_CPOL_Msk                  (0x1U << USART_CR2_CPOL_Pos)       /*!< 0x00000400 */
#define USART_CR2_CPOL                      USART_CR2_CPOL_Msk                 /*!< Clock Polarity */
#define USART_CR2_CLKEN_Pos                 (11U)                              
#define USART_CR2_CLKEN_Msk                 (0x1U << USART_CR2_CLKEN_Pos)      /*!< 0x00000800 */
#define USART_CR2_CLKEN                     USART_CR2_CLKEN_Msk                /*!< Clock Enable */

#define USART_CR2_STOP_Pos                  (12U)                              
#define USART_CR2_STOP_Msk                  (0x3U << USART_CR2_STOP_Pos)       /*!< 0x00003000 */
#define USART_CR2_STOP                      USART_CR2_STOP_Msk                 /*!< STOP[1:0] bits (STOP bits) */
#define USART_CR2_STOP_0                    (0x1U << USART_CR2_STOP_Pos)       /*!< 0x00001000 */
#define USART_CR2_STOP_1                    (0x2U << USART_CR2_STOP_Pos)       /*!< 0x00002000 */

#define USART_CR2_LINEN_Pos                 (14U)                              
#define USART_CR2_LINEN_Msk                 (0x1U << USART_CR2_LINEN_Pos)      /*!< 0x00004000 */
#define USART_CR2_LINEN                     USART_CR2_LINEN_Msk                /*!< LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE_Pos                   (0U)                               
#define USART_CR3_EIE_Msk                   (0x1U << USART_CR3_EIE_Pos)        /*!< 0x00000001 */
#define USART_CR3_EIE                       USART_CR3_EIE_Msk                  /*!< Error Interrupt Enable */
#define USART_CR3_IREN_Pos                  (1U)                               
#define USART_CR3_IREN_Msk                  (0x1U << USART_CR3_IREN_Pos)       /*!< 0x00000002 */
#define USART_CR3_IREN                      USART_CR3_IREN_Msk                 /*!< IrDA mode Enable */
#define USART_CR3_IRLP_Pos                  (2U)                               
#define USART_CR3_IRLP_Msk                  (0x1U << USART_CR3_IRLP_Pos)       /*!< 0x00000004 */
#define USART_CR3_IRLP                      USART_CR3_IRLP_Msk                 /*!< IrDA Low-Power */
#define USART_CR3_HDSEL_Pos                 (3U)                               
#define USART_CR3_HDSEL_Msk                 (0x1U << USART_CR3_HDSEL_Pos)      /*!< 0x00000008 */
#define USART_CR3_HDSEL                     USART_CR3_HDSEL_Msk                /*!< Half-Duplex Selection */
#define USART_CR3_NACK_Pos                  (4U)                               
#define USART_CR3_NACK_Msk                  (0x1U << USART_CR3_NACK_Pos)       /*!< 0x00000010 */
#define USART_CR3_NACK                      USART_CR3_NACK_Msk                 /*!< Smartcard NACK enable */
#define USART_CR3_SCEN_Pos                  (5U)                               
#define USART_CR3_SCEN_Msk                  (0x1U << USART_CR3_SCEN_Pos)       /*!< 0x00000020 */
#define USART_CR3_SCEN                      USART_CR3_SCEN_Msk                 /*!< Smartcard mode enable */
#define USART_CR3_DMAR_Pos                  (6U)                               
#define USART_CR3_DMAR_Msk                  (0x1U << USART_CR3_DMAR_Pos)       /*!< 0x00000040 */
#define USART_CR3_DMAR                      USART_CR3_DMAR_Msk                 /*!< DMA Enable Receiver */
#define USART_CR3_DMAT_Pos                  (7U)                               
#define USART_CR3_DMAT_Msk                  (0x1U << USART_CR3_DMAT_Pos)       /*!< 0x00000080 */
#define USART_CR3_DMAT                      USART_CR3_DMAT_Msk                 /*!< DMA Enable Transmitter */
#define USART_CR3_RTSE_Pos                  (8U)                               
#define USART_CR3_RTSE_Msk                  (0x1U << USART_CR3_RTSE_Pos)       /*!< 0x00000100 */
#define USART_CR3_RTSE                      USART_CR3_RTSE_Msk                 /*!< RTS Enable */
#define USART_CR3_CTSE_Pos                  (9U)                               
#define USART_CR3_CTSE_Msk                  (0x1U << USART_CR3_CTSE_Pos)       /*!< 0x00000200 */
#define USART_CR3_CTSE                      USART_CR3_CTSE_Msk                 /*!< CTS Enable */
#define USART_CR3_CTSIE_Pos                 (10U)                              
#define USART_CR3_CTSIE_Msk                 (0x1U << USART_CR3_CTSIE_Pos)      /*!< 0x00000400 */
#define USART_CR3_CTSIE                     USART_CR3_CTSIE_Msk                /*!< CTS Interrupt Enable */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC_Pos                  (0U)                               
#define USART_GTPR_PSC_Msk                  (0xFFU << USART_GTPR_PSC_Pos)      /*!< 0x000000FF */
#define USART_GTPR_PSC                      USART_GTPR_PSC_Msk                 /*!< PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_PSC_0                    (0x01U << USART_GTPR_PSC_Pos)      /*!< 0x00000001 */
#define USART_GTPR_PSC_1                    (0x02U << USART_GTPR_PSC_Pos)      /*!< 0x00000002 */
#define USART_GTPR_PSC_2                    (0x04U << USART_GTPR_PSC_Pos)      /*!< 0x00000004 */
#define USART_GTPR_PSC_3                    (0x08U << USART_GTPR_PSC_Pos)      /*!< 0x00000008 */
#define USART_GTPR_PSC_4                    (0x10U << USART_GTPR_PSC_Pos)      /*!< 0x00000010 */
#define USART_GTPR_PSC_5                    (0x20U << USART_GTPR_PSC_Pos)      /*!< 0x00000020 */
#define USART_GTPR_PSC_6                    (0x40U << USART_GTPR_PSC_Pos)      /*!< 0x00000040 */
#define USART_GTPR_PSC_7                    (0x80U << USART_GTPR_PSC_Pos)      /*!< 0x00000080 */

#define USART_GTPR_GT_Pos                   (8U)                               
#define USART_GTPR_GT_Msk                   (0xFFU << USART_GTPR_GT_Pos)       /*!< 0x0000FF00 */
#define USART_GTPR_GT                       USART_GTPR_GT_Msk                  /*!< Guard time value */

/******************************************************************************/
/*                                                                            */
/*                                 Debug MCU                                  */
/*                                                                            */
/******************************************************************************/

/****************  Bit definition for DBGMCU_IDCR register  *****************/
#define  MCUDBG_IDCR_DEV_ID                 ((uint32_t)0x00000FFF)        /*!< Device Identifier */

#define  MCUDBG_IDCR_REV_ID                 ((uint32_t)0xFFFF0000)        /*!< REV_ID[15:0] bits (Revision Identifier) */
#define  MCUDBG_IDCR_REV_ID_0               ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  MCUDBG_IDCR_REV_ID_1               ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  MCUDBG_IDCR_REV_ID_2               ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  MCUDBG_IDCR_REV_ID_3               ((uint32_t)0x00080000)        /*!< Bit 3 */
#define  MCUDBG_IDCR_REV_ID_4               ((uint32_t)0x00100000)        /*!< Bit 4 */
#define  MCUDBG_IDCR_REV_ID_5               ((uint32_t)0x00200000)        /*!< Bit 5 */
#define  MCUDBG_IDCR_REV_ID_6               ((uint32_t)0x00400000)        /*!< Bit 6 */
#define  MCUDBG_IDCR_REV_ID_7               ((uint32_t)0x00800000)        /*!< Bit 7 */
#define  MCUDBG_IDCR_REV_ID_8               ((uint32_t)0x01000000)        /*!< Bit 8 */
#define  MCUDBG_IDCR_REV_ID_9               ((uint32_t)0x02000000)        /*!< Bit 9 */
#define  MCUDBG_IDCR_REV_ID_10              ((uint32_t)0x04000000)        /*!< Bit 10 */
#define  MCUDBG_IDCR_REV_ID_11              ((uint32_t)0x08000000)        /*!< Bit 11 */
#define  MCUDBG_IDCR_REV_ID_12              ((uint32_t)0x10000000)        /*!< Bit 12 */
#define  MCUDBG_IDCR_REV_ID_13              ((uint32_t)0x20000000)        /*!< Bit 13 */
#define  MCUDBG_IDCR_REV_ID_14              ((uint32_t)0x40000000)        /*!< Bit 14 */
#define  MCUDBG_IDCR_REV_ID_15              ((uint32_t)0x80000000)        /*!< Bit 15 */

/******************  Bit definition for DBGMCU_CTRL register  *******************/
#define  MCUDBG_CTRL_DBG_SLEEP                  ((uint32_t)0x00000001)        /*!< Debug Sleep Mode */
#define  MCUDBG_CTRL_DBG_STOP                   ((uint32_t)0x00000002)        /*!< Debug Stop Mode */
#define  MCUDBG_CTRL_DBG_STANDBY                ((uint32_t)0x00000004)        /*!< Debug Standby mode */
#define  MCUDBG_CTRL_TRACE_IOEN                 ((uint32_t)0x00000020)        /*!< Trace Pin Assignment Control */

#define  MCUDBG_CTRL_TRACE_MODE                 ((uint32_t)0x000000C0)        /*!< TRACE_MODE[1:0] bits (Trace Pin Assignment Control) */
#define  MCUDBG_CTRL_TRACE_MODE_0               ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  MCUDBG_CTRL_TRACE_MODE_1               ((uint32_t)0x00000080)        /*!< Bit 1 */

#define  MCUDBG_CTRL_DBG_IWDG_STOP              ((uint32_t)0x00000100)        /*!< Debug Independent Watchdog stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_WWDG_STOP              ((uint32_t)0x00000200)        /*!< Debug Window Watchdog stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_TMR1_STOP              ((uint32_t)0x00000400)        /*!< TMR1 counter stopped when core is halted */
#define  MCUDBG_CTRL_DBG_TMR2_STOP              ((uint32_t)0x00000800)        /*!< TMR2 counter stopped when core is halted */
#define  MCUDBG_CTRL_DBG_TMR3_STOP              ((uint32_t)0x00001000)        /*!< TMR3 counter stopped when core is halted */
#define  MCUDBG_CTRL_DBG_TMR4_STOP              ((uint32_t)0x00002000)        /*!< TMR4 counter stopped when core is halted */
#define  MCUDBG_CTRL_DBG_CAN1_STOP              ((uint32_t)0x00004000)        /*!< Debug CAN1 stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_I2C1_SMBUS_TIMEOUT     ((uint32_t)0x00008000)        /*!< SMBUS timeout mode stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_I2C2_SMBUS_TIMEOUT     ((uint32_t)0x00010000)        /*!< SMBUS timeout mode stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_TMR8_STOP              ((uint32_t)0x00020000)        /*!< TMR8 counter stopped when core is halted */
#define  MCUDBG_CTRL_DBG_TMR5_STOP              ((uint32_t)0x00040000)        /*!< TMR5 counter stopped when core is halted */
#define  MCUDBG_CTRL_DBG_TMR6_STOP              ((uint32_t)0x00080000)        /*!< TMR6 counter stopped when core is halted */
#define  MCUDBG_CTRL_DBG_TMR7_STOP              ((uint32_t)0x00100000)        /*!< TMR7 counter stopped when core is halted */
#define  MCUDBG_CTRL_DBG_TMR15_STOP             ((uint32_t)0x00400000)        /*!< Debug TMR15 stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_TMR12_STOP             ((uint32_t)0x02000000)        /*!< Debug TMR12 stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_TMR13_STOP             ((uint32_t)0x04000000)        /*!< Debug TMR13 stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_TMR14_STOP             ((uint32_t)0x08000000)        /*!< Debug TMR14 stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_TMR9_STOP              ((uint32_t)0x10000000)        /*!< Debug TMR9 stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_TMR10_STOP             ((uint32_t)0x20000000)        /*!< Debug TMR10 stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_TMR11_STOP             ((uint32_t)0x40000000)        /*!< Debug TMR11 stopped when Core is halted */
#define  MCUDBG_CTRL_DBG_I2C3_SMBUS_TIMEOUT     ((uint32_t)0x80000000)        /*!< SMBUS timeout mode stopped when Core is halted */


////////////////////////////////////////////////////////////////////////////////
// Copied from stm file

/****************  Bit definition for DBGMCU_IDCODE register  *****************/
#define DBGMCU_IDCODE_DEV_ID_Pos            (0U)                               
#define DBGMCU_IDCODE_DEV_ID_Msk            (0xFFFU << DBGMCU_IDCODE_DEV_ID_Pos) /*!< 0x00000FFF */
#define DBGMCU_IDCODE_DEV_ID                DBGMCU_IDCODE_DEV_ID_Msk           /*!< Device Identifier */

#define DBGMCU_IDCODE_REV_ID_Pos            (16U)                              
#define DBGMCU_IDCODE_REV_ID_Msk            (0xFFFFU << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0xFFFF0000 */
#define DBGMCU_IDCODE_REV_ID                DBGMCU_IDCODE_REV_ID_Msk           /*!< REV_ID[15:0] bits (Revision Identifier) */
#define DBGMCU_IDCODE_REV_ID_0              (0x0001U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00010000 */
#define DBGMCU_IDCODE_REV_ID_1              (0x0002U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00020000 */
#define DBGMCU_IDCODE_REV_ID_2              (0x0004U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00040000 */
#define DBGMCU_IDCODE_REV_ID_3              (0x0008U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00080000 */
#define DBGMCU_IDCODE_REV_ID_4              (0x0010U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00100000 */
#define DBGMCU_IDCODE_REV_ID_5              (0x0020U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00200000 */
#define DBGMCU_IDCODE_REV_ID_6              (0x0040U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00400000 */
#define DBGMCU_IDCODE_REV_ID_7              (0x0080U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x00800000 */
#define DBGMCU_IDCODE_REV_ID_8              (0x0100U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x01000000 */
#define DBGMCU_IDCODE_REV_ID_9              (0x0200U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x02000000 */
#define DBGMCU_IDCODE_REV_ID_10             (0x0400U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x04000000 */
#define DBGMCU_IDCODE_REV_ID_11             (0x0800U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x08000000 */
#define DBGMCU_IDCODE_REV_ID_12             (0x1000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x10000000 */
#define DBGMCU_IDCODE_REV_ID_13             (0x2000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x20000000 */
#define DBGMCU_IDCODE_REV_ID_14             (0x4000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x40000000 */
#define DBGMCU_IDCODE_REV_ID_15             (0x8000U << DBGMCU_IDCODE_REV_ID_Pos) /*!< 0x80000000 */

/******************  Bit definition for DBGMCU_CR register  *******************/
#define DBGMCU_CR_DBG_SLEEP_Pos             (0U)                               
#define DBGMCU_CR_DBG_SLEEP_Msk             (0x1U << DBGMCU_CR_DBG_SLEEP_Pos)  /*!< 0x00000001 */
#define DBGMCU_CR_DBG_SLEEP                 DBGMCU_CR_DBG_SLEEP_Msk            /*!< Debug Sleep Mode */
#define DBGMCU_CR_DBG_STOP_Pos              (1U)                               
#define DBGMCU_CR_DBG_STOP_Msk              (0x1U << DBGMCU_CR_DBG_STOP_Pos)   /*!< 0x00000002 */
#define DBGMCU_CR_DBG_STOP                  DBGMCU_CR_DBG_STOP_Msk             /*!< Debug Stop Mode */
#define DBGMCU_CR_DBG_STANDBY_Pos           (2U)                               
#define DBGMCU_CR_DBG_STANDBY_Msk           (0x1U << DBGMCU_CR_DBG_STANDBY_Pos) /*!< 0x00000004 */
#define DBGMCU_CR_DBG_STANDBY               DBGMCU_CR_DBG_STANDBY_Msk          /*!< Debug Standby mode */
#define DBGMCU_CR_TRACE_IOEN_Pos            (5U)                               
#define DBGMCU_CR_TRACE_IOEN_Msk            (0x1U << DBGMCU_CR_TRACE_IOEN_Pos) /*!< 0x00000020 */
#define DBGMCU_CR_TRACE_IOEN                DBGMCU_CR_TRACE_IOEN_Msk           /*!< Trace Pin Assignment Control */

#define DBGMCU_CR_TRACE_MODE_Pos            (6U)                               
#define DBGMCU_CR_TRACE_MODE_Msk            (0x3U << DBGMCU_CR_TRACE_MODE_Pos) /*!< 0x000000C0 */
#define DBGMCU_CR_TRACE_MODE                DBGMCU_CR_TRACE_MODE_Msk           /*!< TRACE_MODE[1:0] bits (Trace Pin Assignment Control) */
#define DBGMCU_CR_TRACE_MODE_0              (0x1U << DBGMCU_CR_TRACE_MODE_Pos) /*!< 0x00000040 */
#define DBGMCU_CR_TRACE_MODE_1              (0x2U << DBGMCU_CR_TRACE_MODE_Pos) /*!< 0x00000080 */

#define DBGMCU_CR_DBG_IWDG_STOP_Pos         (8U)                               
#define DBGMCU_CR_DBG_IWDG_STOP_Msk         (0x1U << DBGMCU_CR_DBG_IWDG_STOP_Pos) /*!< 0x00000100 */
#define DBGMCU_CR_DBG_IWDG_STOP             DBGMCU_CR_DBG_IWDG_STOP_Msk        /*!< Debug Independent Watchdog stopped when Core is halted */
#define DBGMCU_CR_DBG_WWDG_STOP_Pos         (9U)                               
#define DBGMCU_CR_DBG_WWDG_STOP_Msk         (0x1U << DBGMCU_CR_DBG_WWDG_STOP_Pos) /*!< 0x00000200 */
#define DBGMCU_CR_DBG_WWDG_STOP             DBGMCU_CR_DBG_WWDG_STOP_Msk        /*!< Debug Window Watchdog stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM1_STOP_Pos         (10U)                              
#define DBGMCU_CR_DBG_TIM1_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM1_STOP_Pos) /*!< 0x00000400 */
#define DBGMCU_CR_DBG_TIM1_STOP             DBGMCU_CR_DBG_TIM1_STOP_Msk        /*!< TIM1 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM2_STOP_Pos         (11U)                              
#define DBGMCU_CR_DBG_TIM2_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM2_STOP_Pos) /*!< 0x00000800 */
#define DBGMCU_CR_DBG_TIM2_STOP             DBGMCU_CR_DBG_TIM2_STOP_Msk        /*!< TIM2 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM3_STOP_Pos         (12U)                              
#define DBGMCU_CR_DBG_TIM3_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM3_STOP_Pos) /*!< 0x00001000 */
#define DBGMCU_CR_DBG_TIM3_STOP             DBGMCU_CR_DBG_TIM3_STOP_Msk        /*!< TIM3 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM4_STOP_Pos         (13U)                              
#define DBGMCU_CR_DBG_TIM4_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM4_STOP_Pos) /*!< 0x00002000 */
#define DBGMCU_CR_DBG_TIM4_STOP             DBGMCU_CR_DBG_TIM4_STOP_Msk        /*!< TIM4 counter stopped when core is halted */
#define DBGMCU_CR_DBG_CAN1_STOP_Pos         (14U)                              
#define DBGMCU_CR_DBG_CAN1_STOP_Msk         (0x1U << DBGMCU_CR_DBG_CAN1_STOP_Pos) /*!< 0x00004000 */
#define DBGMCU_CR_DBG_CAN1_STOP             DBGMCU_CR_DBG_CAN1_STOP_Msk        /*!< Debug CAN1 stopped when Core is halted */
#define DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT_Pos (15U)                             
#define DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT_Msk (0x1U << DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT_Pos) /*!< 0x00008000 */
#define DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT    DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT_Msk /*!< SMBUS timeout mode stopped when Core is halted */
#define DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT_Pos (16U)                             
#define DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT_Msk (0x1U << DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT_Pos) /*!< 0x00010000 */
#define DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT    DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT_Msk /*!< SMBUS timeout mode stopped when Core is halted */
#define DBGMCU_CR_DBG_TIM8_STOP_Pos         (17U)                              
#define DBGMCU_CR_DBG_TIM8_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM8_STOP_Pos) /*!< 0x00020000 */
#define DBGMCU_CR_DBG_TIM8_STOP             DBGMCU_CR_DBG_TIM8_STOP_Msk        /*!< TIM8 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM5_STOP_Pos         (18U)                              
#define DBGMCU_CR_DBG_TIM5_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM5_STOP_Pos) /*!< 0x00040000 */
#define DBGMCU_CR_DBG_TIM5_STOP             DBGMCU_CR_DBG_TIM5_STOP_Msk        /*!< TIM5 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM6_STOP_Pos         (19U)                              
#define DBGMCU_CR_DBG_TIM6_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM6_STOP_Pos) /*!< 0x00080000 */
#define DBGMCU_CR_DBG_TIM6_STOP             DBGMCU_CR_DBG_TIM6_STOP_Msk        /*!< TIM6 counter stopped when core is halted */
#define DBGMCU_CR_DBG_TIM7_STOP_Pos         (20U)                              
#define DBGMCU_CR_DBG_TIM7_STOP_Msk         (0x1U << DBGMCU_CR_DBG_TIM7_STOP_Pos) /*!< 0x00100000 */
#define DBGMCU_CR_DBG_TIM7_STOP             DBGMCU_CR_DBG_TIM7_STOP_Msk        /*!< TIM7 counter stopped when core is halted */

/******************************************************************************/
/*                                                                            */
/*                      FLASH and Option Bytes Registers                      */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for FLASH_FCKEY register  ******************/
#define  FLASH_FCKEY_KEY                        ((uint32_t)0xFFFFFFFF)        /*!< FPEC Key */

//maybe this stuff does not even work on the AT32?
//seems to be none existent
#define FLASH_KEY1_Pos                      (0U)                               
#define FLASH_KEY1_Msk                      (0x45670123U << FLASH_KEY1_Pos)    /*!< 0x45670123 */
#define FLASH_KEY1                          FLASH_KEY1_Msk                     /*!< FPEC Key1 */
#define FLASH_KEY2_Pos                      (0U)                               
#define FLASH_KEY2_Msk                      (0xCDEF89ABU << FLASH_KEY2_Pos)    /*!< 0xCDEF89AB */
#define FLASH_KEY2                          FLASH_KEY2_Msk   

/*****************  Bit definition for FLASH_UOBKEY register  ****************/
#define  FLASH_UOBKEY_KEY                       ((uint32_t)0xFFFFFFFF)        /*!< Option Byte Key */

#define  FLASH_OPTKEY1                       FLASH_UOBKEY_KEY                    /*!< Option Byte Key1 */
#define  FLASH_OPTKEY2                       FLASH_FCKEY_KEY   

/******************  Bit definition for FLASH_STS register  *******************/
//Added aliases to match STM HAL names
#define  FLASH_STS_BSY                          ((uint8_t)0x01)               /*!< Busy */
#define  FLASH_SR_BSY   						FLASH_STS_BSY
#define  FLASH_STS_PRGMFLR                      ((uint8_t)0x04)               /*!< Programming Error */
#define  FLASH_SR_PGERR                      	FLASH_STS_PRGMFLR
#define  FLASH_STS_WRPRTFLR                     ((uint8_t)0x10)               /*!< Write Protection Error */
#define  FLASH_SR_WRPRTERR                   	FLASH_STS_WRPRTFLR
#define  FLASH_STS_PRCDN                        ((uint8_t)0x20)               /*!< End of operation */
#define  FLASH_SR_EOP                        	FLASH_STS_PRCDN

/*******************  Bit definition for FLASH_CTRL register  *******************/
#define  FLASH_CTRL_PRGM                        ((uint16_t)0x0001)            /*!< Programming */
#define  FLASH_CR_PG                         	FLASH_CTRL_PRGM
#define  FLASH_CTRL_PGERS                       ((uint16_t)0x0002)            /*!< Page Erase */
#define  FLASH_CR_PER                        	FLASH_CTRL_PGERS
#define  FLASH_CTRL_CHPERS                      ((uint16_t)0x0004)            /*!< Mass Erase */
#define  FLASH_CR_MER                        	FLASH_CTRL_CHPERS
#define  FLASH_CTRL_UOBPRGM                     ((uint16_t)0x0010)            /*!< Option Byte Programming */
#define  FLASH_CR_OPTPG                      	FLASH_CTRL_UOBPRGM
#define  FLASH_CTRL_UOBERS                      ((uint16_t)0x0020)            /*!< Option Byte Erase */
#define  FLASH_CR_OPTER                      	FLASH_CTRL_UOBERS
#define  FLASH_CTRL_STRT                        ((uint16_t)0x0040)            /*!< Start */
#define  FLASH_CR_STRT                       	FLASH_CTRL_STRT
#define  FLASH_CTRL_LCK                         ((uint16_t)0x0080)            /*!< Lock */
#define  FLASH_CR_LOCK                       	FLASH_CTRL_LCK
#define  FLASH_CTRL_UOBWE                       ((uint16_t)0x0200)            /*!< Option Bytes Write Enable */
#define  FLASH_CR_OPTWRE                     	FLASH_CTRL_UOBWE
#define  FLASH_CTRL_FLRIE                       ((uint16_t)0x0400)            /*!< Error Interrupt Enable */
#define  FLASH_CR_ERRIE							FLASH_CTRL_FLRIE
#define  FLASH_CTRL_PRCDNIE                     ((uint16_t)0x1000)            /*!< End of operation interrupt enable */
#define  FLASH_CR_EOPIE							FLASH_CTRL_PRCDNIE

/*******************  Bit definition for FLASH_ADDR register  *******************/
#define  FLASH_ADDR_TA                          ((uint32_t)0xFFFFFFFF)        /*!< Flash Address */

/******************  Bit definition for FLASH_UOB register  *******************/
#define  FLASH_UOB_UOBFLR                       ((uint16_t)0x0001)            /*!< Option Byte Error */
#define  FLASH_OBR_OPTERR                    	FLASH_UOB_UOBFLR
#define  FLASH_UOB_RDPRTEN                      ((uint16_t)0x0002)            /*!< Read protection */
#define  FLASH_OBR_RDPRT       					FLASH_UOB_RDPRTEN              

#define  FLASH_UOB_USR                          ((uint16_t)0x03FC)            /*!< User Option Bytes */
#define  FLASH_OBR_USER							FLASH_UOB_USR
#define  FLASH_UOB_SW_WDG                       ((uint16_t)0x0004)            /*!< WDG_SW */
#define FLASH_OBR_IWDG_SW_Pos               (2U)                               
#define FLASH_OBR_IWDG_SW_Msk               (0x1U << FLASH_OBR_IWDG_SW_Pos)    /*!< 0x00000004 */
#define  FLASH_OBR_IWDG_SW                   	FLASH_UOB_SW_WDG
#define  FLASH_UOB_nSTP_RST                     ((uint16_t)0x0008)            /*!< nRST_STOP */
#define  FLASH_OBR_nRST_STOP                 	FLASH_UOB_nSTP_RST
#define  FLASH_UOB_nSTDBY_RST                   ((uint16_t)0x0010)            /*!< nRST_STDBY */
#define  FLASH_OBR_nRST_STDBY                	FLASH_UOB_nSTDBY_RST
#define  FLASH_UOB_BTOPT                        ((uint16_t)0x0020)            /*!< BFB2 */         

/////////////////////////////////////////////////////////////////////////////////
//copied from STM file
// Somehow does not exists in AT32?
#define FLASH_ACR_PRFTBE_Pos                (4U)                               
#define FLASH_ACR_PRFTBE_Msk                (0x1U << FLASH_ACR_PRFTBE_Pos)     /*!< 0x00000010 */
#define FLASH_ACR_PRFTBE                    FLASH_ACR_PRFTBE_Msk               /*!< Prefetch Buffer Enable */


/******************  Bit definition for FLASH_WRPRT register  ******************/
#define  FLASH_WRPRT_WRPRTBMP                   ((uint32_t)0xFFFFFFFF)        /*!< Write Protect */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for FLASH_RDPRT register  *******************/
#define  FLASH_RDPRT_RDPRT                      ((uint32_t)0x000000FF)        /*!< Read protection option byte */
#define  FLASH_RDPRT_nRDPRT                     ((uint32_t)0x0000FF00)        /*!< Read protection complemented option byte */

/******************  Bit definition for FLASH_USR register  ******************/
#define  FLASH_USR_USR                          ((uint32_t)0x00FF0000)        /*!< User option byte */
#define  FLASH_USR_nUSR                         ((uint32_t)0xFF000000)        /*!< User complemented option byte */

/******************  Bit definition for FLASH_DT0 register  *****************/
#define  FLASH_DT0_DT0                          ((uint32_t)0x000000FF)        /*!< User data storage option byte */
#define  FLASH_DT0_nDT0                         ((uint32_t)0x0000FF00)        /*!< User data storage complemented option byte */

/******************  Bit definition for FLASH_DT1 register  *****************/
#define  FLASH_DT1_DT1                          ((uint32_t)0x00FF0000)        /*!< User data storage option byte */
#define  FLASH_DT1_nDT1                         ((uint32_t)0xFF000000)        /*!< User data storage complemented option byte */

/******************  Bit definition for FLASH_WRPRT0 register  ******************/
#define  FLASH_WRPRT0_WRPRT0                    ((uint32_t)0x000000FF)        /*!< Flash memory write protection option bytes */
#define  FLASH_WRPRT0_nWRPRT0                   ((uint32_t)0x0000FF00)        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRPRT1 register  ******************/
#define  FLASH_WRPRT1_WRPRT1                    ((uint32_t)0x00FF0000)        /*!< Flash memory write protection option bytes */
#define  FLASH_WRPRT1_nWRPRT1                   ((uint32_t)0xFF000000)        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRPRT2 register  ******************/
#define  FLASH_WRPRT2_WRPRT2                    ((uint32_t)0x000000FF)        /*!< Flash memory write protection option bytes */
#define  FLASH_WRPRT2_nWRPRT2                   ((uint32_t)0x0000FF00)        /*!< Flash memory write protection complemented option bytes */

/******************  Bit definition for FLASH_WRPRT3 register  ******************/
#define  FLASH_WRPRT3_WRPRT3                    ((uint32_t)0x00FF0000)        /*!< Flash memory write protection option bytes */
#define  FLASH_WRPRT3_nWRPRT3                   ((uint32_t)0xFF000000)        /*!< Flash memory write protection complemented option bytes */

/**
  * @}
  */

/**
 * @}
 */

#ifdef LIBRARY_VERSION
#include "at32f4xx_conf.h"
#endif

/** @addtogroup Exported_macro
  * @{
  */

#define BIT_SET(REG, BIT)                       ((REG) |= (BIT))

#define BIT_CLEAR(REG, BIT)                     ((REG) &= ~(BIT))

#define BIT_READ(REG, BIT)                      ((REG) & (BIT))

#define REG_CLEAR(REG)                          ((REG) = (0x0))

#define REG_WRITE(REG, VAL)                     ((REG) = (VAL))

#define REG_READ(REG)                           ((REG))

#define REG_CHANGE(REG, CLEARMASK, SETMASK)     REG_WRITE((REG), (((REG_READ(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL))) 


/****************************** ADC Instances *********************************/
#define IS_ADC_ALL_INSTANCE(INSTANCE) (((INSTANCE) == ADC1) || \
                                       ((INSTANCE) == ADC2) || \
                                       ((INSTANCE) == ADC3))
                                       
#define IS_ADC_MULTIMODE_MASTER_INSTANCE(INSTANCE) ((INSTANCE) == ADC1)

#define IS_ADC_COMMON_INSTANCE(INSTANCE) ((INSTANCE) == ADC12_COMMON)

#define IS_ADC_DMA_CAPABILITY_INSTANCE(INSTANCE) (((INSTANCE) == ADC1) || \
                                                  ((INSTANCE) == ADC3))

/****************************** CAN Instances *********************************/    
#define IS_CAN_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CAN1)

/****************************** CRC Instances *********************************/
#define IS_CRC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == CRC)

/****************************** DAC Instances *********************************/
#define IS_DAC_ALL_INSTANCE(INSTANCE) ((INSTANCE) == DAC1)

/****************************** DMA Instances *********************************/
#define IS_DMA_ALL_INSTANCE(INSTANCE) (((INSTANCE) == DMA1_Channel1) || \
                                       ((INSTANCE) == DMA1_Channel2) || \
                                       ((INSTANCE) == DMA1_Channel3) || \
                                       ((INSTANCE) == DMA1_Channel4) || \
                                       ((INSTANCE) == DMA1_Channel5) || \
                                       ((INSTANCE) == DMA1_Channel6) || \
                                       ((INSTANCE) == DMA1_Channel7) || \
                                       ((INSTANCE) == DMA2_Channel1) || \
                                       ((INSTANCE) == DMA2_Channel2) || \
                                       ((INSTANCE) == DMA2_Channel3) || \
                                       ((INSTANCE) == DMA2_Channel4) || \
                                       ((INSTANCE) == DMA2_Channel5))
  
/******************************* GPIO Instances *******************************/
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                        ((INSTANCE) == GPIOB) || \
                                        ((INSTANCE) == GPIOC) || \
                                        ((INSTANCE) == GPIOD) || \
                                        ((INSTANCE) == GPIOE) || \
                                        ((INSTANCE) == GPIOF) || \
                                        ((INSTANCE) == GPIOG))

/**************************** GPIO Alternate Function Instances ***************/
#define IS_GPIO_AF_INSTANCE(INSTANCE) IS_GPIO_ALL_INSTANCE(INSTANCE)

/**************************** GPIO Lock Instances *****************************/
#define IS_GPIO_LOCK_INSTANCE(INSTANCE) IS_GPIO_ALL_INSTANCE(INSTANCE)

/******************************** I2C Instances *******************************/
#define IS_I2C_ALL_INSTANCE(INSTANCE) (((INSTANCE) == I2C1) || \
                                       ((INSTANCE) == I2C2))

/******************************* SMBUS Instances ******************************/
#define IS_SMBUS_ALL_INSTANCE         IS_I2C_ALL_INSTANCE

/******************************** I2S Instances *******************************/
#define IS_I2S_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/****************************** IWDG Instances ********************************/
#define IS_IWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == IWDG)

/****************************** SDIO Instances *********************************/
#define IS_SDIO_ALL_INSTANCE(INSTANCE) ((INSTANCE) == SDIO)

/******************************** SPI Instances *******************************/
#define IS_SPI_ALL_INSTANCE(INSTANCE) (((INSTANCE) == SPI1) || \
                                       ((INSTANCE) == SPI2) || \
                                       ((INSTANCE) == SPI3))

/****************************** START TIM Instances ***************************/
/****************************** TIM Instances *********************************/
#define IS_TIM_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5)    || \
   ((INSTANCE) == TIM6)    || \
   ((INSTANCE) == TIM7))

#define IS_TIM_ADVANCED_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8))

#define IS_TIM_CC1_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_CC2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_CC3_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_CC4_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_CLOCKSOURCE_TIX_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_CLOCKSOURCE_ITRX_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_OCXREF_CLEAR_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_ENCODER_INTERFACE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_XOR_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_MASTER_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5)    || \
   ((INSTANCE) == TIM6)    || \
   ((INSTANCE) == TIM7))

#define IS_TIM_SLAVE_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_DMABURST_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_BREAK_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8))

#define IS_TIM_CCX_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                  \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM8) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM2) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM3) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM4) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4)))           \
    ||                                         \
    (((INSTANCE) == TIM5) &&                   \
     (((CHANNEL) == TIM_CHANNEL_1) ||          \
      ((CHANNEL) == TIM_CHANNEL_2) ||          \
      ((CHANNEL) == TIM_CHANNEL_3) ||          \
      ((CHANNEL) == TIM_CHANNEL_4))))

#define IS_TIM_CCXN_INSTANCE(INSTANCE, CHANNEL) \
   ((((INSTANCE) == TIM1) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3)))            \
    ||                                          \
    (((INSTANCE) == TIM8) &&                    \
     (((CHANNEL) == TIM_CHANNEL_1) ||           \
      ((CHANNEL) == TIM_CHANNEL_2) ||           \
      ((CHANNEL) == TIM_CHANNEL_3))))

#define IS_TIM_COUNTER_MODE_SELECT_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_REPETITION_COUNTER_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8))

#define IS_TIM_CLOCK_DIVISION_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))

#define IS_TIM_DMA_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5)    || \
   ((INSTANCE) == TIM6)    || \
   ((INSTANCE) == TIM7))
    
#define IS_TIM_DMA_CC_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8)    || \
   ((INSTANCE) == TIM2)    || \
   ((INSTANCE) == TIM3)    || \
   ((INSTANCE) == TIM4)    || \
   ((INSTANCE) == TIM5))
    
#define IS_TIM_COMMUTATION_EVENT_INSTANCE(INSTANCE)\
  (((INSTANCE) == TIM1)    || \
   ((INSTANCE) == TIM8))

#define IS_TIM_ETR_INSTANCE(INSTANCE)  (((INSTANCE) == TIM1)    || \
                                        ((INSTANCE) == TIM2)    || \
                                        ((INSTANCE) == TIM3)    || \
                                        ((INSTANCE) == TIM4)    || \
                                        ((INSTANCE) == TIM5)    || \
                                        ((INSTANCE) == TIM8))

#define IS_TIM_HALL_SENSOR_INTERFACE_INSTANCE(INSTANCE) (((INSTANCE) == TIM1)    || \
                                                         ((INSTANCE) == TIM2)    || \
                                                         ((INSTANCE) == TIM3)    || \
                                                         ((INSTANCE) == TIM4)    || \
                                                         ((INSTANCE) == TIM5)    || \
                                                         ((INSTANCE) == TIM8))

#define IS_TIM_32B_COUNTER_INSTANCE(INSTANCE)           0U

/****************************** END TIM Instances *****************************/


/******************** USART Instances : Synchronous mode **********************/                                           
#define IS_USART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                     ((INSTANCE) == USART2) || \
                                     ((INSTANCE) == USART3))

/******************** UART Instances : Asynchronous mode **********************/
#define IS_UART_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART3) || \
                                    ((INSTANCE) == UART4)  || \
                                    ((INSTANCE) == UART5))

/******************** UART Instances : Half-Duplex mode **********************/
#define IS_UART_HALFDUPLEX_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                               ((INSTANCE) == USART2) || \
                                               ((INSTANCE) == USART3) || \
                                               ((INSTANCE) == UART4)  || \
                                               ((INSTANCE) == UART5))

/******************** UART Instances : LIN mode **********************/
#define IS_UART_LIN_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                        ((INSTANCE) == USART2) || \
                                        ((INSTANCE) == USART3) || \
                                        ((INSTANCE) == UART4)  || \
                                        ((INSTANCE) == UART5))

/****************** UART Instances : Hardware Flow control ********************/                                    
#define IS_UART_HWFLOW_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                           ((INSTANCE) == USART2) || \
                                           ((INSTANCE) == USART3))

/********************* UART Instances : Smard card mode ***********************/
#define IS_SMARTCARD_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                         ((INSTANCE) == USART2) || \
                                         ((INSTANCE) == USART3))

/*********************** UART Instances : IRDA mode ***************************/
#define IS_IRDA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                    ((INSTANCE) == USART2) || \
                                    ((INSTANCE) == USART3) || \
                                    ((INSTANCE) == UART4)  || \
                                    ((INSTANCE) == UART5))

/***************** UART Instances : Multi-Processor mode **********************/
#define IS_UART_MULTIPROCESSOR_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                                   ((INSTANCE) == USART2) || \
                                                   ((INSTANCE) == USART3) || \
                                                   ((INSTANCE) == UART4)  || \
                                                   ((INSTANCE) == UART5))

/***************** UART Instances : DMA mode available **********************/
#define IS_UART_DMA_INSTANCE(INSTANCE) (((INSTANCE) == USART1) || \
                                        ((INSTANCE) == USART2) || \
                                        ((INSTANCE) == USART3) || \
                                        ((INSTANCE) == UART4))

/****************************** RTC Instances *********************************/
#define IS_RTC_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == RTC)

/**************************** WWDG Instances *****************************/
#define IS_WWDG_ALL_INSTANCE(INSTANCE)  ((INSTANCE) == WWDG)

/****************************** USB Instances ********************************/
#define IS_USB_ALL_INSTANCE(INSTANCE)   ((INSTANCE) == USB)

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __AT32F4xx_H */

/**
  * @}
  */

/**
* @}
*/
