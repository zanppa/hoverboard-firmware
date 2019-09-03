Phase control pins are accordign to following table. There seems to be some 
confusion between the available schematic and the current firmware, I need to 
check which notation is correct (which matches the markings on the PCB). Bold 
means the function used in the firmware.

| Function           | Pin | Pin function                                   |
|--------------------|-----|------------------------------------------------|
| Left phase A high  | 41  | PA8<br/>**TIM1_CH1**                           |
| Left phase A low   | 34  | PB13<br/>**TIM1_CH1N**                         |
| Left phase B high  | 42  | PA9<br/>**TIM1_CH2**                           |
| Left phase B low   | 35  | PB14<br/>**TIM1_CH2N**                         |
| Left phase C high  | 43  | PA10<br/>**TIM1_CH3**                          |
| Left phase C low   | 36  | PB15<br/>**TIM1_CH3N**                         |
| Right phase A high | 37  | PC6<br/>**TIM8_CH1**<br/>TIM3_CH1              |
| Right phase A low  | 23  | PA7<br/>**TIM8_CH1N** / TIM3_CH2<br/>TIM1_CH1N |
| Right phase B high | 38  | PC7<br/>**TIM8_CH2**<br/>TIM3_CH2              |
| Right phase B low  | 26  | PB0<br/>TIM3_CH3 / **TIM8_CH2N**<br/>TIM1_CH2N |
| Right phase C high | 39  | PC8<br/>**TIM8_CH3**<br/>TIM3_CH3              |
| Right phase C low  | 27  | PB1<br/>TIM3_CH4 / **TIM8_CH3N**<br/>TIM1_CH3N |


All other pins are according to following table. Direction shows what is sensible regarding the circuit around the pin. Bold marks the 
function(s) used in the firmware.

| Function                    | Direction | Pin number | Pin functions                                                               |
|-----------------------------|-----------|------------|-----------------------------------------------------------------------------|
| Power LED                   | **O**     | 28         | **PB2** / BOOT1                                                             |
| Power switch voltage        | **I**     | 15         | PA1<br/>USART2_RTS / ADC123_IN1 / TIM5_CH2 / TIM2_CH2                       |
| Battery voltage             | **I**     | 10         | PC2<br/>ADC123_IN12                                                         |
| Power supply enable         | **O**     | 21         | PA5<br/>SPI1_SCK / DAC_OUT2 / ADC12_IN5                                     |
| Charger connected           | **Ipu**   | 45         | PA12<br/>USART1_RTS / USBDP / CAN_TX / TIM1_ETR                             |
| Buzzer                      | **O**     | 20         | **PA4**<br/>SPI1_NSS / USART2_CK / DAC_OUT1 / ADC12_IN4                     |
| Left Hall A                 | **I**/O   | 57         | PB5<br/>I2C1_SMBA / SPI3_MOSI / I2S3_SD<br/>TIM3_CH2 / SPI1_MOSI            |
| Left Hall B                 | **I**/O   | 58         | PB6<br/>I2C1_SCL / TIM4_CH1<br/>USART1_TX                                   |
| Left Hall C                 | **I**/O   | 59         | PB7<br/>I2C1_SDA / FSMC_NADV / TIM4_CH2<br/>USART1_RX                       |
| Left phase B voltage        | **I**     | 24         | PC4<br/>**ADC12_IN14**                                                      |
| Left phase C voltage        | **I**     | 25         | PC5<br/>**ADC12_IN15**                                                      |
| Left current                | **I**     | 9          | PC1<br/>**ADC123_IN11**                                                     |
| Left overcurrent            | **I**     | 33         | PB12<br/>SPI2_NSS/I2S2_WS / I2C2_SMBA / USART3_CK / TIM1_BKIN               |
| Left UART TX                | I/**O**   | 16         | PA2<br/>USART2_TX / TIM5_CH3 / ADC123_IN2 / TIM2_CH3                        |
| Left UART RX                | **I**/O   | 17         | PA3<br/>USART2_RX / TIM5_CH4 / ADC123_IN3 / TIM2_CH4                        |
| Right Hall A                | **I**/O   | 53         | PC12<br/>UART5_TX / SDIO_CK<br/>USART3_CK                                   |
| Right Hall B                | **I**/O   | 52         | PC11<br/>UART4_RX / SDIO_D3<br/>USART3_RX                                   |
| Right Hall C                | **I**/O   | 51         | PC10<br/>UART4_TX / SDIO_D2<br/>USART3_TX                                   |
| Right phase A voltage       | **I**     | 14         | PA0<br/>WKUP / USART2_CTS / **ADC123_IN0** / TIM2_CH1_ETR / TIM5_CH1 / TIM8_ETR |
| Right phase B voltage       | **I**     | 11         | PC3<br/>**ADC123_IN13**                                                     |
| Right current               | **I**     | 8          | PC0<br/>**ADC123_IN10**                                                     |
| Right overcurrent           | **I**     | 22         | PA6<br/>SPI1_MISO / TIM8_BKIN / ADC12_IN6 / TIM3_CH1<br/>TIM1_BKIN          |
| Right UART TX               | I/**O**   | 29         | PB10<br/>I2C2_SCL / USART3_TX<br/>TIM2_CH3                                  |
| Right UART RX               | **I**/O   | 30         | PB11<br/>I2C2_SDA / USART3_RX<br/>TIM2_CH4                                  |
| SWDIO                       | I/O       | 46         | **JTMS-SWDIO**<br/>PA13                                                     |
| SWCLK                       | I/O       | 49         | **JTCK-SWCLK**<br/>PA14                                                     |
| NRST                        | **I**     | 7          | **NRST**                                                                    |
