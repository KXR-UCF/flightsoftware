#ifndef STM32F446ZE
#define STM32F446ZE
#include "stm32f446xx.h"
#endif

void initClocks(void);

void initSPI(void);
void configSpi1Pins(void);
void setPinMode(void);
void setAF(void);
void configSpi(void);
uint8_t transferSPI(uint8_t tx_data);
