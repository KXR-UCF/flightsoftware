#ifndef SPI_H
#define SPI_H
#include "stm32f446xx.h"

void initClocks(void);

void initSPI(void);
void configSpi1Pins(void);
void setPinMode(void);
void setAF(void);
void configSpi(void);
uint8_t transferSPI(uint8_t tx_data);

#endif SPI_H
