#include "stm32f446xx.h"
#include "bno08x.h"

uint8_t transferSPI(uint8_t tx_data)    //HOLDS THE MPU9250 REGISTER ADDRESS TO REQUEST DATA FROM
{
    uint8_t rx_data = 0;

    //SET SLAVE SELECT LOW
    GPIOB->ODR &= ~(1u << 8);

    //WRITE DATA AND DUMMY BYTE TO DATA REGISTER
    SPI1->DR = (uint16_t)(tx_data << 8);

    //WAIT UNTIL SPI IS NOT BUSY AND RX BUFFER IS NOT EMPTY
    while( ((SPI1->SR)&(1u << 7)) || (!((SPI1->SR)&(1u << 0))) );

    //READ A BYTE FROM THE RX BUFFER
    rx_data = (uint8_t)SPI1->DR;

    //SET SLAVE SELECT HIGH
    GPIOB->ODR |= (1u << 8);

    return rx_data;
}
