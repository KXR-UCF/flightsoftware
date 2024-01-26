#include "stm32f446xx.h"
#include "SPI.h"

// void initSPI(void);
// void initClocks(void);
// void configSpi1Pins(void);
// void setPinMode(void);
// void setAF(void);
// void configSpi(void);
// uint8_t transferSPI(uint8_t tx_data);

// void initClocks(void)
// {
//     /*
//     1. ENABLE GPIOA CLOCK and GPIOB CLOCK (6.3.10 RM0390 Rev 6)
//     2. ENABLE SPI1 CLOCK (6.3.14  RM0390 Rev 6)

//     */
//     RCC->AHB1ENR |= ((0b1 << 0) | (0b1 << 1));
//     RCC->APB2ENR |= (0b1 << 12);
// }

// void setPinMode(void)
// {
//     /*
//     1. Reset Pin Modes - Clear PA1, Clear PA11, Clear PA12, Clear PB0 (7.4.1 RM0390 Rev 6)
//     2. Configure Pin Modes - PA1 to AF, PA11 to AF, PA12 to AF, PB0 to Output (7.4.1 RM0390 Rev 6))
//     */

//     // Reset Pin Modes
//     GPIOA->MODER &= ~((0b11 << (2 * 1)) | (0b11 << (2 * 11) | (0b11 << (2 * 12))));
//     GPIOB->MODER &= ~((0b11 << (2 * 0)));

//     // Configure Pin Modes
//     GPIOA->MODER |= ((0b10 << (2 * 1)) | (0b10 << (2 * 11)) | (0b10 << (2 * 12)));
//     GPIOB->MODER |= (0b01 << (2 * 0));
// }

// void setAF(void)
// {
//     /*
//     1. Reset Pin alternate function - Lower Clear PA1, Upper Clear PA11 and Clear PA12 (7.4.10 - 7.4.11 RM0390 Rev 6)
//     */

//     // Reset Pin alternate function
//     GPIOA->AFR[0] &= ~(0b1111 << (4 * 1));
//     GPIOA->AFR[1] &= ~((0b1111 << (4 * 3)) | (0b1111 << (4 * 4)));

//     // Set Pin alternate function
//     GPIOA->AFR[0] |= (0b0101 << (4 * 1));
//     GPIOA->AFR[1] |= (0b0101 << (4 * 3) | 0b101 << (4 * 4));
// }

// void configSpi1Pins(void)
// {
//     /*
//     1. Clear Control Register 1 Bits (26.7.1, 26.3.5 RM0390)
//         - Full Duplex Mode
//         - Not interested in CRC Calculations, disable crc
//         - Not interested in simplex mode
//         - msb first
//         - reset bits that divide the frequency before setting them again

//     2. Set Control Register 1 Bits (26.7.1, 26.3.5 RM0390)
//         - Software slave management
//         - Internal Slave Select
//         - Divide SPI frequency by 64
//         - Master Mode
//         - Clock Polarity of 1
//         - Clock Phase of 1

//     3. Clear Control Register 2 Bits (26.7.2 RM0390)
//         - RXNE event triggered at 1/2 (16-BIT) RX FIFO level
//         - Clear interrupt related bits as wont be using interupts
//         - SPI in motorola format
//         - Wont be doing consecutive transfes
//         - Wont be using DMA, Diable TX and RX DMA request Generation

//     4. Set Control Register 2 Bits (26.7.2 RM0390)
//         - 16-bit data transfers

//     */

//     // Clear Bits
//     SPI1->CR1 &= ~((0b1u << 15) | (0b1u << 13) | (0b1u << 10) | (0b1u << 11) | (0b1u << 7) | (0b111u << 3));

//     // Set Bits
//     SPI1->CR1 |= ((0b1u << 9) | (0b1u << 8) | (0b101u << 3) | (0b1u << 2) | (0b1u << 1) | (0b1u << 0));

//     // Clear Bits
//     SPI1->CR2 = 0;

//     // Enable SPI1
//     SPI1->CR1 |= (1u << 6);
// }

// void initSPI(void)
// {
//     /*
//     26.3.12
//     */
//     initClocks();

//     configSpi1Pins();

//     // Initialize Slave Select High
//     GPIOB->ODR |= 1u;

//     configSpi();
// }

// uint8_t transferSPI(uint8_t tx_data)
// {
//     uint8_t rx_data = 0;

//     // Set slave select low
//     GPIOB->ODR &= ~(1u << 0);

//     // write data and dummy byte to data register
//     SPI1->DR = (uint16_t)(tx_data << 8);

//     // wait until spi is not busy and rx buffer is not empty
//     while (((SPI1->SR) & (1u << 7)) || (!((SPI1->SR) & (1u << 0))))
//         ;

//     // read a byte from the rx buffer
//     rx_data = (uint8_t)SPI1->DR;

//     // Set slave select high
//     GPIOB->ODR |= (1u << 0);

//     return rx_data;
// }

int main(void)
{
    // MPU9250 Addresses
    uint8_t data[4] = {187, 188, 189, 190};

    // Stores data recieved from mpu9250
    uint8_t rxd = 0;

    // set up the timer
    initTim2();

    // setup spi master
    initSPI();

    while (1)
    {
        // Write to SPI
        rxd = transferSPI(data[0]);

        // Delay MS
        delay(50);
    }
}
