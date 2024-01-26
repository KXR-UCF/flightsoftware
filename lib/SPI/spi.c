#include "stm32f446xx.h"
#include "SPI.h"

void initClocks(void)
{
  RCC->AHB2ENR |= ((1u << 1)     //ENABLE GPIOA CLOCK
                  |(1u << 0)     //ENABLE GPIOB CLOCK
                  );

  RCC->APB2ENR |= (1u << 12);    //ENABLE SPI1 CLOCK
}

void configSpi1Pins(void)
{
    //SET PINS TO ALTERNATE FUNCTION MODE
    setPinMode();

    //SET ALTERNATE FUNCTION TO BE SPI1
    setAF();
}

void setPinMode(void)
{
    //RESET PIN MODES
    GPIOA->MODER &= ~((3u << (2 * 1))       //CLEAR PA1
                     |(3u << (2 * 11))      //CLEAR PA11
                     |(3u << (2 * 12))      //CLEAR PA12
                     );

    GPIOB->MODER &= ~(3u << (2 * 0));       //CLEAR PB0


    //CONFIGURE PIN MODES
    GPIOA->MODER |= ((2u << (2 * 1))        //SET PA1 TO AF
                    |(2u << (2 * 11))       //SET PA11 TO AF
                    |(2u << (2 * 12))       //SET PA12 TO AF
                    );

    GPIOB->MODER |= (1u << (2 * 8))          //SET PB8 TO OUTPUT MODE
                    |(1u << (2 * 15))        //SET PB15 TO OUTPUT MODE
                    |(1u << (2 * 13))        //SET PB13 TO OUTPUT MODE
                    |(1u << (2 * 1)          //SET PB1 TO OUTPUT MODE
                    );
}

void setAF(void)
{
    //RESET PIN ALTERNATE FUNCTION
    GPIOA->AFR[0] &= ~(                     //ACCESS AF LOWER. COVERS PINS 0 TO 7
                       (15u << (4 * 1))     //CLEAR PA1 AF
                      );

    GPIOA->AFR[1] &= ~(                     //ACCESS AF UPPER. COVERS PINS 8 TO 15
                       (15u << (4 * 3))     //CLEAR PA11 AF
                      |(15u << (4 * 4))     //CLEAR PA12 AF
                      );


    //SET PIN ALTERNATE FUNCTION
    GPIOA->AFR[0] |= (                      //ACCESS AF LOWER. COVERS PINS 0 TO 7
                      (5u << (4 * 1))       //SET SPI1 SCLK (PA1)
                     );

    GPIOA->AFR[1] |= (                      //ACCESS AF UPPER. COVERS PINS 8 TO 15
                      (5u << (4 * 3))       //SET SPI1 MISO (PA11)
                     |(5u << (4 * 4))       //SET SPI1 MOSI (PA12)
                     );
}

void initSPI(void)
{
    //INIT ALL REQUIRED CLOCKS FOR SPI1
    initClocks();

    //CONFIGURE PINS FOR SPI1
    configSpi1Pins();

    //INITIALISE SLAVE SELECT HIGH
    GPIOB->ODR |= (1u << 0);

    //CONFIGURE SPI1
    configSpi();
}

void configSpi(void)
{
    //CONFIGURE SPI1_CR1 REGISTER
    //CLEAR BITS
    SPI1->CR1 &= ~((1u << 15)           //FULL DUPLEX MODE
                  |(1u << 13)           //NOT INTERESTED IN CRC CLACULATIONS, DISABLE CRC
                  |(1u << 10)           //NOT INTERESTED IN SIMPLEX MODE
                  |(1u << 7)            //MSB FIRST
                  |(7u << 3)            //RESET BITS THAT DIVIDE THE FREQUENCY BEFORE SETTING THEM AGAIN
                  );

    //SET BITS
    SPI1->CR1 |= ((1u << 9)             //SOFTWARE SLAVE MANAGEMENT
                 |(1u << 8)             //INTERNAL SLAVE SELECT
                 |(5u << 3)             //DIVIDE SPI FREQUENCY BY 64
                 |(1u << 2)             //MASTER MODE
                 |(1u << 1)             //CLOCK POLARITY OF 1
                 |(1u << 0)             //CLOCK PHASE OF 1
                 );


    //CONFIGURE SPI1_CR2 REGISTER
    //CLEAR BITS
    SPI1->CR2 &= ~((1u << 12)           //RXNE EVENT TRIGGERED AT 1/2 (16-BIT) RX FIFO LEVEL
                  |(7u << 5)            //CLEAR INTERRUPT RELATED BITS AS WON'T BE USING INTERRUPTS
                  |(1u << 4)            //SPI IN MOTOROLA FORMAT
                  |(1u << 3)            //WON'T BE DOING CONSECUTIVE TRANSFERS
                  |(3u << 0)            //WON'T BE USING DMA, DISABLE TX AND RX DMA REQUEST GENERATION
                  );

    //SET BITS
    SPI1->CR2 |= (15u << 8);            //16-BIT DATA TRANSFERS

    //ENABLE SPI1
    SPI1->CR1 |= (1u << 6);
}

uint8_t transferSPI(uint8_t tx_data)    //HOLDS THE MPU9250 REGISTER ADDRESS TO REQUEST DATA FROM
{
    uint8_t rx_data = 0;

    //SET SLAVE SELECT LOW
    GPIOB->ODR &= ~(1u << 0);

    //WRITE DATA AND DUMMY BYTE TO DATA REGISTER
    SPI1->DR = (uint16_t)(tx_data << 8);

    //WAIT UNTIL SPI IS NOT BUSY AND RX BUFFER IS NOT EMPTY
    while( ((SPI1->SR)&(1u << 7)) || (!((SPI1->SR)&(1u << 0))) );

    //READ A BYTE FROM THE RX BUFFER
    rx_data = (uint8_t)SPI1->DR;

    //SET SLAVE SELECT HIGH
    GPIOB->ODR |= (1u << 0);

    return rx_data;
}
