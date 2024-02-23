#include <stm32f446xx.h>

#define MODIFY_FIELD(reg, field, value)  \
    ((reg)) = ((reg) & ~(field##_Msk)) | \
              (((uint32_t)(value) << field##_Pos) & field##_Msk)

void Init_SPI1(void)
{
    // Clock gating for SPI1 and GPIOA and B
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // GPIO B pin 1, 8, 13, 15 to NSS
    MODIFY_FIELD(GPIOB->MODER, GPIO_MODER_MODER1, 2);
    MODIFY_FIELD(GPIOB->MODER, GPIO_MODER_MODER8, 2);
    MODIFY_FIELD(GPIOB->MODER, GPIO_MODER_MODER13, 2);
    MODIFY_FIELD(GPIOB->MODER, GPIO_MODER_MODER15, 2);

    // GPIO A pin 1, 11, 12 to AF
    MODIFY_FIELD(GPIOB->MODER, GPIO_MODER_MODER1, 0);
    MODIFY_FIELD(GPIOB->MODER, GPIO_MODER_MODER11, 0);
    MODIFY_FIELD(GPIOB->MODER, GPIO_MODER_MODER12, 0);

    // Clock is divided by 16 (2^(BR+1))
    MODIFY_FIELD(SPI1->CR1, SPI_CR1_BR, 3);
    MODIFY_FIELD(SPI1->CR1, SPI_CR1_MSTR, 1); // Master Mode

    // Select second edge sample, active high clock
    MODIFY_FIELD(SPI1->CR1, SPI_CR1_CPHA, 1);
    MODIFY_FIELD(SPI1->CR1, SPI_CR1_CPOL, 1);

    // Data is MSB first
    MODIFY_FIELD(SPI1->CR1, SPI_CR1_LSBFIRST, 0);

    // SPI in motorola mode
    MODIFY_FIELD(SPI1->CR2, SPI_CR2_FRF, 0);

    // Enable SPI1
    MODIFY_FIELD(SPI1->CR1, SPI_CR1_SPE, 1);
}

// #define F_TIM_CLOCK (48UL * 1000UL * 1000UL) // 48MHz
// #define F_TIM_OVERVIEW (100UL * 1000UL)      // 100 kHz
// #define TIM_PRESCALER (16)
// #define TIM_PERIOD (F_TIM_CLOCK / (TIM_PRESCALER * F_TIM_OVERVIEW))

// void Init_DMA(void)
// {
//     // Set up prescaler and counter to get 100 kHz
//     RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
//     TIM6->ARR = TIM_PERIOD - 1;
//     // Enable DMA request on update
//     TIM6->PSC = TIM_PRESCALER - 1;
//     // Enable counting
//     TIM6->DIER = TIM_DIER_UDE;
// }

// void Init_SPI2(void) {}

// uint8_t SPI_Send_Recieve_Byte(uint8_t d_out)
// {
//     uint8_t d_in;
//     // Wait until the transmitter buffer is empty
//     while ((SPI1->SR & SPI_SR_TXE) == 0)
//         ;

//     // Transmit d_out
//     // Must tell compiler to use a byte write (not half-word)
//     // by casting SPI1->DR into a pointer to a byte (uint8_t)
//     *((uint8_t *)&(SPI1->DR)) = d_out;
//     // Wait until receiver is not empty
//     while ((SPI->SR & SPI_SR_RXNE) == 0)
//         ;
//     // Get d_in
//     d_in = (uint8_t)SPI1->DR;
//     return d_in;
// }

int main(void)
{
    return 0;
}