#include "bmp390.h"
#include "stm32f446xx.h"

float BMP390_compensate_temperature(uint32_t uncomp_temp, struct BMP390_calib_data *calib_data)
{

    float partial_data1;
    float partial_data2;

    partial_data1 = (float)(uncomp_temp - calib_data->par_t1);
    partial_data2 = (float)(partial_data1 * calib_data->par_t2);
    calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->par_t3;

    return calib_data->t_lin;
}

float BMP390_compensate_pressure(uint32_t uncomp_press, struct BMP390_calib_data *calib_data)
{

    float comp_press;
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    partial_data1 = calib_data->par_p6 * calib_data->t_lin;
    partial_data2 = calib_data->par_p7 * (calib_data->t_lin * calib_data->t_lin);
    partial_data3 = calib_data->par_p8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
    partial_out1 = calib_data->par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib_data->par_p2 * calib_data->t_lin;
    partial_data2 = calib_data->par_p3 * (calib_data->t_lin * calib_data->t_lin);
    partial_data3 = calib_data->par_p4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
    partial_out2 = (float)uncomp_press * (calib_data->par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (float)uncomp_press * (float)uncomp_press;
    partial_data2 = calib_data->par_p9 + calib_data->par_p10 * calib_data->t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->par_p11;
    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
}

uint8_t transferSPI(uint8_t addr)    //HOLDS THE MPU9250 REGISTER ADDRESS TO REQUEST DATA FROM
{
    uint8_t rx_data = 0;

    //SET SLAVE SELECT LOW
    GPIOB->ODR &= ~(1u << 1);

    //WRITE DATA AND DUMMY BYTE TO DATA REGISTER
    SPI1->DR = (uint16_t)(addr << 8);

    // Wait until transmit is complete
    while (!(SPI1->SR & SPI_SR_TXE));

    // Wait until receive buffer is full
    while (!(SPI1->SR & SPI_SR_RXNE));

    // Read received data
    data = SPI1->DR;

    //SET SLAVE SELECT HIGH
    GPIOB->ODR |= (1u << 1);

    return rx_data;
}
