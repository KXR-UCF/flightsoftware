#ifndef BMP390_H
#define BMP390_H
#include "stm32f446xx.h"

#define CMD 0x7E

// Calibration data
#define NVM_PAR_P11 0x45
#define NVM_PAR_P10 0x44
#define NVM_PAR_P9_MSB 0x43
#define NVM_PAR_P9_LSB 0x42
#define NVM_PAR_P8 0x41
#define NVM_PAR_P7 0x40
#define NVM_PAR_P6_MSB 0x3F
#define NVM_PAR_P6_LSB 0x3E
#define NVM_PAR_P5_MSB 0x3D
#define NVM_PAR_P5_LSB 0x3C
#define NVM_PAR_P4 0x3B
#define NVM_PAR_P3 0x3A
#define NVM_PAR_P2_MSB 0x39
#define NVM_PAR_P2_LSB 0x38
#define NVM_PAR_P1_MSB 0x37
#define NVM_PAR_P1_LSB 0x36
#define NVM_PAR_T3 0x35
#define NVM_PAR_T2_MSB 0x34
#define NVM_PAR_T2_LSB 0x33
#define NVM_PAR_T1_MSB 0x32
#define NVM_PAR_T1_LSB 0x31
//
#define CONFIG 0x1F
#define ODR 0x1D
#define OSR 0x1C
#define PWR_CTRL 0x1B
#define IF_CONF 0x1A
#define INT_CTRL 0x19
#define FIFO_CONFIG_2 0x18
#define FIFO_CONFIG_1 0x17
#define FIFO_WTM_1 0x16
#define FIFO_WTM_0 0x15
#define FIFO_DATA 0x14
#define FIFO_LENGTH_1 0x13
#define FIFO_LENGTH_0 0x12
#define INT_STATUS 0x11
#define EVENT 0x10

// DATA
#define SENSOR_TIME_MSB 0x0E
#define SENSOR_TIME_LSB 0x0D
#define SENSOR_TIME_XLSB 0x0C
#define TEMP_MSB 0x09
#define TEMP_LSB 0x08
#define TEMP_XLSB 0x07
#define PRES_MSB 0x06
#define PRES_LSB 0x05
#define PRES_XLSB 0x04

#define STATUS 0x03
#define ERR_REG 0x02
#define REV_ID 0x01
#define CHIP_ID 0x00

typedef struct
{
    int8_t par_p11;
    int8_t par_p10;
    int16_t par_p9;
    int8_t par_p8;
    int8_t par_p7;
    uint16_t par_p6;
    uint8_t par_p5;
    int8_t par_p4;
    int8_t par_p3;
    int16_t par_p2;
    int16_t par_p1;
    int8_t par_t3;
    uint16_t par_t2;
    uint16_t par_t1;
    float t_lin;
} BMP390_calib_data;

typedef struct
{

} BMP390;

void initClocks(void);

void initSPI(void);
void configSpi1Pins(void);
void setPinMode(void);
void setAF(void);
void configSpi(void);
uint8_t transferSPI(uint8_t tx_data);


uint8_t bmp390_init(bmp390_interface_t interface, bmp390_address_t addr_pin);
uint8_t bmp390_deinit(void);
uint8_t bmp390_read(float *temperature_c, float *pressure_pa);

float BMP390_compensate_temperature(uint32_t uncomp_temp, struct BMP390_calib_data calib_data);
float BMP390_compensate_pressure(uint32_t uncomp_press, struct BMP390_calib_data calib_data);


#endif // !BMP390_H
