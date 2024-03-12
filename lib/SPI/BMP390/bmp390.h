#ifndef BMP390_H
#define BMP390_H

#include "stm32f446xx.h"
#include "bmp3_defs.h"
#include <stdint.h>

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

// Data
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

// Compensated
typedef struct
{
    uint64_t pressure;

    int64_t temperature;

} BMP390_uncomp_data;

// Uncompensated
typedef struct
{
    double temperature;

    double pressure;

} BMP390_data;

// Device settings
typedef struct
{
    // Output mode
    uint8_t op_mode;

    // Active high/low
    uint8_t level;

    // Latched or Non-latched
    uint8_t latch;

    // Data ready interrupt
    uint8_t drdy_en;

} BMP390_int_ctrl_settings;

typedef struct 
{
    // Pressure oversampling
    uint8_t pressure_os;

    // Temperature oversampling
    uint8_t temp_os;

    // IIR filter
    uint8_t iir_filter;

    // Output data rate
    uint8_t odr;

} BMP390_odr_filter_settings;

typedef struct
{
    // Power mode
    uint8_t op_mode;

    // Enable/disable pressure sensor
    uint8_t press_en;

    // Enable/disable temperature sensore
    uint8_t temp_en;

    // ODR and filter
    struct BMP390_odr_filter_settings odr_filter;

    // Interrupt
    struct BMP390_int_ctrl_settings int_settings;

} BMP390_settings;

typedef struct 
{
    // Command ready status
    uint8_t cmd_rdy;

    // Data ready for pressure
    uint8_t drdy_press;

    // Data ready for temperature
    uint8_t drdy_temp;

} BMP_sens_status;

typedef struct 
{
    // Fatal error
    uint8_t fatal;

    // Command error
    uint8_t cmd;

    // Config error
    uint8_t config;

} BMP_err_status;

typedef struct 
{
    // Sensor
    struct BMP390_sens_status;

    // Error
    struct BMP390_err_status err;

    // Power on reset
    uint8_t power_on_reset;

} BMP_status;

typedef struct
{
    uint8_t chip_id;

    void *intf_ptr;

    enum intf = BMP390_SPI_INTF;

    BMP390_read_fptr_t read;
    BMP390_write_fptr_t write;

    BMP390_delay_us_fptr_t delay_us;

    struct BMP390_calib_data calib_data;

} BMP390;

int8_t bmp390_init(struct BMP390 *dev); // initialize sensor
int8_t bmp390_set_sensor(uint32_t desired, struct BMP390_settings *settings, struct BMP390 *dev); // enable/diable pressure/temperature

#define SPI_HANDLE	(hspi1)

#define BUS_TIMEOUT             1000

void bmp3_delay_us(uint32_t period, void *intf_ptr);

void BMP390_SPI_Init(void);

#ifndef USE_BOSCH_SENSOR_API
#define USE_BOSCH_SENSOR_API

int8_t BMP390_SPI_Read(uint8_t subaddress, uint8_t *pBuffer, uint16_t ReadNumbr, void *intf_ptr);
int8_t BMP390_SPI_Write(uint8_t subaddress, uint8_t *pBuffer, uint16_t WriteNumbr, void *intf_ptr);

#endif

#endif // !BMP390_H
