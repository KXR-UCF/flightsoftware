#include "stm32f4xx_hal.h"
#include "bmp390.h"
#include "bmp3.h" 
#include <stdio.h>

static uint8_t dev_addr = 0;
uint8_t GTXBuffer[512], GRXBuffer[2048];

void BMP390_Check_Result(const char api_name[], int8_t result)
{
    switch (result)
    {
        case BMP3_OK:
            // Do nothing 
            break;
        case BMP3_E_NULL_PTR:
            printf("API [%s] Error [%d] : Null pointer\r\n", api_name, result);
            break;
        case BMP3_E_COMM_FAIL:
            printf("API [%s] Error [%d] : Communication failure\r\n", api_name, result);
            break;
        case BMP3_E_INVALID_LEN:
            printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, result);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            printf("API [%s] Error [%d] : Device not found\r\n", api_name, result);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, result);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, result);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, result);
            break;
        default:
            printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, result);
            break;
    }
}

void BMP390_SPI_Init(void) {
    HAL_SPI_Init(&hspi1);
}

BMP3_INTF_RET_TYPE BMP390_interface_init(struct BMP390 *bmp390) {
    int8_t result = BMP_OK;

    if(bmp390 != NULL) {
		dev_addr = 0;
		BMP390->read = BMP390_SPI_Read;
		BMP390->write = BMP390_SPI_Write;

        bmp3->delay_us = bmp3_delay_us;
	    bmp3->intf_ptr = &dev_addr;
    } else {
        return -1;
    }

    return result;
}

int8_t BMP390_SPI_Read(uint8_t subaddress, uint8_t *pBuffer, uint16_t ReadNumbr, void *intf_ptr)
{
    GTXBuffer[0] = subaddress | 0x80;

    HAL_GPIO_WritePin(GPIOA, LD2_Pin|ncs_sensor_Pin, GPIO_PIN_RESET); // NSS low

    HAL_SPI_TransmitReceive(&hspi1, GTXBuffer, GRXBuffer, ReadNumbr+1, BUS_TIMEOUT); // timeout 1000msec;
    while(hspi1.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(GPIOA, LD2_Pin|ncs_sensor_Pin, GPIO_PIN_SET);  // NSS high
    memcpy(pBuffer, GRXBuffer+1, ReadNumbr);

    return 0;
}

int8_t BMP390_SPI_Write(uint8_t subaddress, uint8_t *pBuffer, uint16_t WriteNumbr, void *intf_ptr)
{
    GTXBuffer[0] = subaddress & 0x7F;
    memcpy(&GTXBuffer[1], pBuffer, WriteNumbr);

    HAL_GPIO_WritePin(GPIOA, LD2_Pin|ncs_sensor_Pin, GPIO_PIN_RESET); // NSS low

    HAL_SPI_Transmit(&hspi1, GTXBuffer, WriteNumbr+1, BUS_TIMEOUT); // send register address + write data
    while(hspi1.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(GPIOA, LD2_Pin|ncs_sensor_Pin, GPIO_PIN_SET);  // NSS high

    return 0;
}

// Read sensor example
int main(void) {
    #define ITERATION UINT8_C(100)

    int8_t result;
    uint8_t loop = 0;
    uint8_t settings_sel;

    struct BMP390 dev;
    struct BMP390_Data data = { 0 };
    struct BMP390_settings settings = { 0 };
    struct BMP390_status status = { { 0 } };

    // SPI init
    BMP390_SPI_Init();

    // BMP390 init
    result = BMP390_interface_init(&dev);
    BMP390_Check_Result("BMP390_interface_init", result);

    // Enable sensors
    settings.int_settings.drdy_en = BMP3_ENABLE;
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;

    // Sensor settings
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_100_HZ; 
    
    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_DRDY_EN;

    result = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    BMP390_Check_Result("bmp3_set_sensor_settings", result);

    settings.op_mode = BMP3_MODE_NORMAL;
    result = bmp3_set_op_mode(&settings, &dev);
    BMP390_Check_Result("bmp3_set_op_mode", result);

    // Read temperature and pressure
    while(loop < ITERATION) {
        result = bmp3_get_status(&status, &dev);
        BMP390_Check_Result("bmp3_get_status", result);

        result = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
		BMP390_Check_Result("bmp3_get_sensor_data", result);

		result = bmp3_get_status(&status, &dev);
		BMP390_Check_Result("bmp3_get_status", result);

        #ifdef BMP3_FLOAT_COMPENSATION
        printf("Data[%d]  T: %.2f deg C, P: %.2f Pa\n", loop, (data.temperature), (data.pressure));
        #else
        printf("Data[%d]  T: %ld deg C, P: %lu Pa\n", loop, (long int)(int32_t)(data.temperature / 100), (long unsigned int)(uint32_t)(data.pressure / 100));
        #endif

        loop = loop + 1;
    }

    return result;
}