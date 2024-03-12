#ifndef AD7248
#define AD7248

void AD7124_Init(AD7124_Device *device, SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin);
int AD7124_Reset(AD7124_Device *device);
int AD7124_ReadRegister(AD7124_Device *device, AD7124_RegisterID reg, uint32_t *data);
int AD7124_WriteRegister(AD7124_Device *device, AD7124_RegisterID reg, uint32_t data);

#endif // !AD7248