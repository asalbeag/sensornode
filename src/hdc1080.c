#include "main.h"

//*************************
//C = ((temp/2^16)*165)-40;
//H = ((humidity/2^16)*100;
uint8_t read_sensor(uint16_t* temperature, uint16_t* humidity)
{
    //Write to register 0 to initialise a read
    I2C_select_register(SENSOR_ADDRESS, 0x00);

    //Wait until the read completes ~3.65mS for temperature and ~2.5mS for humidity
    delay_ms(10);

    I2C_TransferHandling(I2C_PERIPH, SENSOR_ADDRESS, 4, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_RXNE) == RESET);

    *temperature = I2C_ReceiveData(I2C_PERIPH)<<8;
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_RXNE) == RESET);

    *temperature |= I2C_ReceiveData(I2C_PERIPH);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_RXNE) == RESET);


    *humidity = I2C_ReceiveData(I2C_PERIPH)<<8;
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_RXNE) == RESET);

    *humidity |= I2C_ReceiveData(I2C_PERIPH);

    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_STOPF) == RESET);
    I2C_ClearFlag(I2C_PERIPH, I2C_ICR_STOPCF);

    return(0);
}

void sensor_init()
{
    //Measure both temperature and humidity
    //11bit temperature resolution
    //8bit humidity resolution
    //Heater Disabled
    uint16_t data = 0x1C00;
    I2C_write_word(SENSOR_ADDRESS, 0x02, data);
    return;
}
