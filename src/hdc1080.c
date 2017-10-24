#include "main.h"

uint8_t read_sensor(uint16_t* temperature, uint16_t* humidity)
{
    //Write to register 0 to initialise a read
    I2C_write_word(SENSOR_ADDRESS, 0x00, 0x01);

    //Wait until the read completes ~3.65mS for temperature and ~2.5mS for humidity
    delay_ms(7);

    //The sensor address is 0x01, the register adresses are 0x0A and 0x0B
    *temperature = I2C_read_word(SENSOR_ADDRESS, 0x00);
    *humidity = I2C_read_word(SENSOR_ADDRESS, 0x01);

    return(0);
}

void sensor_init()
{
    //Measure both temperature and humidity
    //11bit temperature resolution
    //8bit humidity resolution
    //Heater Disabled
    uint16_t data = 0x1600;
    I2C_write_word(SENSOR_ADDRESS, 0x02, data);
    return;
}
