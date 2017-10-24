#include <stdint.h>
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

#ifndef MAIN_H_
#define MAIN_H_

#define SENSOR_ADDRESS          0x80        //7-bit address

#define SPI_PERIPH              SPI1
#define SPI_CLK                 RCC_APB2Periph_SPI1
#define SPI_GPIO                GPIOA
#define SPI_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define SPI_SCK_PIN             GPIO_Pin_5
#define SPI_MISO_PIN            GPIO_Pin_6
#define SPI_MOSI_PIN            GPIO_Pin_7
#define SPI_CS_GPIO             GPIOA
#define SPI_CS_GPIO_CLK         RCC_AHBPeriph_GPIOA
#define SPI_CS_PIN              GPIO_Pin_4

#define I2C_PERIPH              I2C1
#define I2C_CLK                 RCC_APB1Periph_I2C1
#define I2C_GPIO                GPIOA
#define I2C_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define I2C_SCL_PIN             GPIO_Pin_9
#define I2C_SDA_PIN             GPIO_Pin_10

#define LED_GPIO                GPIOA
#define LED_GPIO_CLK            RCC_AHBPeriph_GPIOA
#define LED_PIN                 GPIO_Pin_1

//*********
//Functions
//*********

//main.c
void RCC_Config(void);
void GPIO_Config(void);

//i2c.c
void I2C_Config(void);
uint16_t I2C_read_word(uint8_t address, uint8_t reg);
uint8_t I2C_write_word(uint8_t address, uint8_t reg, uint16_t data);

//spi.c
void SPI_Config(void);
void SPI_write_byte(uint8_t data);
uint8_t SPI_read_byte(uint8_t from);
void SPI_select();
void SPI_unselect();

//hdc1080.c
void sensor_init(void);
uint8_t read_sensor(uint16_t* temperature, uint16_t* humidity);

#endif /* MAIN_H_ */
