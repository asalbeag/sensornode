#include "main.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"

//***************
// Initialise I2C
//***************
void I2C_Config()
{
    //***********************
    //Initialise the I2C Pins
    GPIO_InitTypeDef    GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(I2C_GPIO_CLK, ENABLE);

    // SDA, SCL pin configuration
    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN | I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;     // 10 MHz
    GPIO_Init(I2C_GPIO, &GPIO_InitStructure);

    //Setup GPIO alternative functions
    GPIO_PinAFConfig(I2C_GPIO, I2C_SDA_PIN, GPIO_AF_1);
    GPIO_PinAFConfig(I2C_GPIO, I2C_SCL_PIN, GPIO_AF_1);


    //*****************************
    //Initialise the I2C peripheral
    I2C_InitTypeDef   I2C_InitStructure;

    // Enable I2C clock, I2C1
    RCC_APB1PeriphClockCmd(I2C_CLK,ENABLE);

    // Configure the I2C peripheral
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Timing = 0x00201D2B;

    //Initialise the peripheral
    I2C_Init(I2C_PERIPH, &I2C_InitStructure);

    //Enable the peripheral
    I2C_Cmd(I2C_PERIPH, ENABLE);
}

//*****************
// Read an I2C Word
//*****************
uint16_t I2C_read_word(uint8_t address, uint8_t reg)
{
    uint16_t read_word;
    uint8_t bytes = 2;

    I2C_TransferHandling(I2C_PERIPH, address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TXIS) == RESET);

    I2C_SendData(I2C_PERIPH, (uint8_t)reg);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TC) == RESET);

    I2C_TransferHandling(I2C_PERIPH, address, bytes, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

    read_word = I2C_ReceiveData(I2C_PERIPH)<<8;
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_RXNE) == RESET);

    read_word |= I2C_ReceiveData(I2C_PERIPH);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_RXNE) == RESET);

    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH, I2C_ICR_STOPCF);

    return read_word;
}

//*******************
// Write and I2C Word
//*******************
uint8_t I2C_write_word(uint8_t address, uint8_t reg, uint16_t data)
{
    uint8_t bytes = 2;

    I2C_TransferHandling(I2C_PERIPH, address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TXIS) == RESET);

    I2C_SendData(I2C_PERIPH, (uint8_t)reg);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TC) == RESET);

    I2C_TransferHandling(I2C_PERIPH, address, bytes, I2C_AutoEnd_Mode,I2C_Generate_Start_Read);

    I2C_SendData(I2C_PERIPH, (uint8_t)(data>>8));
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TC) == RESET);

    I2C_SendData(I2C_PERIPH, (uint8_t)(data & 0xFF));
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TC) == RESET);

    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH, I2C_ICR_STOPCF);

    return(0);
}
