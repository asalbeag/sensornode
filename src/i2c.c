#include "main.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"

//***************
// Initialise I2C
//***************
void I2C_Config()
{

    // Enable I2C clocks
    RCC_AHBPeriphClockCmd(I2C_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(I2C_CLK,ENABLE);

    //***********************
    //Initialise the I2C Pins
    GPIO_InitTypeDef    GPIO_InitStructure;

    // SDA, SCL pin configuration
    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN | I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
    GPIO_Init(I2C_GPIO, &GPIO_InitStructure);

    //Setup GPIO alternative functions
    //GPIO_PinAFConfig(I2C_GPIO, I2C_SDA_PIN, GPIO_AF_4);
    //GPIO_PinAFConfig(I2C_GPIO, I2C_SCL_PIN, GPIO_AF_4);

    //Since the PinAFConfig function ^^^^^ doesn't seem to work, manually set PA9 and PA10 to Alternate Function 4
    I2C_GPIO->AFR[1] |= 0x0440;

    //*****************************
    //Initialise the I2C peripheral
    I2C_InitTypeDef   I2C_InitStructure;

    // Configure the I2C peripheral
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    //I2C_InitStructure.I2C_Timing = 0x10420F13;              //8MHz clock
    I2C_InitStructure.I2C_Timing = 0x00201D2B;            //48MHz clock

    //Initialise the peripheral
    I2C_Init(I2C_PERIPH, &I2C_InitStructure);

    //Enable the peripheral
    I2C_Cmd(I2C_PERIPH, ENABLE);
}

//****************************************************
// Select an I2C Register (usually to initiate a read)
//****************************************************
uint8_t I2C_select_register(uint8_t address, uint8_t reg)
{
    I2C_TransferHandling(I2C_PERIPH, address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TXIS) == RESET);

    //Register address
    I2C_SendData(I2C_PERIPH, (uint8_t)reg);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TC) == RESET);

    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH, I2C_ICR_STOPCF);

    return(0);
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
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_RXNE) == RESET);

    read_word = I2C_ReceiveData(I2C_PERIPH)<<8;
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_RXNE) == RESET);

    read_word |= I2C_ReceiveData(I2C_PERIPH);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_RXNE) == RESET);

    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH, I2C_ICR_STOPCF);

    return read_word;
}

//******************
// Write an I2C Word
//******************
uint8_t I2C_write_word(uint8_t address, uint8_t reg, uint16_t data)
{
    I2C_TransferHandling(I2C_PERIPH, address, 3, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TXIS) == RESET);

    //Register address
    I2C_SendData(I2C_PERIPH, (uint8_t)reg);
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TXIS) == RESET);        //TODO: Check the NACK Flag in these whiles to ensure we don't hang forever if this transaction fails!!

    //Send the word one byte at a time
    I2C_SendData(I2C_PERIPH, (uint8_t)(data>>8));
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TXIS) == RESET);

    I2C_SendData(I2C_PERIPH, (uint8_t)(data & 0xFF));
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_ISR_TC) == RESET);

    //Send Stop
    while(I2C_GetFlagStatus(I2C_PERIPH, I2C_FLAG_STOPF));
    I2C_ClearFlag(I2C_PERIPH, I2C_ICR_STOPCF);

    return(0);
}

