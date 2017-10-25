#include "main.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"

//**************
//Initialise SPI
//**************
void SPI_Config(void)
{
    //***********************
    //Initialise the SPI pins
    GPIO_InitTypeDef    GPIO_InitStructure;

    /* Enable SPI clock, SPI1 */
    RCC_AHBPeriphClockCmd(SPI_GPIO_CLK, ENABLE);

    // SPI SCK, MOSI, MISO pin configuration
    GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN | SPI_MOSI_PIN | SPI_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1; // 10 MHz
    GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

    // Configure CS pin as output floating
    GPIO_InitStructure.GPIO_Pin = SPI_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SPI_CS_GPIO, &GPIO_InitStructure);

    //Setup GPIO alternative functions
    GPIO_PinAFConfig(SPI_GPIO, SPI_SCK_PIN, GPIO_AF_0);
    GPIO_PinAFConfig(SPI_GPIO, SPI_MOSI_PIN, GPIO_AF_0);
    GPIO_PinAFConfig(SPI_GPIO, SPI_MISO_PIN, GPIO_AF_0);

    //*****************************
    //Initialise the SPI peripheral
    SPI_InitTypeDef   SPI_InitStructure;

    //Enable SPI clock, SPI1
    RCC_APB2PeriphClockCmd(SPI_CLK, ENABLE);

    //Configure the SPI peripheral
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    //Initialise the peripheral
    SPI_Init(SPI_PERIPH, &SPI_InitStructure);

    //Set the SSOE bit
    SPI_PERIPH->CR2 |= 0x0C;

    //Enable the peripheral
    SPI_Cmd(SPI_PERIPH, ENABLE);

    return;
}

//************************
// SPI write a single byte
//************************
void SPI_write_byte(uint8_t data)
{
  // Wait for TX buffer empty (transfer finished)
  while ((SPI_PERIPH->SR & SPI_I2S_FLAG_TXE) == RESET);

  // Transfer
  *(__IO uint8_t *)&SPI_PERIPH->DR = data;

  // Wait for TX buffer empty (transfer finished)
  while( !(SPI1->SR & SPI_I2S_FLAG_TXE) );

  //Wait for SPI bus to no longer be busy
  while(SPI1->SR & SPI_I2S_FLAG_BSY);

  return;
}

//************************
// SPI read a single byte
//************************
uint8_t SPI_read_byte(uint8_t from)
{
  uint16_t data=0;

  // Wait for TX buffer empty (transfer finished)
  while (!(SPI_PERIPH->SR & SPI_I2S_FLAG_TXE));

  // Transfer
  data = from;               //The 8-bit register address followed by 0x00 to clock the response byte
  SPI_PERIPH->DR = data;

  // Wait for TX buffer empty (transfer finished)
  while( !(SPI1->SR & SPI_I2S_FLAG_TXE) );

  // Wait for RX buffer not empty (transfer finished) - this only works with 16-bit transfer :/
  while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));

  //Wait for SPI bus to no longer be busy
  while(SPI1->SR & SPI_I2S_FLAG_BSY);

  // Get received data
  data = SPI_PERIPH->DR;

  return (uint8_t)(data>>8);
}

//**********************
// Set the device CS Pin
//**********************
void SPI_select()
{
    GPIO_SetBits(SPI_CS_GPIO, SPI_CS_PIN);
}

//************************
// Reset the device CS pin
//************************
void SPI_unselect()
{
    GPIO_ResetBits(SPI_CS_GPIO, SPI_CS_PIN);
}
