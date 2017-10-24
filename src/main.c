/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   Last committed:     $Revision: 00 $
   Last changed by:    $Author: $
   Last changed date:  $Date:  $
   ID:                 $Id:  $

**********************************************************************/
#include <stdint.h>
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "main.h"
#include "delay.h"
#include "RFM69.h"

/* timing is not guaranteed :) */
void simple_delay(uint32_t us)
{
	/* simple delay loop */
	while (us--)
    {
		asm volatile ("nop");
	}
}

/* system entry point */
int main(void)
{
    mstimer_init();

    //RCC_DeInit();
    RCC_Config();
    GPIO_Config();
    SPI_Config();


    // setup RFM69 and optional reset
    RFM69_Struct rfm69;
    RFM69_chipUnselect();
    RFM69_reset(&rfm69);

    //Test
    uint32_t timeEntry = mstimer_get();
    //uint16_t tempvalue = RFM69_readRegister(&rfm69, 0x5C);


    RFM69_init(&rfm69, true);

    RFM69_setAESEncryption(&rfm69, 0, 0);


    // init RF module and put it to sleep
    RFM69_sleep(&rfm69);


    // check if a packet has been received
    char rx[64];

    int bytesReceived = RFM69_receive(&rfm69, rx, sizeof(rx));
/*
    for (;;)
    {
        // set led on
		GPIO_SetBits(GPIOA, GPIO_Pin_1);

        // receive a packet
        RFM69_receive(&rfm69, rx, 64);

		// delay
		simple_delay(500000);

		// clear led
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);

		// delay
		simple_delay(500000);
    }
*/

    char testdata[4] = {1, 1, 0, 0x55};
	for (;;)
    {
		// set led on
		GPIO_SetBits(GPIOA, GPIO_Pin_1);

        // send a packet and let RF module sleep

        RFM69_send(&rfm69, testdata, sizeof(testdata));

        RFM69_setMode(&rfm69, RFM69_MODE_RX);
        // delay
		simple_delay(100000);

        bytesReceived = RFM69_receive(&rfm69, rx, sizeof(rx));
        //RFM69_sleep(&rfm69);


		// clear led
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);

		// delay
		simple_delay(100000);

	}
	/*
    char testdata[5] = {1, 2, 0x40, 'l', 'o'};
	for (;;)
    {
		// set led on
		GPIO_SetBits(GPIOA, GPIO_Pin_1);

        // send a packet and let RF module sleep

        RFM69_send(&rfm69, testdata, sizeof(testdata));
        RFM69_sleep(&rfm69);

		// delay
		simple_delay(1000000);

		// clear led
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);

		// delay
		simple_delay(1000000);

	}
    */
	//Read temperature sensor
	//Send Data
	//Sleep, wake up only for button interrupt or accelerometer interrupt
    return(0);
}


void RCC_Config(void)
{
  // PCLK2 = HCLK/2
  RCC_PCLKConfig(RCC_HCLK_Div2);
}



void GPIO_Config(void)
{
    //Enable clock for GPOIA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);


    // gpio init struct
	GPIO_InitTypeDef GPIO_Init_LED;

    // initialize gpio structure
	GPIO_StructInit(&GPIO_Init_LED);

	// use pin 0
	GPIO_Init_LED.GPIO_Pin = LED_PIN;

	// mode: output
	GPIO_Init_LED.GPIO_Mode = GPIO_Mode_OUT;

	// output type: push-pull
	GPIO_Init_LED.GPIO_OType = GPIO_OType_PP;

	// apply configuration
	GPIO_Init(LED_GPIO, &GPIO_Init_LED);


    GPIO_InitTypeDef GPIO_Init_SPI;

    // Configure SPIz pins: SCK, MISO and MOSI ---------------------------------
    GPIO_Init_SPI.GPIO_Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
    // Configure SCK and MOSI pins as Alternate Function Push Pull
    GPIO_Init_SPI.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init_SPI.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPI_GPIO, &GPIO_Init_SPI);

    GPIO_Init_SPI.GPIO_Pin = SPI_MISO_PIN;
    // Configure MISO pin as Input Floating
    GPIO_Init_SPI.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(SPI_GPIO, &GPIO_Init_SPI);

}

