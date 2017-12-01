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
    //clock_config();
    mstimer_init();

    //RCC_DeInit();
    RCC_Config();
    GPIO_Config();
    SPI_Config();
    I2C_Config();
    RTC_Config();
    RTC_AlarmConfig();

    sensor_init();

    GPIO_WriteBit(GPIOA, GPIO_Pin_1, 0);

    //GPIO_WriteBit(GPIOA, GPIO_Pin_1, 0);

    //*********************
    //Read the uC unique ID
    uint32_t idPart1 = STM32_UUID[0];
    uint32_t idPart2 = STM32_UUID[1];
    uint32_t idPart3 = STM32_UUID[2];

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
    char testdata[7] = {1, 1, 0, 0, 0};
    uint16_t temperature;
    uint16_t humidity;

	for (;;)
    {
        read_sensor(&temperature, &humidity);

        //Pack the sensor reading into the RF packet
        testdata[4] = temperature>>8;
        testdata[5] = temperature && 0xFF;
        testdata[6] = humidity>>8;
        testdata[7] = humidity && 0xFF;

        //Wakeup and send a packet
        RFM69_setMode(&rfm69, RFM69_MODE_STANDBY);
        RFM69_send(&rfm69, testdata, sizeof(testdata));

        //Put the RF module to sleep
        RFM69_sleep(&rfm69);

        GPIO_WriteBit(GPIOA, GPIO_Pin_1, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1));

        //simple_delay(50000);

        //Put the uC to sleep
        StandbyRTCMode();

        ///TODO: Clock still seems a little fast...
        ///TODO: Wake up for hall sensor interrupt

	}
    return(0);
}


void clock_config(void)
{
    //FLASH_SetLatency(FLASH_Latency_1);
    RCC_HSICmd(ENABLE);

    // Wait for HSIRDY after enabling HSI.
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) != SET){}

    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);             // Select the HSI as clock source.
    SystemCoreClockUpdate();
}

void RCC_Config(void)
{
    // PCLK2 = HCLK/2
    RCC_PCLKConfig(RCC_HCLK_Div2);

    // Enable PWR APB1 Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
}



void GPIO_Config(void)
{
    //Enable clock for GPOIA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    //****************************************
    //****************************************
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
}

//***********************************************************
// Configure the RTC peripheral by selecting the clock source
//***********************************************************
void RTC_Config(void)
{
    RTC_InitTypeDef RTC_InitStructure;
    RTC_TimeTypeDef  RTC_TimeStruct;

    // Enable the PWR clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    // Allow access to RTC
    PWR_BackupAccessCmd(ENABLE);

    // Enable the LSI OSC
    RCC_LSICmd(ENABLE);

    // Wait till LSI is ready
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET){}

    // Select the RTC Clock Source
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

    // Configure the RTC data register and RTC prescaler
    RTC_InitStructure.RTC_AsynchPrediv = 124;
    RTC_InitStructure.RTC_SynchPrediv  = 295;
    RTC_InitStructure.RTC_HourFormat   = RTC_HourFormat_24;
    RTC_Init(&RTC_InitStructure);

    // Set the time to 00h 00mn 00s AM
    RTC_TimeStruct.RTC_H12     = RTC_H12_AM;
    RTC_TimeStruct.RTC_Hours   = 0x00;
    RTC_TimeStruct.RTC_Minutes = 0x00;
    RTC_TimeStruct.RTC_Seconds = 0x00;
    RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);

    // Enable the RTC Clock
    RCC_RTCCLKCmd(ENABLE);

    // Wait for RTC APB registers synchronisation
    RTC_WaitForSynchro();
}


//************************
// Configure the RTC Alarm
void RTC_AlarmConfig(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    RTC_AlarmTypeDef RTC_AlarmStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // EXTI configuration
    EXTI_ClearITPendingBit(EXTI_Line17);
    EXTI_InitStructure.EXTI_Line = EXTI_Line17;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable the RTC Alarm Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //Disable the alarm
    RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

    //********************************
    //Read the current time in the RTC
    RTC_TimeTypeDef   RTC_TimeStruct;
    RTC_DateTypeDef RTC_DateStruct;
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

    RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = RTC_TimeStruct.RTC_H12;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours = RTC_TimeStruct.RTC_Hours;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = RTC_TimeStruct.RTC_Minutes;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = RTC_TimeStruct.RTC_Seconds+10;

    // Set the Alarm A
    RTC_AlarmStructure.RTC_AlarmDateWeekDay = RTC_DateStruct.RTC_WeekDay;
    RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;

    // Set the alarmA Masks (we only want minutes and seconds)
    RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours;
    RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

    // Enable AlarmA interrupt
    RTC_ITConfig(RTC_IT_ALRA, ENABLE);

    // Enable the alarmA
    RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

}

//**********************************
//Read the current time in the Alarm
void IncrementRtcAlarmA(void)
{
    RTC_AlarmTypeDef   RTC_AlarmStructure;

    //Disable the alarm
    RTC_AlarmCmd(RTC_Alarm_A, DISABLE);

    RTC_GetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

    //TODO Account for rollover
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes += 1;

    RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure);

    //Enable the alarm
    RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
}

//************************************************************
// This function handles RTC Alarm interrupt (A and B) request
void RTC_IRQHandler(void)
{
    uint32_t tmp =0;

    //Toggle the LED
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, !GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1));

    // Check on the AlarmA flag
    if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
    {
        // Clear RTC AlarmA Flags
        RTC_ClearITPendingBit(RTC_IT_ALRA);

        IncrementRtcAlarmA();
    }

    // Clear the EXTIL line 17
    EXTI_ClearITPendingBit(EXTI_Line17);
}

//****************************************
//Enter Standby mode with the RTC running
void StandbyRTCMode(void)
{
    // Request to enter STANDBY mode (Wake Up flag is cleared in PWR_EnterSTANDBYMode function)
    PWR_EnterSTANDBYMode();

    // Infinite loop
    while (1){}
}
