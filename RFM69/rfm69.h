/**
 * @file rfm69.hpp
 *
 * @brief RFM69 and RFM69HW library for sending and receiving packets in connection with a STM32 controller.
 * @date January, February 2015
 * @author André Heßling
 *
 * This is a protocol agnostic driver library for handling HopeRF's RFM69 433/868/915 MHz RF modules.
 * Support is also available for the +20 dBm high power modules called RFM69HW/RFM69HCW.
 *
 * A CSMA/CA (carrier sense multiple access) algorithm can be enabled to avoid collisions.
 * If you want to enable CSMA, you should initialize the random number generator before.
 *
 * This library is written for the STM32 family of controllers, but can easily be ported to other devices.
 *
 * You have to provide your own functions for delay_ms and mstimer_get.
 * Use the SysTick timer (for example) with a 1 ms resolution which is present on all ARM controllers.
 *
 * If you want to port this library to other devices, you have to provide an SPI instance
 * derived from the SPIBase class.
 */

#include "stdbool.h"
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"

#ifndef RFM69_HPP_
#define RFM69_HPP_

#define RFM69_MAX_PAYLOAD		64 ///< Maximum bytes payload

#define RFM69_CS_GPIO           GPIOA
#define RFM69_CS_PIN            GPIO_Pin_4

#define RFM69_RESET_GPIO        GPIOA
#define RFM69_RESET_PIN         GPIO_Pin_3

//****************************
// Valid RFM69 operation modes
//****************************
typedef enum
{
  RFM69_MODE_SLEEP = 0,//!< Sleep mode (lowest power consumption)
  RFM69_MODE_STANDBY,  //!< Standby mode
  RFM69_MODE_FS,       //!< Frequency synthesizer enabled
  RFM69_MODE_TX,       //!< TX mode (carrier active)
  RFM69_MODE_RX        //!< RX mode
} RFM69Mode;

//***********************
// Valid RFM69 data modes
//***********************
typedef enum
{
  RFM69_DATA_MODE_PACKET = 0,                 //!< Packet engine active
  RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC = 2,   //!< Continuous mode with clock recovery
  RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC = 3,//!< Continuous mode without clock recovery
} RFM69DataMode;

// RFM69 driver library for STM32 controllers
typedef struct
{
  RFM69Mode mode;
  bool highPowerDevice;
  uint8_t powerLevel;
  int rssi;
  bool autoReadRSSI;
  bool ookEnabled;
  RFM69DataMode dataMode;
  bool highPowerSettings;
  bool csmaEnabled;
  char rxBuffer[RFM69_MAX_PAYLOAD];
  unsigned int rxBufferLength;
}RFM69_Struct;

//***********************
//RFM69 Control Functions
bool RFM69_init(RFM69_Struct *rfm69, bool highPowerDevice);
void RFM69_setResetPin();
void RFM69_setDataPin();
int RFM69_getRSSI(RFM69_Struct* rfm69);
void RFM69_setAutoReadRSSI(RFM69_Struct* rfm69, bool enable);
void RFM69_setCSMA(RFM69_Struct* rfm69, bool enable);
void RFM69_reset(RFM69_Struct *rfm69);
void RFM69_setFrequency(RFM69_Struct *rfm69, unsigned int frequency);
void RFM69_setFrequencyDeviation(RFM69_Struct *rfm69, unsigned int frequency);
void RFM69_setBitrate(RFM69_Struct *rfm69, unsigned int bitrate);
RFM69Mode RFM69_setMode(RFM69_Struct *rfm69, RFM69Mode mode);
void RFM69_setPowerLevel(RFM69_Struct *rfm69, uint8_t power);
int RFM69_setPowerDBm(RFM69_Struct *rfm69, int8_t dBm);
void RFM69_setHighPowerSettings(RFM69_Struct *rfm69, bool enable);
void RFM69_setCustomConfig(RFM69_Struct *rfm69, const uint8_t config[][2], unsigned int length);
int RFM69_send(RFM69_Struct *rfm69, const void* data, unsigned int dataLength);
int RFM69_receive(RFM69_Struct *rfm69, char* data, unsigned int dataLength);
void RFM69_sleep(RFM69_Struct *rfm69);
void RFM69_setOOKMode(RFM69_Struct *rfm69, bool enable);
void RFM69_setDataMode(RFM69_Struct *rfm69, RFM69DataMode dataMode);
void RFM69_continuousBit(RFM69_Struct *rfm69, bool bit);
void RFM69_dumpRegisters(RFM69_Struct *rfm69);
void RFM69_setPASettings(RFM69_Struct *rfm69, uint8_t forcePA);
bool RFM69_setAESEncryption(RFM69_Struct *rfm69, const void* aesKey, unsigned int keyLength);
uint8_t RFM69_readRegister(RFM69_Struct *rfm69, uint8_t reg);
void RFM69_writeRegister(RFM69_Struct *rfm69, uint8_t reg, uint8_t value);
void RFM69_chipSelect();
void RFM69_chipUnselect();
void RFM69_clearFIFO(RFM69_Struct *rfm69);
void RFM69_waitForModeReady(RFM69_Struct *rfm69);
void RFM69_waitForPacketSent(RFM69_Struct *rfm69);
int RFM69_readRSSI(RFM69_Struct *rfm69);
bool RFM69_channelFree(RFM69_Struct *rfm69);

#endif /* RFM69_HPP_ */

/** @}
 *
 */
