/**
 * @file rfm69.cpp
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
 * You have to provide your own functions for delay_ms() and mstimer_get().
 * Use the SysTick timer (for example) with a 1 ms resolution which is present on all ARM controllers.
 *
 * If you want to port this library to other devices, you have to provide an SPI instance
 * derived from the SPIBase class.
 */

/** @addtogroup RFM69
 * @{
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "rfm69.h"
#include "delay.h"

#define TIMEOUT_MODE_READY    100 ///< Maximum amount of time until mode switch [ms]
#define TIMEOUT_PACKET_SENT   100 ///< Maximum amount of time until packet must be sent [ms]
#define TIMEOUT_CSMA_READY    500 ///< Maximum CSMA wait time for channel free detection [ms]
#define CSMA_RSSI_THRESHOLD   -85 ///< If RSSI value is smaller than this, consider channel as free [dBm]

/** RFM69 base configuration after init().
 *
 * Change these to your needs or call setCustomConfig() after module init.
 */
static const uint8_t rfm69_base_config[][2] =
{
    {0x01, 0x04}, // RegOpMode: Standby Mode
    {0x02, 0x00}, // RegDataModul: Packet mode, FSK, no shaping
    {0x03, 0x02}, // RegBitrateMsb: 4.8 kbps
    {0x04, 0x40}, // RegBitrateLsb
    {0x05, 0x03}, // RegFdevMsb: 5 kHz
    {0x06, 0x33}, // RegFdevLsb
    {0x07, 0xE4}, // RegFrfMsb: 915 MHz 0xE4C000  868MHz 0xD90000
    {0x08, 0xC0}, // RegFrfMid
    {0x09, 0x00}, // RegFrfLsb
    {0x11, 0x7F},
    {0x13, 0x1A},
    {0x18, 0x88}, // RegLNA: 200 Ohm impedance, gain set by AGC loop
    {0x19, 0x42}, // RegRxBw: 10 kHz

    { 0x25, 0x40 }, // DIO0 is the only IRQ we're using
    { 0x26, 0x07 }, // DIO5 ClkOut disable for power saving
    ///* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    ///* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm

    {0x2C, 0x00}, // RegPreambleMsb: 3 bytes preamble
    {0x2D, 0x03}, // RegPreambleLsb
    {0x2E, 0x88}, // RegSyncConfig: Enable sync word, 2 bytes sync word
    {0x2F, 0x2D}, // RegSyncValue1: 0x2D48
    {0x30, 0x01}, // RegSyncValue2
    {0x37, 0xD0}, // RegPacketConfig1: Variable length, CRC off, whitening
    {0x38, 0x40}, // RegPayloadLength: 64 bytes max payload
    {0x3C, 0x8F}, // RegFifoThresh: TxStart on FifoNotEmpty, 2 bytes FifoLevel
    {0x3D, 0x12},
    {0x5A, 0x5D}, // PA Settings
    {0x5C, 0x7C}, // PA Settings
    {0x58, 0x1B}, // RegTestLna: Normal sensitivity mode
    {0x6F, 0x30}, // RegTestDagc: Improved margin, use if AfcLowBetaOn=0 (default)

 /*
    {0x01, 0x04}, // RegOpMode: Standby Mode
    {0x02, 0x00}, // RegDataModul: Packet mode, FSK, no shaping
    {0x03, 0x02}, // RegBitrateMsb: 4.8 kbps
    {0x04, 0x40}, // RegBitrateLsb
    {0x05, 0x00}, // RegFdevMsb: 5 kHz
    {0x06, 0x52}, // RegFdevLsb
    {0x07, 0xE4}, // RegFrfMsb: 915 MHz 0xE4C000  868MHz 0xD90000
    {0x08, 0xC0}, // RegFrfMid
    {0x09, 0x00}, // RegFrfLsb
    {0x11, 0x7F},
    {0x13, 0x1A},
    {0x18, 0x88}, // RegLNA: 200 Ohm impedance, gain set by AGC loop
    {0x19, 0x42}, // RegRxBw: 10 kHz
    {0x2C, 0x00}, // RegPreambleMsb: 3 bytes preamble
    {0x2D, 0x03}, // RegPreambleLsb
    {0x2E, 0x88}, // RegSyncConfig: Enable sync word, 2 bytes sync word
    {0x2F, 0x2D}, // RegSyncValue1: 0x2D48
    {0x30, 0x01}, // RegSyncValue2
    {0x37, 0xC0}, // RegPacketConfig1: Variable length, CRC off, whitening
    {0x38, 0x40}, // RegPayloadLength: 64 bytes max payload
    {0x3C, 0x8F}, // RegFifoThresh: TxStart on FifoNotEmpty, 2 bytes FifoLevel
    {0x3D, 0x12},
    {0x5A, 0x5D}, // PA Settings
    {0x5C, 0x7C}, // PA Settings
    {0x58, 0x1B}, // RegTestLna: Normal sensitivity mode
    {0x6F, 0x30}, // RegTestDagc: Improved margin, use if AfcLowBetaOn=0 (default)
    */
};

// Clock constants. DO NOT CHANGE THESE!
#define RFM69_XO               32000000    ///< Internal clock frequency [Hz]
#define RFM69_FSTEP            61.03515625 ///< Step width of synthesizer [Hz]

/**
 * Initialize the RFM69 module.
 * A base configuration is set and the module is put in standby mode.
 *
 * @return Always true
 */
bool RFM69_init(RFM69_Struct* rfm69, bool highPowerDevice)
{
  rfm69->mode = RFM69_MODE_STANDBY;
  rfm69->highPowerDevice = highPowerDevice;
  rfm69->powerLevel = 15;
  rfm69->rssi = -127;
  rfm69->ookEnabled = false;
  rfm69->autoReadRSSI = false;
  rfm69->dataMode = RFM69_DATA_MODE_PACKET;
  rfm69->highPowerSettings = true;
  rfm69->csmaEnabled = false;
  rfm69->rxBufferLength = 0;

  // Set base configuration
  RFM69_setCustomConfig(rfm69, rfm69_base_config, sizeof(rfm69_base_config) / 2);

    RFM69_writeRegister(rfm69, 0x13,0x1A);
    RFM69_writeRegister(rfm69, 0x11, 0x7F);

  // Set PA and OCP settings according to RF module (normal/high power)
  //RFM69_setPASettings(rfm69, 0x60);

  // Set output power
  //RFM69_setPowerLevel(rfm69, rfm69->powerLevel);
  //RFM69_setPowerDBm(rfm69, 20);
  //RFM69_setHighPowerSettings(rfm69, rfm69->highPowerDevice);



  // Enable/Disable CSMA/CA algorithm
  //RFM69_setCSMA(rfm69, rfm69->csmaEnabled);

  // Clear FIFO and flags
  RFM69_clearFIFO(rfm69);

  return(0);
}


//************************************
// Gets the last "cached" RSSI reading
//************************************
int RFM69_getRSSI(RFM69_Struct* rfm69)
{
    return rfm69->rssi;
}

//*******************************************************************************
// Enable/disable the automatic reading of the RSSI value during packet reception
//*******************************************************************************
void RFM69_setAutoReadRSSI(RFM69_Struct* rfm69, bool enable)
{
    rfm69->autoReadRSSI = enable;
}

//*****************************************************************************
// Enable/disable the CSMA/CA (carrier sense) algorithm before sending a packet
//*****************************************************************************
void RFM69_setCSMA(RFM69_Struct* rfm69, bool enable)
{
    rfm69->csmaEnabled = enable;
}


//*****************************************************
// Reset the RFM69 module using the external reset line
//*****************************************************
void RFM69_reset(RFM69_Struct* rfm69)
{
  // Generate reset impulse
  GPIO_SetBits(RFM69_RESET_GPIO, RFM69_RESET_PIN);
  delay_ms(1);
  GPIO_ResetBits(RFM69_RESET_GPIO, RFM69_RESET_PIN);

  // Wait until module is ready
  delay_ms(10);

  rfm69->mode = RFM69_MODE_STANDBY;
}


//***********************************************************
// Set the carrier frequency in Hz.
// After calling this function, the module is in standby mode
//***********************************************************
void RFM69_setFrequency(RFM69_Struct* rfm69, unsigned int frequency)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == rfm69->mode || RFM69_MODE_TX == rfm69->mode)
  {
      RFM69_setMode(rfm69, RFM69_MODE_STANDBY);
  }

  // calculate register value
  frequency /= RFM69_FSTEP;

  // set new frequency
  RFM69_writeRegister(rfm69, 0x07, frequency >> 16);
  RFM69_writeRegister(rfm69, 0x08, frequency >> 8);
  RFM69_writeRegister(rfm69, 0x09, frequency);
}

//***********************************************************
// Set the FSK frequency deviation in Hz.
// After calling this function, the module is in standby mode
 //**********************************************************
void RFM69_setFrequencyDeviation(RFM69_Struct* rfm69, unsigned int frequency)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == rfm69->mode || RFM69_MODE_TX == rfm69->mode)
  {
      RFM69_setMode(rfm69, RFM69_MODE_STANDBY);
  }

  // calculate register value
  frequency /= RFM69_FSTEP;

  // set new frequency
  RFM69_writeRegister(rfm69, 0x05, frequency >> 8);
  RFM69_writeRegister(rfm69, 0x06, frequency);
}

//***********************************************************
// Set the bitrate in bits per second.
// After calling this function, the module is in standby mode
//***********************************************************
void RFM69_setBitrate(RFM69_Struct* rfm69, unsigned int bitrate)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == rfm69->mode || RFM69_MODE_TX == rfm69->mode)
  {
      RFM69_setMode(rfm69, RFM69_MODE_STANDBY);
  }

  // calculate register value
  bitrate = RFM69_XO / bitrate;

  // set new bitrate
  RFM69_writeRegister(rfm69, 0x03, bitrate >> 8);
  RFM69_writeRegister(rfm69, 0x04, bitrate);
}

//****************************
// Read a RFM69 register value
//****************************
uint8_t RFM69_readRegister(RFM69_Struct* rfm69, uint8_t reg)
{
  uint8_t data=0;
  //Ensure the register is within range
  if (reg > 0x7f)
    return 0;

  //Read value from register
  RFM69_chipSelect();

  data = SPI_read_byte(reg);

  //Read value from register
  RFM69_chipUnselect();

  return (data);
}

//*****************************
// Write a RFM69 register value
//*****************************
void RFM69_writeRegister(RFM69_Struct* rfm69, uint8_t reg, uint8_t value)
{
  //Ensure the register is within range
  if (reg > 0x7f)
    return;

  //Transfer value to register and set the write flag
  RFM69_chipSelect();

  SPI_write_byte(reg | 0x80);            //Set the MSB to 1 to indicate a write command to register "reg"
  SPI_write_byte(value);

  RFM69_chipUnselect();
}

//************************
// Set the chip select pin
//************************
void RFM69_chipSelect()
{
    RFM69_CS_GPIO->BRR = RFM69_CS_PIN;
}

//***************************
// Release the chip elect pin
//***************************
void RFM69_chipUnselect(RFM69_Struct* rfm69)
{
    RFM69_CS_GPIO->BSRR = RFM69_CS_PIN;
}

//****************************************************************************
// Switch the mode of the RFM69 module.
// Using this function you can manually select the RFM69 mode (sleep for example).
//
// This function also takes care of the special registers that need to be set when
// the RFM69 module is a high power device (RFM69Hxx).
//
// This function is usually not needed because the library handles mode changes automatically.
//
// @param mode RFM69_MODE_SLEEP, RFM69_MODE_STANDBY, RFM69_MODE_FS, RFM69_MODE_TX, RFM69_MODE_RX
// @return The new mode
//*****************************
RFM69Mode RFM69_setMode(RFM69_Struct* rfm69, RFM69Mode mode)
{
  if ((rfm69->mode == mode) || (mode > RFM69_MODE_RX))
  {
    return rfm69->mode;
  }

  // Set new mode
  RFM69_writeRegister(rfm69, 0x01, mode << 2);

  // Set special registers if this is a high power device (RFM69HW)
  if (rfm69->highPowerDevice == true)
  {
    switch (mode)
    {
        case RFM69_MODE_RX:
              // normal RX mode
              if (true == rfm69->highPowerSettings)
              {
                RFM69_setHighPowerSettings(rfm69, false);
              }
          break;

        case RFM69_MODE_TX:
              // +20dBm operation on PA_BOOST
              if (true == rfm69->highPowerSettings)
              {
                RFM69_setHighPowerSettings(rfm69, true);
              }
          break;

        default:
          break;
    }
  }

  rfm69->mode = mode;

  return rfm69->mode;
}


//*****************************************************************************************
// Enable/disable the power amplifier(s) of the RFM69 module.
//
// PA0 for regular devices is enabled and PA1 is used for high power devices (default).
//
// @note Use this function if you want to manually override the PA settings.
// @note PA0 can only be used with regular devices (not the high power ones!)
// @note PA1 and PA2 can only be used with high power devices (not the regular ones!)
//
// @param forcePA If this is 0, default values are used. Otherwise, PA settings are forced.
//0x01 for PA0, 0x02 for PA1, 0x04 for PA2, 0x08 for +20 dBm high power settings.
//*****************************************************************************************
void RFM69_setPASettings(RFM69_Struct* rfm69, uint8_t forcePA)
{
  // Disable OCP for high power devices, enable otherwise
  if(rfm69->highPowerDevice)
  {
        RFM69_writeRegister(rfm69, 0x13, 0x1A);
  }
  else
  {
        RFM69_writeRegister(rfm69, 0x13, 0x0A);
  }

  if (forcePA == 0)
  {
    if(rfm69->highPowerDevice == true)
    {
      // enable PA1 only
      RFM69_writeRegister(rfm69, 0x11, (RFM69_readRegister(rfm69, 0x11) & 0x1F) | 0x40);
    }
    else
    {
      // enable PA0 only
      RFM69_writeRegister(rfm69, 0x11, (RFM69_readRegister(rfm69, 0x11) & 0x1F) | 0x80);
    }
  }
  else
  {
    // PA settings forced
    uint8_t pa = 0;

    if (forcePA & 0x01)
      pa |= 0x80;

    if (forcePA & 0x02)
      pa |= 0x40;

    if (forcePA & 0x04)
      pa |= 0x20;

    // check if high power settings are forced
    rfm69->highPowerSettings = (forcePA & 0x08) ? true : false;
    RFM69_setHighPowerSettings(rfm69, rfm69->highPowerSettings);

    RFM69_writeRegister(rfm69, 0x11, (RFM69_readRegister(rfm69, 0x11) & 0x1F) | pa);
  }
}

//***********************************************
// Set the output power level of the RFM69 module
//
// Power level from 0 to 31.
//***********************************************
void RFM69_setPowerLevel(RFM69_Struct* rfm69, uint8_t power)
{
  if (power > 31)
    power = 31;

  RFM69_writeRegister(rfm69, 0x11, (RFM69_readRegister(rfm69, 0x11) & 0xE0) | power);

  rfm69->powerLevel = power;
}

//************************************************************
// Enable the +20 dBm high power settings of RFM69Hxx modules.
// Enabling only works with high power devices.
//************************************************************
void RFM69_setHighPowerSettings(RFM69_Struct* rfm69, bool enable)
{
  //Enabling only works if this is a high power device
  if (enable == true && rfm69->highPowerDevice == false)
  {
    enable = false;
  }

  RFM69_writeRegister(rfm69, 0x5A, enable ? 0x5D : 0x55);
  RFM69_writeRegister(rfm69, 0x5C, enable ? 0x7C : 0x70);
}

//************************************************************
// Reconfigure the RFM69 module by writing multiple registers at once
// @param config Array of register/value tuples
// @param length Number of elements in config array
//************************************************************
void RFM69_setCustomConfig(RFM69_Struct* rfm69, const uint8_t config[][2], unsigned int length)
{
  for (unsigned int i = 0; i < length; i++)
  {
    RFM69_writeRegister(rfm69, config[i][0], config[i][1]);
    simple_delay(1000);
  }
}

//************************************************************
// Send a packet over the air.
//
// After sending the packet, the module goes to standby mode.
// CSMA/CA is used before sending if enabled by function setCSMA() (default: off).
//
// @note A maximum amount of RFM69_MAX_PAYLOAD bytes can be sent.
// @note This function blocks until packet has been sent.
//
// @param data Pointer to buffer with data
// @param dataLength Size of buffer
//
// @return Number of bytes that have been sent
//************************************************************
int RFM69_send(RFM69_Struct* rfm69, const void* data, unsigned int dataLength)
{
  // switch to standby and wait for mode ready, if not in sleep mode
  if (RFM69_MODE_SLEEP != rfm69->mode)
  {
    RFM69_setMode(rfm69, RFM69_MODE_STANDBY);
    RFM69_waitForModeReady(rfm69);
  }

  // clear FIFO to remove old data and clear flags
  RFM69_clearFIFO(rfm69);

  // limit max payload
  if (dataLength > RFM69_MAX_PAYLOAD)
    dataLength = RFM69_MAX_PAYLOAD;

  // payload must be available
  if (0 == dataLength)
    return 0;

  // Wait for a free channel, if CSMA/CA algorithm is enabled.
  //This takes around 1,4 ms to finish if channel is free */
  if (true == rfm69->csmaEnabled)
  {
    // Restart RX
    RFM69_writeRegister(rfm69, 0x3D, (RFM69_readRegister(rfm69, 0x3D) & 0xFB) | 0x20);

    // switch to RX mode
    RFM69_setMode(rfm69, RFM69_MODE_RX);

    // wait until RSSI sampling is done; otherwise, 0xFF (-127 dBm) is read

    // RSSI sampling phase takes ~960 µs after switch from standby to RX
    uint32_t timeEntry = mstimer_get();
    while (((RFM69_readRegister(rfm69, 0x23) & 0x02) == 0) && ((mstimer_get() - timeEntry) < 10));

    while ((false == RFM69_channelFree(rfm69)) && ((mstimer_get() - timeEntry) < TIMEOUT_CSMA_READY))
    {
      // wait for a random time before checking again
      delay_ms(rand() % 10);

      /* try to receive packets while waiting for a free channel
       * and put them into a temporary buffer */
      int bytesRead;
      if ((bytesRead = RFM69_receive(rfm69, rfm69->rxBuffer, RFM69_MAX_PAYLOAD)) > 0)
      {
        rfm69->rxBufferLength = bytesRead;

        // module is in RX mode again

        // Restart RX and wait until RSSI sampling is done
        RFM69_writeRegister(rfm69, 0x3D, (RFM69_readRegister(rfm69, 0x3D) & 0xFB) | 0x20);
        uint32_t timeEntry = mstimer_get();
        while (((RFM69_readRegister(rfm69, 0x23) & 0x02) == 0) && ((mstimer_get() - timeEntry) < 10));
      }
    }

    RFM69_setMode(rfm69, RFM69_MODE_STANDBY);
  }

  // transfer packet to FIFO
  RFM69_chipSelect();

  // address FIFO
  SPI_write_byte(0x00 | 0x80);

  // send length byte
  SPI_write_byte(dataLength);

   //These should be added and the datalength should be changed accordingly
  //SPI_transfer_byte(destination);
  //SPI_transfer_byte(source);
  //SPI_transfer_byte(control);

  // send payload
  for (unsigned int i = 0; i < dataLength; i++)
    SPI_write_byte(((uint8_t*)data)[i]);

  RFM69_chipUnselect(rfm69);

  // start radio transmission
  RFM69_setMode(rfm69, RFM69_MODE_TX);

  // wait for packet sent
  RFM69_waitForPacketSent(rfm69);

  // go to standby
  RFM69_setMode(rfm69, RFM69_MODE_STANDBY);

  return dataLength;
}

//*************************************
// Clear FIFO and flags of RFM69 module
//*************************************
void RFM69_clearFIFO(RFM69_Struct* rfm69)
{
  // clear flags and FIFO
  RFM69_writeRegister(rfm69, 0x28, 0x10);
}

//******************************************************
// Wait until the requested mode is available or timeout
//******************************************************
void RFM69_waitForModeReady(RFM69_Struct* rfm69)
{
  uint32_t timeEntry = mstimer_get();

  while (((RFM69_readRegister(rfm69, 0x27) & 0x80) == 0) && ((mstimer_get() - timeEntry) < TIMEOUT_MODE_READY));
}

//*********************************************************
// Put the RFM69 module to sleep (lowest power consumption)
//*********************************************************
void RFM69_sleep(RFM69_Struct* rfm69)
{
  RFM69_setMode(rfm69, RFM69_MODE_SLEEP);
}

//************************************************************
// Put the RFM69 module in RX mode and try to receive a packet.
//
// @note The module resides in RX mode.
//
// @param data Pointer to a receiving buffer
// @param dataLength Maximum size of buffer
// @return Number of received bytes; 0 if no payload is available.
//************************************************************
int RFM69_receive(RFM69_Struct* rfm69, char* data, unsigned int dataLength)
{
  // check if there is a packet in the internal buffer and copy it
  if (rfm69->rxBufferLength > 0)
  {
    // copy only until dataLength, even if packet in local buffer is actually larger
    memcpy(data, rfm69->rxBuffer, dataLength);

    unsigned int bytesRead = rfm69->rxBufferLength;

    // empty local buffer
    rfm69->rxBufferLength = 0;

    return bytesRead;
  }
  else
  {
    // regular receive
    // go to RX mode if not already in this mode
    if (RFM69_MODE_RX != rfm69->mode)
    {
        RFM69_setMode(rfm69, RFM69_MODE_RX);
        RFM69_waitForModeReady(rfm69);
    }

    // check for flag PayloadReady
    if (RFM69_readRegister(rfm69, 0x28) & 0x04)
    {
        // go to standby before reading data
        RFM69_setMode(rfm69, RFM69_MODE_STANDBY);

        // get FIFO content
        unsigned int bytesRead = 0;

        // read until FIFO is empty or buffer length exceeded
        while ((RFM69_readRegister(rfm69, 0x28) & 0x40) && (bytesRead < dataLength))
        {
            // read next byte
            data[bytesRead] = RFM69_readRegister(rfm69, 0x00);
            bytesRead++;
        }

        // automatically read RSSI if requested
        if (true == rfm69->autoReadRSSI)
        {
            RFM69_readRSSI(rfm69);
        }

        // go back to RX mode
        RFM69_setMode(rfm69, RFM69_MODE_RX);
        // todo: wait needed?
        //		waitForModeReady();

        return bytesRead;
    }
    else
    {
        return 0;
    }
  }
}

//***************************************************************************
// Enable and set or disable AES hardware encryption/decryption.
//
// The AES encryption module will be disabled if an invalid key or key length
// is passed to this function (aesKey = 0 or keyLength != 16).
// Otherwise encryption will be enabled.
//
// The key is stored as MSB first in the RF module.
//
// @param aesKey Pointer to a buffer with the 16 byte AES key
// @param keyLength Number of bytes in buffer aesKey; must be 16 bytes
// @return State of encryption module (false = disabled; true = enabled)
//***************************************************************************
bool RFM69_setAESEncryption(RFM69_Struct* rfm69, const void* aesKey, unsigned int keyLength)
{
  bool enable = false;

  // check if encryption shall be enabled or disabled
  if ((0 != aesKey) && (16 == keyLength))
    enable = true;

  // switch to standby
  RFM69_setMode(rfm69, RFM69_MODE_STANDBY);

  if (true == enable)
  {
    // transfer AES key to AES key register
    RFM69_chipSelect(rfm69);

    // address first AES MSB register
    SPI_write_byte(0x3E | 0x80);

    // transfer key (0x3E..0x4D)
    for (unsigned int i = 0; i < keyLength; i++)
      SPI_write_byte(((uint8_t*)aesKey)[i]);

    RFM69_chipUnselect(rfm69);
  }

  // set/reset AesOn Bit in packet config
  RFM69_writeRegister(rfm69, 0x3D, (RFM69_readRegister(rfm69, 0x3D) & 0xFE) | (enable ? 1 : 0));

  return enable;
}

//********************************************************
// Wait until packet has been sent over the air or timeout
//********************************************************
void RFM69_waitForPacketSent(RFM69_Struct* rfm69)
{
  uint32_t timeEntry = mstimer_get();

  while (((RFM69_readRegister(rfm69, 0x28) & 0x08) == 0) && ((mstimer_get() - timeEntry) < TIMEOUT_PACKET_SENT));
}

//***************************************************************************
// Transmit a high or low bit in continuous mode using the external data line
//
// @note Use setDataPin() before calling this function.
// @note Call setDataMode() before to enable continuous mode.
//
// @param bit true: high bit; false: low bit
//***************************************************************************
/*
void RFM69_continuousBit(RFM69_Struct* rfm69, bool bit)
{
  // only allow this in continuous mode and if data pin was specified
  if ((RFM69_DATA_MODE_PACKET == rfm69->dataMode) || (0 == rfm69->dataGPIO))
    return;

  // send low or high bit
  if (false == bit)
    GPIO_ResetBits(rfm69->dataGPIO, rfm69->dataPin);
  else
    GPIO_SetBits(rfm69->dataGPIO, rfm69->dataPin);
}
*/
//*******************************************************************************************
// Read the last RSSI value.
//
// @note Only if the last RSSI value was above the RSSI threshold, a sample can be read.
// Otherwise, you always get -127 dBm. Be also careful if you just switched to RX mode
// You may have to wait until a RSSI sample is available.
//
// @return RSSI value in dBm.
//*******************************************************************************************
int RFM69_readRSSI(RFM69_Struct* rfm69)
{
  rfm69->rssi = -RFM69_readRegister(rfm69, 0x24) / 2;

  return rfm69->rssi;
}

//************************************************************
// Debug function to dump all RFM69 registers.
//
// Symbol 'DEBUG' has to be defined.
//************************************************************
void RFM69_dumpRegisters(RFM69_Struct* rfm69)
{
#ifdef DEBUG
  for (unsigned int i = 1; i <= 0x71; i++)
  {
    printf("[0x%X]: 0x%X\n", i, RFM69_readRegister(rfm69, i));
  }
#endif
}

//************************************************************
// Enable/disable OOK modulation (On-Off-Keying).
//
// Default modulation is FSK.
// The module is switched to standby mode if RX or TX was active.
//
// @param enable true or false
//************************************************************
void RFM69_setOOKMode(RFM69_Struct* rfm69, bool enable)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == rfm69->mode || RFM69_MODE_TX == rfm69->mode)
    RFM69_setMode(rfm69, RFM69_MODE_STANDBY);

  if (false == enable)
  {
    // FSK
    RFM69_writeRegister(rfm69, 0x02, (RFM69_readRegister(rfm69, 0x02) & 0xE7));
  }
  else
  {
    // OOK
    RFM69_writeRegister(rfm69, 0x02, (RFM69_readRegister(rfm69, 0x02) & 0xE7) | 0x08);
  }

  rfm69->ookEnabled = enable;
}

//************************************************************
// Configure the data mode of the RFM69 module.
//
// Default data mode is 'packet'. You can choose between 'packet',
// 'continuous with clock recovery', 'continuous without clock recovery'.
//
// The module is switched to standby mode if RX or TX was active.
//
// @param dataMode RFM69_DATA_MODE_PACKET, RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC, RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC
//************************************************************
void RFM69_setDataMode(RFM69_Struct* rfm69, RFM69DataMode dataMode)
{
  // switch to standby if TX/RX was active
  if (RFM69_MODE_RX == rfm69->mode || RFM69_MODE_TX == rfm69->mode)
    RFM69_setMode(rfm69, RFM69_MODE_STANDBY);

  switch (dataMode)
  {
  case RFM69_DATA_MODE_PACKET:
    RFM69_writeRegister(rfm69, 0x02, (RFM69_readRegister(rfm69, 0x02) & 0x1F));
    break;

  case RFM69_DATA_MODE_CONTINUOUS_WITH_SYNC:
    RFM69_writeRegister(rfm69, 0x02, (RFM69_readRegister(rfm69, 0x02) & 0x1F) | 0x40);
    RFM69_writeRegister(rfm69, 0x25, 0x04); // Dio2Mapping = 01 (Data)
    RFM69_continuousBit(rfm69, false);
    break;

  case RFM69_DATA_MODE_CONTINUOUS_WITHOUT_SYNC:
    RFM69_writeRegister(rfm69, 0x02, (RFM69_readRegister(rfm69, 0x02) & 0x1F) | 0x60);
    RFM69_writeRegister(rfm69, 0x25, 0x04); // Dio2Mapping = 01 (Data)
    RFM69_continuousBit(rfm69, false);
    break;

  default:
    return;
  }

  rfm69->dataMode = dataMode;
}

//************************************************************
// Set the output power level in dBm.
//
// This function takes care of the different PA settings of the modules.
// Depending on the requested power output setting and the available module,
// PA0, PA1 or PA1+PA2 is enabled.
//
// @param dBm Output power in dBm
// @return 0 if dBm valid; else -1.
//************************************************************
int RFM69_setPowerDBm(RFM69_Struct* rfm69, int8_t dBm)
{
  /* Output power of module is from -18 dBm to +13 dBm
   * in "low" power devices, -2 dBm to +20 dBm in high power devices */
  if (dBm < -18 || dBm > 20)
    return -1;

  if (false == rfm69->highPowerDevice && dBm > 13)
    return -1;

  if (true == rfm69->highPowerDevice && dBm < -2)
    return -1;

  uint8_t powerLevel = 0;

  if (false == rfm69->highPowerDevice)
  {
    // only PA0 can be used
    powerLevel = dBm + 18;

    // enable PA0 only
    RFM69_writeRegister(rfm69, 0x11, 0x80 | powerLevel);
  }
  else
  {
    if (dBm >= -2 && dBm <= 13)
    {
      // use PA1 on pin PA_BOOST
      powerLevel = dBm + 18;

      // enable PA1 only
      RFM69_writeRegister(rfm69, 0x11, 0x40 | powerLevel);

      // disable high power settings
      rfm69->highPowerSettings = false;
      RFM69_setHighPowerSettings(rfm69, rfm69->highPowerSettings);
    }
    else if (dBm > 13 && dBm <= 17)
    {
      // use PA1 and PA2 combined on pin PA_BOOST
      powerLevel = dBm + 14;

      // enable PA1+PA2
      RFM69_writeRegister(rfm69, 0x11, 0x60 | powerLevel);

      // disable high power settings
      rfm69->highPowerSettings = false;
      RFM69_setHighPowerSettings(rfm69, rfm69->highPowerSettings);
    }
    else
    {
      // output power from 18 dBm to 20 dBm, use PA1+PA2 with high power settings
      powerLevel = dBm + 11;

      // enable PA1+PA2
      RFM69_writeRegister(rfm69, 0x11, 0x60 | powerLevel);

      // enable high power settings
      rfm69->highPowerSettings = true;
      RFM69_setHighPowerSettings(rfm69, rfm69->highPowerSettings);
    }
  }

  return 0;
}

//*************************************************************
// Check if the channel is free using RSSI measurements.
//
// This function is part of the CSMA/CA algorithm.
//
// @return true = channel free; otherwise false.
//************************************************************
bool RFM69_channelFree(RFM69_Struct* rfm69)
{
  if (RFM69_readRSSI(rfm69) < CSMA_RSSI_THRESHOLD)
  {
    return true;
  }
  else
  {
    return false;
  }
}
