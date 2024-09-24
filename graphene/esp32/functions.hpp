#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include "Arduino.h"
#include "Wire.h"

#define I2C_READ_BYTES 1
#define I2C_STOP_END true

#define ON_CMD 0x11
#define OFF_CMD 0x00
#define I2C_TEST_DELAY 2000

#define START_ADDRESS 8
#define END_ADDRESS 13 // Inclusive
#define INPUT_A_PRESSED 0xAA
#define INPUT_B_PRESSED 0x55



/// @brief Sends a CMD with data to the MP3 Player
/// @param SerialComm The serial port to communicate with the mp3 module 
/// @param cmd One of the defined cmd for the player
/// @param data Data associated with the command
void sendMP3CMD(Stream &SerialComm, uint8_t cmd, uint16_t data);


/// @brief Sends a command to the specified i2c-address
/// @param WireComm The wire i2c instance 
/// @param address The submodule address
/// @param cmd The command to send
/// @return The return code for the operation
uint8_t sendWireCMD(TwoWire &WireComm, uint8_t address, uint8_t cmd);

/// @brief Test all the devices, by sending a ON command followed
/// by an OFF command. This tests doesn't verify if the secondary
/// devices respond to the commands. Useful for checking
/// general connections
/// @param WireComm The wire i2c communication bus
/// @param SerialComm The serial port use for debug messages
void generalTest(TwoWire &WireComm, Stream &SerialComm);


/// @brief Performs an strict testing to every submodule. All
/// outputs are set to HIGH and each device should have both
/// buttons pressed in the order of the corresponding addresses
/// from MIN to MAX address.
/// @param WireComm The i2c bus
void strictTest(TwoWire &WireComm);

void getNodeOrder(TwoWire &WireComm, uint8_t addresses, uint8_t length, uint8_t offset);

#endif