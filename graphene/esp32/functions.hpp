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
#define NUMBER_OF_NODES 6


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

/// @brief Performs an ordering routine for the available nodes. Waits for the user to press the button to indicate the correct order sequence
/// @param WireComm The I2C-Wire object
/// @param nodes The node array pointer to modify and return
/// @param length The number of nodes available
/// @param offset The offset address. The first node start at address=8, so the offset will be 8
/// @param start The start address
/// @param end The end address
void getNodeOrder(TwoWire &WireComm, uint8_t* nodes, uint8_t length, uint8_t offset, uint8_t start=START_ADDRESS, uint8_t end=END_ADDRESS);

/// @brief Return the next index on the ordered node array
/// @param current_node The current node index
/// @param length The number of available nodes
/// @return The next index of the array 
uint8_t nextNode(uint8_t current_node, uint8_t length=NUMBER_OF_NODES);

/// @brief Return the previous index on the ordered node array
/// @param current_node The current node index
/// @param length The number of available nodes
/// @return The previous index of the array
uint8_t prevNode(uint8_t current_node, uint8_t length=NUMBER_OF_NODES);

#endif