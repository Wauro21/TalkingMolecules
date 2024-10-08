#ifndef FUNCTIONS_HPP
#define FUNCTIONS_HPP

#include <Arduino.h>

// ---------- Software Debouncer Configurations ----------
#define PRESS_BUTTON_MASK 0x80000000
#define HOLD_BUTTON_MASK 0xFFFFFFFF
typedef enum
{
    BUTTON_IDLE = 0x00,
    BUTTON_PRESS = 0x01,
    BUTTON_HOLD = 0x02,
} ButtonStates;


/// @brief Read and debounce the desired button
/// @param debounce_var The state variable for the button
/// @param pin The pin to read and debounce
/// @return The current state of the button
ButtonStates readButtonDebounce(uint32_t *debounce_var, int pin);

/// @brief Sends a CMD with data to the MP3 Player
/// @param SerialComm The serial port used to send commands
/// @param cmd One of the defined cmd for the player
/// @param data Data associated with the command
void sendMP3CMD(Stream &SerialComm, uint8_t cmd, uint16_t data);



#endif