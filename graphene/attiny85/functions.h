#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <avr/io.h>

#define PRESS_BUTTON_MASK 0x80000000
#define HOLD_BUTTON_MASK 0xFFFFFFFF

/// @brief Possible states for the buttons
typedef enum
{
    BUTTON_IDLE = 0x00,
    BUTTON_PRESS = 0x01,
    BUTTON_HOLD = 0x02,
} ButtonStates;

/// @brief Configures the I/O ports of the device
/// @param out The pin set to be used as an output
/// @param in_a The first input pin - (PULL UP is enable)
/// @param in_b The second input pin - (PULL UP is enable)
/// @param initial If true, the output pin starts as HIGH
void setupPorts(uint8_t out, uint8_t in_a, uint8_t in_b, uint8_t initial);

/// @brief Setups Timer0 as a counter with TOP=0xFF. No compare units
/// @param  void
void setupTimer0(void);

/// @brief Starts Timer0 with a prescaler of 1024 and generating interrupts when counter overflows
/// @param  void
void startTimer0(void);

/// @brief Stops Timer0, clear the counter and disables counter overflow interrupts
/// @param  void
void stopTimer0(void);

/// @brief Read and debounce the desired button
/// @param debounce_var The state variable for the button
/// @param pin The pin to read and debounce
/// @return The current state of the button
ButtonStates readButtonDebounce(uint32_t *debounce_var, uint8_t pin);

#endif