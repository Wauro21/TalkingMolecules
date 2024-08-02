#include <avr/io.h>
#include <avr/interrupt.h>
#include "USI_TWI_Slave.h"

#define LED_STATUS PB4
#define MCU_ADDRESS 0x08
#define INPUT_PIN PB3
#define OUTPUT_PIN PB1

// GENERAL FSM
typedef enum
{
    TEST,
    IDLE,
    USI_START,
    LISTEN,
    HOLD,
    RESET,
} GENERAL_FSM;

GENERAL_FSM current_state = IDLE;

// ---------- Software Debouncer Configurations ----------
#define PRESS_BUTTON_MASK 0x80000000
#define HOLD_BUTTON_MASK 0xFFFFFFFF
typedef enum
{
    BUTTON_IDLE = 0x00,
    BUTTON_PRESS = 0x01,
    BUTTON_HOLD = 0x02,
} ButtonStates;

ButtonStates input_state = BUTTON_IDLE;
uint32_t button_debounce = 0U;
bool output_a = false;

// ---------------- Functions declarations --------------

/// @brief Read and debounce the desired button
/// @param debounce_var The state variable for the button
/// @param pin The pin to read and debounce
/// @return The current state of the button
ButtonStates readButtonDebounce(uint32_t *debounce_var, uint8_t pin);

int main()
{
    DDRB |= _BV(LED_STATUS) | _BV(OUTPUT_PIN);
    DDRB &= ~_BV(INPUT_PIN);  // INPUT
    PORTB &= ~_BV(INPUT_PIN); // INPUT
    PORTB |= _BV(LED_STATUS);
    PORTB &= ~_BV(OUTPUT_PIN);
    // usiTwiSlaveInit(8);
    uint8_t data = 0;
    current_state = IDLE;
    while (1)
    {
        switch (current_state)
        {
        case TEST:
        {
            current_state = IDLE;
        }

        case IDLE:
        {
            // Wait for button press
            input_state = readButtonDebounce(&button_debounce, INPUT_PIN);

            // Check change in state
            if (input_state == BUTTON_PRESS)
            {
                current_state = LISTEN; // Go to USI mode
            }
            break;
        }
        case USI_START:
        {
            current_state = LISTEN;
            output_a = true;
            // Enable USI
            usiTwiSlaveInit(8);
            sei();
        }

        case LISTEN:
        {
            data = usiTwiReceiveByte();
            if (data == 1)
            {
                PORTB |= _BV(LED_STATUS);
            }

            else
            {
                PORTB &= ~_BV(LED_STATUS);
            }
            break;
        }
        }

        // UPDATE OUPUTS
        if (output_a)
        {
            PORTB |= _BV(OUTPUT_PIN);
        }
        else
        {
            PORTB &= ~_BV(OUTPUT_PIN);
        }
    }
}

ButtonStates readButtonDebounce(uint32_t *debounce_var, uint8_t pin)
{
    // Read the current state of the button
    uint8_t read_value = ((~PINB & _BV(pin)) >> pin); // Negative logic --> when button pressed is low, resulting = 1111_1111 | for high = 0000_0001
    // Debounce the value
    *debounce_var = (*debounce_var << 1) | read_value;

    // Check current state
    switch (*debounce_var)
    {
    case PRESS_BUTTON_MASK:
        return BUTTON_PRESS;
    case HOLD_BUTTON_MASK:
        return BUTTON_HOLD;
    default:
        return BUTTON_IDLE;
    }
}