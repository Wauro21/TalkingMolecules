#include <avr/io.h>

#define BUTTON_INPUT PINB2
#define LED_OUTPUT PB0

#define PRESS_BUTTON_MASK 0x80000000
#define HOLD_BUTTON_MASK 0xFFFFFFFF

// Globals
uint8_t out_state = 0;

// -> Button debouncer
uint32_t button_debounce;
uint8_t button_state;

uint8_t readButton(uint32_t *state)
{
    /// The button has negative logic, a press corresponds to a low level
    uint8_t btn_read = (~PINB & _BV(BUTTON_INPUT)) >> BUTTON_INPUT;
    /// The read value in inverted for positive logic and shifted
    /// Read and shift for debouncing
    *state = (*state << 1) | btn_read;
    switch (*state)
    {
    case PRESS_BUTTON_MASK:
        return 0x01;
    case HOLD_BUTTON_MASK:
        return 0x02;
    default:
        return 0x00;
    }
}

void enableTWI()
{
    USICR = 0;
    USICR |= _BV(USICS1); // External clock
    USICR |= _BV(USIWM1); // TWI mode
}

void disableTWI()
{
    USICR &= ~_BV(USIWM1); // DISABLE TWI
}

int main(void)
{
    // Initialize button state
    button_debounce = 0U;
    button_state = 0x00; // IDLE

    // SETUP IO
    DDRB = 0;
    DDRB &= ~_BV(BUTTON_INPUT);
    DDRB |= _BV(LED_OUTPUT);

    // CLEAR OUTPUT VALUES
    PORTB = 0;
    PORTB |= _BV(PB2); // ENABLE PULLUP


    while (1)
    {
        // Read button
        button_state = readButton(&button_debounce);

        if (button_state == 0x01)
        {
            out_state = 0x01; // Enable TWI interface
        }

        // Check TWI status
        if(out_state == 0x01)
        {
            enableTWI();
            out_state = 0x02; // Next state
        }

        // Start listening
        if(out_state == 0x02)
        {

        }
    }
}