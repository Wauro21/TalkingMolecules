#include "functions.h"

void setupPorts(uint8_t out, uint8_t in_a, uint8_t in_b, uint8_t initial)
{
    DDRB |= _BV(out);                 // enable out as output
    DDRB &= ~(_BV(in_a) | _BV(in_b)); // set in_a and in_b as inputs
    PORTB |= _BV(in_a) | _BV(in_b);   // Enable pull-up for in_a and in_b
    if (initial)
    {
        PORTB |= _BV(out); // set as high
    }
    else
    {
        PORTB &= ~_BV(out);
    }
}

void setupTimer0(void)
{
    TCCR0A &= ~(_BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0)); // Compare units disconnected
    TCCR0A &= ~(_BV(WGM02) | _BV(WGM01) | _BV(WGM00));                  // Normal mode
}

void startTimer0(void)
{
    TCNT0 = 0;                       // Reset counter
    TIMSK |= _BV(TOIE0);             // Overflow interrupt enable
    TCCR0B |= _BV(CS02) | _BV(CS00); // Clock with /1024 prescaler
}

void stopTimer0(void)
{
    TCCR0B &= ~(_BV(CS02) | _BV(CS00)); // Disconnect clock
    TIMSK &= ~_BV(TOIE0);               // Disable overflow interrupt
    TCNT0 = 0;                          // Clear counter
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