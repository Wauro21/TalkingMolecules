#include <avr/io.h>
#include <avr/interrupt.h>
#include "USI_TWI_Slave.h"

// ---------- USI-I2C module Configurations ----------
#define MCU_ADDRESS 0x08
// -------------- PIN OUT Definitions ----------------
#define INPUT_PIN_A PB3
#define INPUT_PIN_B PB4
#define OUTPUT_PIN PB1
// ---------- Software Debouncer Configurations ----------
#define PRESS_BUTTON_MASK 0x80000000
#define HOLD_BUTTON_MASK 0xFFFFFFFF
typedef enum
{
    BUTTON_IDLE = 0x00,
    BUTTON_PRESS = 0x01,
    BUTTON_HOLD = 0x02,
} ButtonStates;

ButtonStates input_state_a = BUTTON_IDLE;
ButtonStates input_state_b = BUTTON_IDLE;
uint32_t button_debounce_a = 0U;
uint32_t button_debounce_b = 0U;
uint8_t button_lock = 0x00;
static uint8_t cmd = 0x00;
static uint8_t pressed_reg = 0x00;
static uint8_t timer_flag = 0x00;
uint8_t data_amount = 0;
static uint8_t clear_flag = 0x00;

// ---------------- Functions declarations --------------

/// @brief Read and debounce the desired button
/// @param debounce_var The state variable for the button
/// @param pin The pin to read and debounce
/// @return The current state of the button
ButtonStates readButtonDebounce(uint32_t *debounce_var, uint8_t pin);

// CALLBACKS
// void RX(uint8_t n_data);
void TX(void);
void startTimer(void);
void stopTimer(void);

int main()
{

    input_state_a = BUTTON_IDLE;
    input_state_b = BUTTON_IDLE;
    DDRB |= _BV(OUTPUT_PIN);
    DDRB &= ~(_BV(INPUT_PIN_A) | _BV(INPUT_PIN_B)); // INPUT
    PORTB |= _BV(INPUT_PIN_A) | _BV(INPUT_PIN_B);   // INPUT PULLUP
    PORTB |= _BV(OUTPUT_PIN);

    // USI Callback
    usi_onRequestPtr = TX;

    // SETUP TIMER
    TCCR0A &= ~(_BV(COM0A1) | _BV(COM0A0) | _BV(COM0B1) | _BV(COM0B0)); // Compare units disconnected
    TCCR0A &= ~(_BV(WGM02) | _BV(WGM01) | _BV(WGM00));                  // Normal mode
    // TCCR0B |= _BV(CS01);                                                // Prescaler 8
    // TIMSK |= _BV(TOIE0); // Overflow interrupt enable

    // USI SETUP
    usiTwiSlaveInit(MCU_ADDRESS);
    sei();

    while (1)
    {
        if (clear_flag == 0xff)
        {
            // Clear the button registry and flag
            pressed_reg = 0x00;
            clear_flag = 0x00;
            button_lock = 0x00;
            input_state_a = BUTTON_IDLE;
            input_state_b = BUTTON_IDLE;
        }

        // Update button register
        if (button_lock == 0x00)
        {
            // Check for button press
            input_state_a = readButtonDebounce(&button_debounce_a, INPUT_PIN_A);
            input_state_b = readButtonDebounce(&button_debounce_b, INPUT_PIN_B);

            if (input_state_a == BUTTON_PRESS)
            {
                // PORTB &= ~_BV(OUTPUT_PIN);
                pressed_reg = 0x80;
                button_lock = 0xFF;
                startTimer();
            }

            if (input_state_b == BUTTON_PRESS)
            {
                pressed_reg = 0x01;
                button_lock = 0xFF;
                startTimer();
            }
        }
        // If wait too long clear the button status
        if (timer_flag)
        {
            stopTimer();
            pressed_reg = 0x00;
            button_lock = 0x00;
            timer_flag = 0x00;
            input_state_a = BUTTON_IDLE;
            input_state_b = BUTTON_IDLE;
        }

        // Check for current CMD
        data_amount = usiTwiAmountDataInReceiveBuffer();
        if (data_amount > 0)
        {
            cmd = usiTwiReceiveByte();
        }

        // Update output status if button pressed
        if (cmd == 0x11)
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

void TX()
{
    usiTwiTransmitByte(pressed_reg);
    // clear the pressed register
    clear_flag = 0xff;
}

void startTimer(void)
{
    TCNT0 = 0;
    TIMSK |= _BV(TOIE0); // Overflow interrupt enable
    TCCR0B |= _BV(CS02) | _BV(CS00);
}

void stopTimer(void)
{
    TCCR0B &= ~(_BV(CS02) | _BV(CS00));
    TIMSK &= ~_BV(TOIE0);
    TCNT0 = 0;
}

ISR(TIM0_OVF_vect)
{
    timer_flag = 0xFF;
}