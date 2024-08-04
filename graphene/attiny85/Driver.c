#include <avr/io.h>
#include <avr/interrupt.h>
#include "USI_TWI_Slave.h"
#include "functions.h"

// ---------- USI-I2C module Configurations --------------
#define MCU_ADDRESS 0x08
// ---------------- Pinout Configurations ----------------
#define INPUT_PIN_A PB3
#define INPUT_PIN_B PB4
#define OUTPUT_PIN PB1
// -------------- States and readed values ---------------
#define INPUT_A_PRESSED_REG 0xAA
#define INPUT_B_PRESSED_REG 0x55
#define LOCKED 0xFF
#define UNLOCKED 0x00
// -> Debounce button states and button lock
ButtonStates input_state_a = BUTTON_IDLE;
ButtonStates input_state_b = BUTTON_IDLE;
uint32_t button_debounce_a = 0U;
uint32_t button_debounce_b = 0U;
uint8_t button_lock = 0x00;
static uint8_t pressed_reg = 0x00;
// -> USI vars
static uint8_t cmd = 0x00;
uint8_t data_amount = 0;
// -> Flags
static uint8_t timer_flag = 0x00;
static uint8_t clear_flag = 0x00;

// ---------------- Functions declarations --------------
void TX(void);

int main()
{
    // Setup I/O
    setupPorts(OUTPUT_PIN, INPUT_PIN_A, INPUT_PIN_B, 0x01);
    // Setup Timer0
    setupTimer0();
    // Setup USI
    usi_onRequestPtr = TX; // Callaback for TX operations
    usiTwiSlaveInit(MCU_ADDRESS);
    // Initialize button states
    input_state_a = BUTTON_IDLE;
    input_state_b = BUTTON_IDLE;
    button_debounce_a = 0U;
    button_debounce_b = 0U;
    button_lock = 0x00;
    pressed_reg = 0x00;
    // Initialize USI variables
    cmd = 0;
    data_amount = 0;
    // Flags
    timer_flag = 0x00;
    clear_flag = 0x00;
    // Enable global interrupts
    sei();
    // Main loop
    while (1)
    {
        if (clear_flag == 0xff)
        {
            // Clear the button registry and flag
            pressed_reg = 0x00;
            clear_flag = 0x00;
            button_lock = UNLOCKED;
            input_state_a = BUTTON_IDLE;
            input_state_b = BUTTON_IDLE;
        }

        // Update button register
        if (button_lock == UNLOCKED)
        {
            // Check for button press
            input_state_a = readButtonDebounce(&button_debounce_a, INPUT_PIN_A);
            input_state_b = readButtonDebounce(&button_debounce_b, INPUT_PIN_B);

            if (input_state_a == BUTTON_PRESS)
            {
                pressed_reg = INPUT_A_PRESSED_REG;
                button_lock = LOCKED;
                startTimer0();
            }

            if (input_state_b == BUTTON_PRESS)
            {
                pressed_reg = INPUT_B_PRESSED_REG;
                button_lock = LOCKED;
                startTimer0();
            }
        }
        // If waited to long for USI-READ -> Clear the button reg and unlock
        if (timer_flag)
        {
            stopTimer0();
            pressed_reg = 0x00;
            button_lock = UNLOCKED;
            timer_flag = 0x00;
            input_state_a = BUTTON_IDLE;
            input_state_b = BUTTON_IDLE;
        }

        // Check if CMD is available and update it
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

void TX()
{
    usiTwiTransmitByte(pressed_reg);
    // clear the pressed register
    clear_flag = 0xff;
}

ISR(TIM0_OVF_vect)
{
    timer_flag = 0xFF;
}