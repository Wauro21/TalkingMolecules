#include <avr/io.h>
#include <avr/interrupt.h>

#define LED_STATUS PB4
#define MCU_ADDRESS 0x08

volatile char led = 0;
volatile char state = 0x00;
volatile char cmd = 0x00;
// 0x00 -> Listening address
// 0x01 -> Writing preparation
// 0x02 -> Read preparation
// 0x03 -> Writing process

int main()
{

    // Set outputs
    DDRB |= _BV(LED_STATUS);

    // Set the output to LOW to start
    // PORTB &= ~_BV(LED_STATUS);
    PORTB |= _BV(LED_STATUS);

    // Initialize i2c
    // -> SET SDA AND SCL as inputs
    DDRB &= ~_BV(PB0); // SDA
    DDRB &= ~_BV(PB2); // SCL
    // -> SET TWI interface register
    USICR = 0;            // Clear the register
    USICR |= _BV(USISIE); // Start condition interrupt enable
    // USICR |= _BV(USIOIE); // Counter overflow interrupt enable -> The counter is used to count the number of bits TX/RX
    USICR |= _BV(USIWM1) | _BV(USIWM0); // ENABLE TWI
    USICR |= _BV(USICS1);               // External clock

    // Enable Interrupts
    sei();

    while (1)
    {
        if (led)
        {
            PORTB |= _BV(LED_STATUS);
        }
        else
        {
            PORTB &= ~_BV(LED_STATUS);
        }
    }
}

ISR(USI_START_vect)
{
    USICR |= _BV(USIOIE); // Enable counter overflow interrupt
    USISR |= _BV(USISIF); // Clear the start interrupt flag and release the SCL line
    USISR &= 0xF0;        // Reset the counter
}

ISR(USI_OVF_vect)
{
    switch (state)
    {
    case 0x00: // Addressing
    {
        // Check if being addressed
        if ((USIDR >> 1) == MCU_ADDRESS)
        {
            // Check if R/W
            if (USIDR & 0x01)
                state = 0x02; // Read operation
            else
            {
                state = 0x01; // Write operation
            }
            DDRB |= _BV(PB0); // Set as output -> Default value low
            USISR &= 0xF0;    // clear the counter before assigning value
            USISR |= 0xFE;    // clear the overflow flag and set counter to 14-one cycle before overflow
            // USISR |= _BV(USIOIF);
        }
        else // Reset and wait for new start condition
        {
            if (led)
                led = 0;
            else
                led = 1;
            USICR &= ~_BV(USIOIE); // Disable counter overflow interrupt
            USISR |= _BV(USIOIF);  // clear the overflow flag and release SCL line
            state = 0x00;
        }
        break;
    }

    case 0x01: // Write preparations
    {
        state = 0x03;
        DDRB &= ~_BV(PB0);    // Release SDA line
        USISR |= _BV(USIOIF); // Clear overflow flag and release SCL line
        break;
    }

    case 0x03: // Writing process
    {
        state = 0x00; // ASSUMGIN ONLY ONE BYTE
        cmd = USIDR;  // Read the cmd
        // Prepare for another write
        USICR &= ~_BV(USIOIE); // Disable counter overflow interrupt
        USISR |= _BV(USIOIF);  // Clear the overflow flag and release SCL line
        break;
    }
    }
}