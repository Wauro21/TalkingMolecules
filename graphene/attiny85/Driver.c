#include <avr/io.h>
#include <avr/interrupt.h>
#include "USI_TWI_Slave.h"

#define LED_STATUS PB4
#define MCU_ADDRESS 0x08


int main()
{
    DDRB |= _BV(LED_STATUS);
    DDRB |= _BV(PB3);
    PORTB |= _BV(LED_STATUS);
    usiTwiSlaveInit(8);
    uint8_t data = 0;
    sei();
    while(1)
    {
        // Check data received
        data = usiTwiReceiveByte();
        if(data == 1)
        {
            PORTB |= _BV(LED_STATUS);
        }

        else
        {
            PORTB &= ~_BV(LED_STATUS);
        }
    }
}