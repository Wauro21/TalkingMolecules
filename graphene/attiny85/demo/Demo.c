// SLAVE

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define STATUS_LED PB4

//--------Identidad del esclavo--------
uint8_t address_read = 0x0B;
uint8_t address_write = 0x0A;
//-------------------------------------

uint8_t usi_data;

void usi_init(void)
{
    USICR |= 1 << USICS1; // Fuente de clock externa, flanco positivo. Counter software strobe (USITC)
    // USICR |= 1<<USIOIE; //Counter overflow eneable
    // USICR |= 1<<USISIE; //Start condition interrupt eneable
    // sei();

    USICR |= 1 << USIWM1; // TWI eneable
}

void get_data(void)
{
    USISR &= 0xF0; // Limpiar todas las flags y counter bits = 0; La condicion de start tambien incrementa el contador
    while (!(USISR & (1 << USIOIF)))
        ;                 // Esperar a que el registro se llene
    USISR |= 1 << USIOIF; // Limpiar la flag de overflow
}

void usi_stop(void)
{
    USICR &= ~(1 << USIWM1); // TWI disable
}

void listen_address(void)
{
    while (!(USISR & (1 << USISIF)))
        ; // Espera a la señal de start
    get_data();
}

void ack_response(void)
{
    DDRB |= 1 << PB0; // Toma el control de SDA; Esto pone a la linea en LOW por default
    while ((USISR & 0x02) == 0)
        ;                 // Esperar a que incremente el contador
    _delay_us(5);         // Tiempo de estabilizacion
    DDRB &= ~(1 << PB0);  // Libera la linea SDA
    USISR |= 1 << USIOIF; // Limpiar la flag de overflow
}

int main(void)
{
    _delay_ms(100); // Tiempo de estabilizacion
    // SETUP LED STATUS
    DDRB = 0;
    DDRB |= _BV(STATUS_LED);
    PORTB |= _BV(STATUS_LED);
    while (1)
    {
        usi_init();
        listen_address();
        usi_data = USIDR;                                              // Recuperar la address del registro de datos
        if ((usi_data == address_write) || (usi_data == address_read)) // Checar si la address da un match
        {
            ack_response();
            get_data();
            usi_data = USIDR; // Recuperar la data
            if (usi_data == 0b00010001)
            {
                PORTB |= _BV(STATUS_LED);
            }

            else if (usi_data == 0b00000000)
            {
                PORTB &= ~_BV(STATUS_LED);
            }
            ack_response(); // Podria mandarse el ACK sin hacer la evaluacion del if si se desea
        }
        usi_stop();
    }
}