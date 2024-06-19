#include <Arduino.h>
//#include "DFRobotDFPlayerMini.h"

#define PRESS_BUTTON_MASK 0x80000000
#define HOLD_BUTTON_MASK 0xFFFFFFFF

bool state_a = true;
bool state_b = true;
bool state_c = true;

// button button states
int button_a_state = 0;
int button_b_state = 0;
int button_c_state = 0;
uint32_t button_a_debounce = 0U;
uint32_t button_b_debounce = 0U;
uint32_t button_c_debounce = 0U;

int readButtonDebounce(uint32_t* debounce_var, int pin);

void sendMP3CMD(int8_t cmd, int16_t data);

void setup()
{
    Serial2.begin(9600);
    delay(500);

    // Testing MP3 Player
    // -> Setup SD card
    sendMP3CMD(0x09, 0x02);
    delay(200);
    // -> Select folder
    sendMP3CMD(0x0F, 0x0101);
    delay(200);
    // -> select track
    sendMP3CMD(0x03, 0x01);


    // Setup pins
    // -> Output leds 12, 14, 27
    pinMode(12, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(27, OUTPUT);
    // -> Input buttons 13, 26, 25
    pinMode(13, INPUT_PULLUP);
    pinMode(26, INPUT_PULLUP);
    pinMode(25, INPUT_PULLUP);
}


void loop()
{
    // Read the buttons values
    button_a_state = readButtonDebounce(&button_a_debounce, 13);
    button_b_state = readButtonDebounce(&button_b_debounce, 26);
    button_c_state = readButtonDebounce(&button_c_debounce, 25);

    if(button_a_state == 1)
    {
        state_a = !state_a;
    }

    if(button_b_state == 1)
    {
        state_b = !state_b;
    }

    if(button_c_state == 1)
    {
        state_c = !state_c;
    }


    digitalWrite(12, state_a);
    digitalWrite(14, state_b);
    digitalWrite(27, state_c);

}

int readButtonDebounce(uint32_t* debounce_var, int pin)
{
  // Read the current state of the button
  uint8_t temporal_read = (uint8_t) digitalRead(pin);
  uint8_t read_value = (uint8_t) (~temporal_read & bit(pin) >> pin) ; // Negative logic --> when button pressed is low, resulting = 1111_1111 | for high = 0000_0001
  // Debounce the value 
  *debounce_var = (*debounce_var << 1) | read_value;

  // Check current state
  switch(*debounce_var)
  {
    case PRESS_BUTTON_MASK: return 1;
    case HOLD_BUTTON_MASK: return 2;
    default: return 0;
  }

}

void sendMP3CMD(int8_t cmd, int16_t data)
{
    uint16_t checksum = 0;
    uint8_t frame[10] = { 0 };
    frame[0] = 0x7e; // Starting frame 
    frame[1] = 0xff; // Version
    frame[2] = 0x06; // Number of remaining bytes
    frame[3] = cmd; // CMD
    frame[4] = 0x00; // No feedback
    frame[5] = (int8_t) (data >> 8); // Data: high byte
    frame[6] = (int8_t) (data); // Data: low byte
    
    // Calculate checksum
    for (uint8_t i = 2; i < 8;  )
    
    frame[9] = 0xef; // end byte

    for (uint8_t i = 0; i < 8; i++)
    {
        Serial2.write(frame[i]);
    }
    
}