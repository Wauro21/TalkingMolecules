#include "Arduino.h"
#include "functions.hpp"

// --------------- SERIAL 2 Configurations ---------------
// -> Pinout
#define RX2 16
#define TX2 17
// -> BaudRate for MP3 module
#define BAUDRATE 9600
// ----------------- MP3 Defaults Values ----------------
#define CMD_PLAY_FOLDER_TRACK 0x0F
#define A_FOLDER_TRACK 0x0101

// ---------------- Pinout Configurations ----------------
// -> Used to read the state of the MP3 player:
#define BUSY_READ_PIN 4
#define BUSY_THRESHOLD 2500
#define INPUT_BUTTON 2

// -> Input buttons states
ButtonStates button_a_state = BUTTON_IDLE;
// -> Debouncing states
uint32_t button_a_debounce = 0U;

// -> MP3 module busy state
uint16_t busy_pin_read = 0;
// -> FSM State - 0: idle, 1: playing
int current_state = 0;

// Led bank A
#define LED_A_1 18
#define LED_A_2 19
#define LED_A_3 21

// Led bank B
#define LED_B_1 23
#define LED_B_2 22
#define LED_B_3 25

void setup()
{
    // SETUP IO
    pinMode(BUSY_READ_PIN, INPUT); // MP3 Busy flag
    pinMode(INPUT_BUTTON, INPUT_PULLUP); // Button input
    // -> LED: Bank A 
    pinMode(LED_A_1, OUTPUT);
    pinMode(LED_A_2, OUTPUT);
    pinMode(LED_A_3, OUTPUT);
    // -> LED: Bank  B
    pinMode(LED_B_1, OUTPUT);
    pinMode(LED_B_2, OUTPUT);
    pinMode(LED_B_3, OUTPUT);

    Serial2.begin(BAUDRATE, SERIAL_8N1, RX2, TX2);
    delay(1000);
    // SETUP MP3 PLAYER
    // -> Reset module
    sendMP3CMD(Serial2, 0x0C, 0x0000);
    // -> Setup SD card
    sendMP3CMD(Serial2, 0x09, 0x0002);
    // -> Setup volume to 20/30
    sendMP3CMD(Serial2, 0x06, 0x0014);

    // Initialize state variables
    busy_pin_read = 0;
    current_state = 0;
    // Button states
    button_a_state = BUTTON_IDLE;
    button_a_debounce = 0U;

    // Testing LEDs
    digitalWrite(LED_A_1, HIGH);
    digitalWrite(LED_A_2, HIGH);
    digitalWrite(LED_A_3, HIGH);
    digitalWrite(LED_B_1, HIGH);
    digitalWrite(LED_B_2, HIGH);
    digitalWrite(LED_B_3, HIGH);
    delay(2000);
    digitalWrite(LED_A_1, LOW);
    digitalWrite(LED_A_2, LOW);
    digitalWrite(LED_A_3, LOW);
    digitalWrite(LED_B_1, LOW);
    digitalWrite(LED_B_2, LOW);
    digitalWrite(LED_B_3, LOW);
    

}

void loop()
{

    // -> BUTTON LOGIC
    if(current_state == 0) // Checking if IDLE
    {
        // Read button
        button_a_state = readButtonDebounce(&button_a_debounce, INPUT_BUTTON);

        if(button_a_state == BUTTON_PRESS)
        {
            sendMP3CMD(Serial2, CMD_PLAY_FOLDER_TRACK, A_FOLDER_TRACK);
            current_state = 1;
            delay(1000);
        }
    }

    else
    {
        // Read the state of busy bit
        busy_pin_read = analogRead(BUSY_READ_PIN);
        if(busy_pin_read > BUSY_THRESHOLD)
        {
            current_state = 0;
            button_a_state = BUTTON_IDLE;
            button_a_debounce = 0;
        }
    }

    // -> LIGHT LOGIC
}
