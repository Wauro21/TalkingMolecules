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
#define LIMIT_A 12 // CHECK
#define LIMIT_B 13 // CHECK

typedef enum
{
    IDLE,
    ONE,
    TWO,
    THREE
} LedStatus;

// -> Input buttons states
ButtonStates button_a_state = BUTTON_IDLE;
ButtonStates limit_a_state = BUTTON_IDLE;
ButtonStates limit_b_state = BUTTON_IDLE;
// -> Debouncing states
uint32_t button_a_debounce = 0U;
uint32_t limit_a_debounce = 0U;
uint32_t limit_b_debounce = 0U;

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

// LED STATUS
LedStatus a = IDLE;
LedStatus b = IDLE;

bool a_1 = false;
bool a_2 = false;
bool a_3 = false;
bool b_1 = false;
bool b_2 = false;
bool b_3 = false;

// IF ENOUGH TIME FOR TIMERS
// https://www.luisllamas.es/en/esp32-timers/
// https://docs.espressif.com/projects/arduino-esp32/en/latest/api/timer.html

void setup()
{
    // SETUP IO
    pinMode(BUSY_READ_PIN, INPUT);       // MP3 Busy flag
    pinMode(INPUT_BUTTON, INPUT_PULLUP); // Button input
    // -> LIMIT SWITCHES
    pinMode(LIMIT_A, INPUT_PULLUP);
    pinMode(LIMIT_B, INPUT_PULLUP);
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

    // Limit states
    limit_a_state = BUTTON_IDLE;
    limit_b_state = BUTTON_IDLE;
    limit_a_debounce = 0U;
    limit_b_debounce = 0U;

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

    // LED STATUS
    a = IDLE;
    b = IDLE;
    a_1 = false;
    a_2 = false;
    a_3 = false;
    b_1 = false;
    b_2 = false;
    b_3 = false;
}

void loop()
{

    // LIMIT LOGIC
    limit_a_state = readButtonDebounce(&limit_a_debounce, LIMIT_A);
    limit_b_state = readButtonDebounce(&limit_b_debounce, LIMIT_B);

    if(limit_a_state == BUTTON_PRESS)
    {
        switch(a)
        {
            case IDLE:
            {
                a_1 = true;
                a = ONE;
                break;
            }

            case ONE:
            {
                a_2 = true;
                a = TWO;
                break;
            }

            case TWO:
            {
                a_3 = true;
                a = THREE;
                break;
            }

            case THREE:
            {
                a_1 = false;
                a_2 = false;
                a_3 = false;
                a = IDLE;
                break;
            }
        }
    }

    if(limit_b_state == BUTTON_PRESS)
    {
        switch(b)
        {
            case IDLE:
            {
                b_1 = true;
                b = ONE;
                break;
            }

            case ONE:
            {
                b_2 = true;
                b = TWO;
                break;
            }

            case TWO:
            {
                b_3 = true;
                b = THREE;
                break;
            }

            case THREE:
            {
                b_1 = false;
                b_2 = false;
                b_3 = false;
                b = IDLE;
                break;
            }
        }
        
    }

    // -> BUTTON LOGIC
    if (current_state == 0) // Checking if IDLE
    {
        // Read button
        button_a_state = readButtonDebounce(&button_a_debounce, INPUT_BUTTON);

        if (button_a_state == BUTTON_PRESS)
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
        if (busy_pin_read > BUSY_THRESHOLD)
        {
            current_state = 0;
            button_a_state = BUTTON_IDLE;
            button_a_debounce = 0;
        }
    }

    // -> LIGHT LOGIC

    digitalWrite(LED_A_1, a_1);
    digitalWrite(LED_A_2, a_2);
    digitalWrite(LED_A_3, a_3);
    digitalWrite(LED_B_1, b_1);
    digitalWrite(LED_B_2, b_2);
    digitalWrite(LED_B_3, b_3);
}
