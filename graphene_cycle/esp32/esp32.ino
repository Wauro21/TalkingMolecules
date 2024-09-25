#include "Arduino.h"

// --------------- SERIAL 2 Configurations ---------------
// -> Pinout
#define RX2 16
#define TX2 17
// -> BaudRate for MP3 module
#define BAUDRATE 9600
// ----------------- MP3 Defaults Values ----------------
#define CMD_PLAY_FOLDER_TRACK 0x0F
#define A_FOLDER_TRACK 0x0101

// ---------- Software Debouncer Configurations ----------
#define PRESS_BUTTON_MASK 0x80000000
#define HOLD_BUTTON_MASK 0xFFFFFFFF
typedef enum
{
    BUTTON_IDLE = 0x00,
    BUTTON_PRESS = 0x01,
    BUTTON_HOLD = 0x02,
} ButtonStates;

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

// ---------------- Functions declarations --------------

/// @brief Read and debounce the desired button
/// @param debounce_var The state variable for the button
/// @param pin The pin to read and debounce
/// @return The current state of the button
ButtonStates readButtonDebounce(uint32_t *debounce_var, int pin);

/// @brief Sends a CMD with data to the MP3 Player
/// @param cmd One of the defined cmd for the player
/// @param data Data associated with the command
void sendMP3CMD(uint8_t cmd, uint16_t data);

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

    Serial2.begin(BAUDRATE, SERIAL_8N1, RX2, TX2);
    delay(1000);
    // SETUP MP3 PLAYER
    // -> Reset module
    sendMP3CMD(0x0C, 0x0000);
    // -> Setup SD card
    sendMP3CMD(0x09, 0x0002);
    // -> Setup volume to 20/30
    sendMP3CMD(0x06, 0x0014);

    // Initialize state variables
    busy_pin_read = 0;
    current_state = 0;
    // Button states
    button_a_state = BUTTON_IDLE;
    button_a_debounce = 0U;
}

void loop()
{
    if(current_state == 0) // Checking if IDLE
    {
        // Read button
        button_a_state = readButtonDebounce(&button_a_debounce, INPUT_BUTTON);

        if(button_a_state == BUTTON_PRESS)
        {
            sendMP3CMD(CMD_PLAY_FOLDER_TRACK, A_FOLDER_TRACK);
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
}

void sendMP3CMD(uint8_t cmd, uint16_t data)
{
    uint8_t frame[10] = {0};
    frame[0] = 0x7e;                        // Starting frame
    frame[1] = 0xff;                        // Version
    frame[2] = 0x06;                        // Number of remaining bytes
    frame[3] = cmd;                         // CMD
    frame[4] = 0x00;                        // No feedback
    frame[5] = (uint8_t)(data >> 8) & 0xFF; // Data: high byte
    frame[6] = (uint8_t)(data) & 0xFF;      // Data: low byte

    // Calculate checksum
    uint16_t checksum = 0xFFFF;
    for (uint8_t i = 1; i < 7; i++)
    {
        checksum -= frame[i];
    }
    checksum += 1;
    frame[7] = (checksum >> 8) & 0xFF;
    frame[8] = checksum & 0xFF;
    frame[9] = 0xef; // end byte

    for (uint8_t i = 0; i < 10; i++)
    {
        Serial2.write(frame[i]);
    }
}

ButtonStates readButtonDebounce(uint32_t *debounce_var, int pin)
{
    // Read the current state of the button
    uint8_t temporal_read = (uint8_t)digitalRead(pin);
    uint8_t read_value = (uint8_t)(~temporal_read & bit(pin) >> pin); // Negative logic --> when button pressed is low, resulting = 1111_1111 | for high = 0000_0001
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