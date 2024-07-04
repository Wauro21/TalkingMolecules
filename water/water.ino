#include <Arduino.h>

// --------------- SERIAL 2 Configurations ---------------
// -> Pinout
#define RX2 16
#define TX2 17
// -> BaudRate for MP3 module
#define BAUDRATE 9600
// ---------- Software Debouncer Configurations ----------
#define PRESS_BUTTON_MASK 0x80000000
#define HOLD_BUTTON_MASK 0xFFFFFFFF
typedef enum {
    BUTTON_IDLE     = 0x00,
    BUTTON_PRESS    = 0x01,
    BUTTON_HOLD     = 0x02,
} ButtonStates;
// ---------------- Pinout Configurations ----------------
// -> Used to read the state of the MP3 player:
#define BUSY_READ_PIN 34 
#define BUSY_THRESHOLD 2500
// -> Used for relay and motor control of each atom
#define O_PIN_A 2
#define O_PIN_B 0
#define O_PIN_C 4
// -> Input pins for buttons
#define I_PIN_A 33
#define I_PIN_B 32
#define I_PIN_C 25
// -> Status LED
#define STATUS_LED 23
// ----------------- MP3 Defaults Values ----------------
#define CMD_PLAY_FOLDER_TRACK 0x0F
#define A_FOLDER_TRACK 0x0101
#define B_FOLDER_TRACK 0x0102
#define C_FOLDER_TRACK 0x0103
// -------------- States and readed values --------------
// -> Output pin states
bool state_a = false;
bool state_b = false;
bool state_c = false;
// -> Input buttons states
ButtonStates button_a_state = BUTTON_IDLE;
ButtonStates button_b_state = BUTTON_IDLE;
ButtonStates button_c_state = BUTTON_IDLE;
// -> Debouncing states
uint32_t button_a_debounce = 0U;
uint32_t button_b_debounce = 0U;
uint32_t button_c_debounce = 0U;
// -> MP3 module busy state 
uint16_t busy_pin_read = 0;
// -> FSM State - 0: idle, 1: playing
int current_state = 0;
// ---------------- Functions declarations --------------

/// @brief Read and debounce the desired button
/// @param debounce_var The state variable for the button
/// @param pin The pin to read and debounce
/// @return The current state of the button
ButtonStates readButtonDebounce(uint32_t* debounce_var, int pin);

/// @brief Sends a CMD with data to the MP3 Player
/// @param cmd One of the defined cmd for the player
/// @param data Data associated with the command
void sendMP3CMD(uint8_t cmd, uint16_t data);

/// @brief Setup operation of the microcontroller
void setup()
{
    // For Serial2 the output pins should be explicitly defined
    Serial2.begin(BAUDRATE, SERIAL_8N1, RX2, TX2);
    // Wait a second for UART initialization 
    delay(1000);
    // SETUP MP3 PLAYER
    // -> Reset module
    sendMP3CMD(0x0C, 0x0000);
    // -> Setup SD card
    sendMP3CMD(0x09, 0x0002);
    // -> Setup volume to 20/30
    sendMP3CMD(0x06, 0x0014);
    
    // SETUP OUTPUT PINS
    pinMode(O_PIN_A, OUTPUT);
    pinMode(O_PIN_B, OUTPUT);
    pinMode(O_PIN_C, OUTPUT);
    // SETUP BUTTON PINS
    pinMode(I_PIN_A, INPUT_PULLUP);
    pinMode(I_PIN_B, INPUT_PULLUP);
    pinMode(I_PIN_C, INPUT_PULLUP);
    // SETUP PIN FOR MP3 BUSY FLAG
    pinMode(BUSY_READ_PIN, INPUT);
    // SETUP STATUS LED
    pinMode(STATUS_LED, OUTPUT);

    // INITIALIZE OUTPUS
    digitalWrite(STATUS_LED, LOW);
    digitalWrite(O_PIN_A, LOW);
    digitalWrite(O_PIN_B, LOW);
    digitalWrite(O_PIN_C, LOW);

    // Initialize state variables
    busy_pin_read = 0;
    current_state = 0;
    // -> Output states
    state_a = false;
    state_b = false;
    state_c = false;
    // -> Button states
    button_a_state = BUTTON_IDLE;
    button_b_state = BUTTON_IDLE;
    button_c_state = BUTTON_IDLE;
    // -> Button debounce states
    button_a_debounce = 0U;
    button_b_debounce = 0U;
    button_c_debounce = 0U;
}


void loop()
{
    if(current_state == 0) //check if IDLE
    {
        // Always reset the state of the outputs
        state_a = false;
        state_b = false;
        state_c = false; 

        // Read the buttons values
        button_a_state = readButtonDebounce(&button_a_debounce, I_PIN_A);
        button_b_state = readButtonDebounce(&button_b_debounce, I_PIN_B);
        button_c_state = readButtonDebounce(&button_c_debounce, I_PIN_C);

        if(button_a_state == BUTTON_PRESS)
        {
            state_a = true;
            sendMP3CMD(CMD_PLAY_FOLDER_TRACK, A_FOLDER_TRACK);
            current_state = 1;
            delay(1000);
        }

        if(button_b_state == BUTTON_PRESS)
        {
            state_b = true;
            sendMP3CMD(CMD_PLAY_FOLDER_TRACK, B_FOLDER_TRACK);
            current_state = 1;
            delay(1000);
        }

        if(button_c_state == BUTTON_PRESS)
        {
            state_c = true;
            sendMP3CMD(CMD_PLAY_FOLDER_TRACK, C_FOLDER_TRACK);
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
            button_b_state = BUTTON_IDLE;
            button_c_state = BUTTON_IDLE;
            button_a_debounce = 0;
            button_b_debounce = 0;
            button_c_debounce = 0;
        }
    }


    // Update outputs and state led
    digitalWrite(O_PIN_A, state_a);
    digitalWrite(O_PIN_B, state_b);
    digitalWrite(O_PIN_C, state_c);
    digitalWrite(STATUS_LED, current_state);
    

}

ButtonStates readButtonDebounce(uint32_t* debounce_var, int pin)
{
  // Read the current state of the button
  uint8_t temporal_read = (uint8_t) digitalRead(pin);
  uint8_t read_value = (uint8_t) (~temporal_read & bit(pin) >> pin) ; // Negative logic --> when button pressed is low, resulting = 1111_1111 | for high = 0000_0001
  // Debounce the value 
  *debounce_var = (*debounce_var << 1) | read_value;

  // Check current state
  switch(*debounce_var)
  {
    case PRESS_BUTTON_MASK: return BUTTON_PRESS;
    case HOLD_BUTTON_MASK: return BUTTON_HOLD;
    default: return BUTTON_IDLE;
  }

}

void sendMP3CMD(uint8_t cmd, uint16_t data)
{
    uint8_t frame[10] = { 0 };
    frame[0] = 0x7e; // Starting frame 
    frame[1] = 0xff; // Version
    frame[2] = 0x06; // Number of remaining bytes
    frame[3] = cmd; // CMD
    frame[4] = 0x00; // No feedback
    frame[5] = (uint8_t) (data >> 8) & 0xFF; // Data: high byte
    frame[6] = (uint8_t) (data) & 0xFF; // Data: low byte
    
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