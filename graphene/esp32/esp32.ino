#include <Arduino.h>
#include <Wire.h>
// --------------- I2C      Configurations ---------------
#define START_ADDRESS 8
#define END_ADDRESS 13 // Inclusive
#define ON_CMD 0x11
#define OFF_CMD 0x00
#define INPUT_A_PRESSED 0xAA
#define INPUT_B_PRESSED 0x55
#define I2C_READ_BYTES 1
#define I2C_STOP_END true
#define WIRE_INIT_DELAY 1000
#define I2C_STRIC_TEST_PIN 32
#define I2C_STRICT_PIN_THRESHOLD 500 // LOW FOR ON
#define I2C_TEST_DELAY 2000
// --------------- SERIAL 2 Configurations ---------------
// -> Pinout
#define RX2 16
#define TX2 17
// -> BaudRate for MP3 module
#define BAUDRATE 9600
// ---------------- Pinout Configurations ----------------
// -> Used to read the state of the MP3 player:
#define BUSY_READ_PIN 34
#define BUSY_THRESHOLD 2500
#define STATUS_LED 23
// ----------------- MP3 Defaults Values ----------------
#define CMD_PLAY_FOLDER_TRACK 0x0F
// -> Tracks for button A pressed
#define A_FOLDER_TRACK_A 0x0101
#define B_FOLDER_TRACK_A 0x0102
#define C_FOLDER_TRACK_A 0x0103
#define D_FOLDER_TRACK_A 0x0104
#define E_FOLDER_TRACK_A 0x0105
#define F_FOLDER_TRACK_A 0x0106
// -> Tracks for button B pressed
#define A_FOLDER_TRACK_B 0x0201
#define B_FOLDER_TRACK_B 0x0202
#define C_FOLDER_TRACK_B 0x0203
#define D_FOLDER_TRACK_B 0x0204
#define E_FOLDER_TRACK_B 0x0205
#define F_FOLDER_TRACK_B 0x0206
// -> MP3 module busy state
uint16_t busy_pin_read = 0;
// -> FSM State - 0: idle, 1: playing
int current_state = 0;
// ----------------- INTERNAL FSM -----------------------
bool lock_cpu = true;
int node_pressed = 0;
uint8_t node_button = 0;
int audio_track = 0;
// ---------------- Functions declarations --------------

/// @brief Sends a CMD with data to the MP3 Player
/// @param cmd One of the defined cmd for the player
/// @param data Data associated with the command
void sendMP3CMD(uint8_t cmd, uint16_t data);

/// @brief Sends a command to the specified i2c-address
/// @param address The submodule address
/// @param cmd The command to send
/// @return The return code for the operation
uint8_t sendWireCMD(uint8_t address, uint8_t cmd);

/// @brief Test all the devices, by sending a ON command followed
/// by an OFF command. This tests doesn't verify if the secondary
/// devices respond to the commands. Useful for checking
/// general connections
void generalTest();

/// @brief Performs an strict testing to every submodule. All
/// outputs are set to HIGH and each device should have both
/// buttons pressed in the order of the corresponding addresses
/// from MIN to MAX address.
void strictTest();

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

    // SETUP PIN FOR MP3 BUSY FLAG
    pinMode(BUSY_READ_PIN, INPUT);
    // SETUP STATUS LED
    pinMode(STATUS_LED, OUTPUT);

    // Setup Wire/i2c communication
    pinMode(I2C_STRIC_TEST_PIN, INPUT);
    delay(WIRE_INIT_DELAY);
    // Check for strict mode testing
    uint8_t strict_test_read = analogRead(I2C_STRIC_TEST_PIN);
    Wire.begin();
    // Wait and check for testing
    delay(WIRE_INIT_DELAY);
    generalTest();
    delay(WIRE_INIT_DELAY);
    if (strict_test_read >= I2C_STRICT_PIN_THRESHOLD)
    {
        strictTest();
    }

    // INITIALIZE OUTPUS
    digitalWrite(STATUS_LED, LOW);

    // Initialize state variables
    busy_pin_read = 0;
    current_state = 0;
}

void loop()
{
    // Main loop: Check if any buttons has been pressed - HOLD CPU UNTIL BUTTON IS REPORTED
    while (lock_cpu)
    {
        // Poll nodes
        uint8_t response_length = 0;
        for (uint8_t add_i = START_ADDRESS; add_i <= END_ADDRESS; add_i++)
        {
            response_length = Wire.requestFrom((int)add_i, I2C_READ_BYTES, I2C_STOP_END);
            if (response_length)
            {
                // A valid message has been received from node
                node_pressed = add_i;
                node_button = Wire.read();
                // Unlock CPU to go to sound and animation routine
                lock_cpu = false;
                // Escape checking routine
                break;
            }
        }
    }

    // Send the audio for playback
    audio_track = getNodeAudioTrack(node_pressed, node_button);
    sendMP3CMD(CMD_PLAY_FOLDER_TRACK, audio_track);
    // Wait before checking pin for animation
    delay(500);
    // Get the animation
    while (!lock_cpu)
    {
        busy_pin_read = analogRead(BUSY_READ_PIN);
        if(busy_pin_read < BUSY_THRESHOLD)
        {
            lock_cpu = true;
            // Clear the animation
            sendWireCMD(0, OFF_CMD);
            break;
        }

        // Select the animation to display [TO BE IMPLEMENTED]
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

uint8_t sendWireCMD(uint8_t address, uint8_t cmd)
{
    uint8_t ret_value = 0;
    Wire.beginTransmission(address);
    Wire.write(cmd);
    ret_value = Wire.endTransmission(I2C_STOP_END);
    return ret_value;
}

void generalTest(void)
{
    char error_code = 0x00;
    char cmd = ON_CMD;
    for (int i = 0; i < 2; i++)
    {
        error_code = sendWireCMD(0, cmd);
        if (error_code)
        {
            Serial.print("Error during TEST - code: ");
            Serial.println(error_code);
            // Could add LED indication for error
        }
        delay(I2C_TEST_DELAY);
        cmd = OFF_CMD;
    }
}

void strictTest(void)
{
    // Turn on all devices
    sendWireCMD(0, ON_CMD);

    uint8_t readed_len = 0;
    uint8_t readed_value = 0x00;
    uint8_t test = 0x00;
    // Check buttons on each device
    for (uint8_t add_i = START_ADDRESS; add_i <= END_ADDRESS; add_i++)
    {
        while (test != 0xFF)
        {
            readed_len = Wire.requestFrom((int)add_i, I2C_READ_BYTES, I2C_STOP_END);
            if (readed_len)
            {
                readed_value = Wire.read();
                switch (readed_value)
                {
                case INPUT_A_PRESSED:
                    test |= 0x0F;
                    break;

                case INPUT_B_PRESSED:
                    test |= 0xF0;
                    break;

                default:
                    break;
                }
            }
        }
        // If device passed turn-off led and go to the next device on list
        sendWireCMD(add_i, OFF_CMD);
        readed_value = 0x00;
        test = 0x00;
    }
}

int getNodeAudioTrack(int node_id, uint8_t button_pressed)
{
    switch (node_id)
    {
    case 8:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return A_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return A_FOLDER_TRACK_B;
        }
        break;
    }
    case 9:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return B_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return B_FOLDER_TRACK_B;
        }
        break;
    }
    case 10:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return C_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return C_FOLDER_TRACK_B;
        }
        break;
    }
    case 11:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return D_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return D_FOLDER_TRACK_B;
        }
        break;
    }
    case 12:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return E_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return E_FOLDER_TRACK_B;
        }
        break;
    }
    case 13:
    {
        if (button_pressed == INPUT_A_PRESSED)
        {
            return F_FOLDER_TRACK_A;
        }
        else if (button_pressed == INPUT_B_PRESSED)
        {
            return F_FOLDER_TRACK_B;
        }
        break;
    }

    default:
        return 0;
    }
}