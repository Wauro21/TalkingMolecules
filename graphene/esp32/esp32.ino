#include <Arduino.h>
#include <Wire.h>
#include "functions.hpp"
#include "audio.hpp"

// ---------------- DEBUG --------------------------------
#define STRICT_TESTING_FLAG false

// --------------- I2C      Configurations ---------------
#define WIRE_INIT_DELAY 1000
#define I2C_STRIC_TEST_PIN 32
#define I2C_STRICT_PIN_THRESHOLD 500 // LOW FOR ON

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

// ----------------- Animations Values ----------------
#define ATOM_ANIMATION 0x11
#define BOND_ANIMATION 0x99
#define FULL_ANIMATION 0x55

// -> MP3 module busy state
uint16_t busy_pin_read = 0;
// -> FSM State - 0: idle, 1: playing
int current_state = 0;
// ----------------- INTERNAL FSM -----------------------
bool lock_cpu = true;
int node_pressed = 0;
uint8_t node_button = 0;
int audio_track = 0;
uint8_t nodes_order[6];
uint8_t current_animation = 0x00;
// ---------------- Functions declarations --------------

/// @brief Setup operation of the microcontroller
void setup()
{
    // For Serial2 the output pins should be explicitly defined
    Serial2.begin(BAUDRATE, SERIAL_8N1, RX2, TX2);
    // Wait a second for UART initialization
    delay(1000);
    // SETUP MP3 PLAYER
    // -> Reset module
    sendMP3CMD(Serial2, 0x0C, 0x0000);
    // -> Setup SD card
    sendMP3CMD(Serial2, 0x09, 0x0002);
    // -> Setup volume to 20/30
    sendMP3CMD(Serial2, 0x06, 0x0014);

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
    generalTest(Wire, Serial);
    delay(WIRE_INIT_DELAY);
    if (STRICT_TESTING_FLAG)
    {
        strictTest(Wire);
    }

    // INITIALIZE OUTPUS
    digitalWrite(STATUS_LED, LOW);

    // Initialize state variables
    busy_pin_read = 0;
    current_state = 0;

    // Ordering the nodes
    getNodeOrder(Wire, nodes_order, 6, 8);
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
                node_button = Wire.read();
                if (node_button != 0x00)
                {
                    // A valid message has been received from node
                    node_pressed = add_i;
                    // Unlock CPU to go to sound and animation routine
                    lock_cpu = false;

                    // Select animation
                    if (node_button == INPUT_A_PRESSED)
                    {
                        current_animation = BOND_ANIMATION;
                    }

                    else if (node_button == INPUT_B_PRESSED)
                    {
                        if ((add_i == 8) || (add_i == 10) || (add_i == add_i == 11))
                        {
                            current_animation = FULL_ANIMATION;
                        }
                        else
                        {
                            current_animation = ATOM_ANIMATION;
                        }
                    }

                    // Escape checking routine
                    break;
                }
            }
        }
    }

    // Send the audio for playback
    audio_track = getNodeAudioTrack(node_pressed, node_button);
    sendMP3CMD(Serial2, CMD_PLAY_FOLDER_TRACK, audio_track);
    // Wait before checking pin for animation
    delay(500);
    // Get the animation

    // Check the current node against the nodes order
    uint8_t node_order_index = node_pressed;
    uint8_t animation_counter = 0;

    for (int i = 0; i < 6; i++)
    {
        if (node_pressed == nodes_order[i])
        {
            node_order_index = i;
            break;
        }
    }

    while (!lock_cpu)
    {
        busy_pin_read = analogRead(BUSY_READ_PIN);
        if (busy_pin_read > BUSY_THRESHOLD)
        {
            lock_cpu = true;
            // Clear the animation
            current_animation = 0x00;
            animation_counter = 0;
            sendWireCMD(Wire, 0, OFF_CMD);
            break;
        }

        // Select the animation to display
        else
        {
            switch (current_animation)
            {
            case BOND_ANIMATION:
            {
                // Turn off previous nodes
                uint8_t pre_n = prevNode(node_order_index);
                uint8_t pre_pre_n = prevNode(pre_n);
                uint8_t next_node = nextNode(node_order_index);
                sendWireCMD(Wire, nodes_order[pre_pre_n], OFF_CMD);
                sendWireCMD(Wire, nodes_order[pre_n], OFF_CMD);

                // Turn on current nodes
                sendWireCMD(Wire, nodes_order[node_order_index], ON_CMD);
                sendWireCMD(Wire, nodes_order[next_node], ON_CMD);

                // Update current node
                node_order_index = next_node;
                delay(1000);
                break;
            }

            case ATOM_ANIMATION:
            {
                sendWireCMD(Wire, 0, ON_CMD);
                delay(1000);
                sendWireCMD(Wire, 0, OFF_CMD);
                delay(1000);
                break;
            }

            case FULL_ANIMATION:
            {
                uint8_t animation_cmd = ON_CMD;
                if (animation_counter < NUMBER_OF_NODES)
                {
                    animation_cmd = ON_CMD;
                }

                else
                {
                    animation_cmd = OFF_CMD;
                }
                sendWireCMD(Wire, nodes_order[node_order_index], animation_cmd);
                if (animation_counter == (2*NUMBER_OF_NODES))
                {
                    animation_counter = 0;
                }
                else
                {
                    animation_counter += 1;
                    node_order_index = nextNode(node_order_index);
                }
                delay(500);
                break;
            }

            default:
                break;
            }
        }
    }
}