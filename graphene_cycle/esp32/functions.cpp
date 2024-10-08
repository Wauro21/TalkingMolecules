#include "functions.hpp"


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


void sendMP3CMD(Stream &SerialComm, uint8_t cmd, uint16_t data)
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
        SerialComm.write(frame[i]);
    }
}