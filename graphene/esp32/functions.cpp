#include "functions.hpp"

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

uint8_t sendWireCMD(TwoWire &WireComm, uint8_t address, uint8_t cmd)
{
    uint8_t ret_value = 0;
    WireComm.beginTransmission(address);
    WireComm.write(cmd);
    ret_value = WireComm.endTransmission(I2C_STOP_END);
    return ret_value;
}