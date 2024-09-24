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

void generalTest(TwoWire &WireComm, Stream &SerialComm)
{
    char error_code = 0x00;
    char cmd = ON_CMD;
    for (int i = 0; i < 2; i++)
    {
        error_code = sendWireCMD(WireComm, 0, cmd);
        if (error_code)
        {
            SerialComm.print("Error during TEST - code: ");
            SerialComm.println(error_code);
            // Could add LED indication for error
        }
        delay(I2C_TEST_DELAY);
        cmd = OFF_CMD;
    }
}

void strictTest(TwoWire &WireComm)
{
    // Turn on all devices
    sendWireCMD(WireComm, 0, ON_CMD);

    uint8_t readed_len = 0;
    uint8_t readed_value = 0x00;
    uint8_t test = 0x00;
    // Check buttons on each device
    for (uint8_t add_i = START_ADDRESS; add_i <= END_ADDRESS; add_i++)
    {
        while (test != 0xFF)
        {
            readed_len = WireComm.requestFrom((int)add_i, I2C_READ_BYTES, I2C_STOP_END);
            if (readed_len)
            {
                readed_value = WireComm.read();
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
        sendWireCMD(WireComm, add_i, OFF_CMD);
        readed_value = 0x00;
        test = 0x00;
    }
}

void getNodeOrder(TwoWire &WireComm, uint8_t addresses, uint8_t length, uint8_t offset)
{
    uint8_t pressed_values = 0x00;

    for(int node_i = 0; node_i < length; node_i++)
    {
        // wait for a button to be pressed
    }
}