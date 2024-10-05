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


// CHECK AND CORRECT THIS FUNCTION -> WORKS BUT NEEDS CHECKING!
void getNodeOrder(TwoWire &WireComm, uint8_t *nodes, uint8_t length, uint8_t offset, uint8_t start, uint8_t end)
{
    // Array to hold the already checked nodes
    bool burnt_address[length];
    // Initialize the burnt array to uncheck status
    for (int i = 0; i < length; i++)
    {
        burnt_address[i] = false;
    }

    // Temporal values for checking nodes
    bool search_flag = true;
    uint8_t readed_len = 0;
    uint8_t readed_value = 0x00;
    // Start by ordering the noder from 0 to length
    for (uint8_t node_index = 0; node_index < length; node_index++)
    {
        // Query every node : Lock CPU until a node answers
        search_flag = true;
        while (search_flag)
        {
            // Transverse every possible address
            for (uint8_t n_address = start; n_address <= end; n_address++)
            {
                // Only nodes that haven't answer can respond to the poll
                if (!burnt_address[n_address - offset])
                {
                    readed_len = WireComm.requestFrom((int)n_address, I2C_READ_BYTES, I2C_STOP_END);
                    if (readed_len > 0)
                    {
                        // Button has been pressed
                        readed_value = WireComm.read();
                        // Only accept the button associated to the nodes (green nodes)
                        if(readed_value == INPUT_B_PRESSED)
                        {
                            // Inform the user that the node has been checked by turning it ON
                            sendWireCMD(WireComm, n_address, ON_CMD);
                            nodes[node_index] = n_address;
                            burnt_address[n_address-offset] = true;
                            search_flag = false;
                            break;
                        }
                    }
                }
            }
        }
    }

    // After the nodes have been ordered: Inform the user by animating the found order
    // Turn all nodes to OFF and then turn then ON in order and finally all back to OFF state
    delay(1000);
    sendWireCMD(WireComm, 0, OFF_CMD);
    delay(1000);
    for(uint8_t node_index = 0; node_index < length; node_index++)
    {
        sendWireCMD(WireComm, (int) nodes[node_index], ON_CMD);
        delay(500);
    }
    delay(2000);
    sendWireCMD(WireComm, 0, OFF_CMD);
}

uint8_t nextNode(uint8_t current_node, uint8_t length)
{
    if (current_node == (length-1))
    {
        // Cycle back to start
        return 0;
    }
    else
    {
        // Just go to the next index available
        return current_node + 1;
    }
}

uint8_t prevNode(uint8_t current_node, uint8_t length)
{
    if (current_node == 0)
    {
        // First node, loop to the end of node list
        return (length-1);
    }
    else
    {
        // Return next node available
        return current_node - 1;
    }
}