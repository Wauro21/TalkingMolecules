#include <Wire.h>

#define DEBUG_SERIAL false
#define DEBUG_DELAY 1
#define STRICT_TESTING true
#define START_ADDRESS 8
#define END_ADDRESS 9
#define ON_CMD 0x11
#define OFF_CMD 0x00
#define INPUT_A_PRESSED 0xAA
#define INPUT_B_PRESSED 0x55
#define N_READ_BYTES 1
#define STOP_AT_END true
#define STATUS_LED 13
#define TEST_DELAY 2000
#define WIRE_INIT_DELAY 1000

// -> NEW 




// VARIABLES
uint8_t read_response = 0x00;
int read_len = 0;
bool dev_status[] = {false, false};
int offset = 8;
bool EMULATE_DELAY = false;
bool NEW_TEST = false;

// FUNCTIONS
void generalTest(void);
void strictTest(void);
uint8_t sendCMD(uint8_t address, uint8_t cmd);


void debugRead(int address, char response);
void answerWire(int address, char read_val);
void emulateWait(uint8_t address, uint8_t read);

void setup()
{
    // Setup I/O
    pinMode(STATUS_LED, OUTPUT);
    // Serial for debugging
    Serial.begin(9600);
    // Start i2c
    Wire.begin();
    // WAIT AND PERFORM A CHECK ON ALL DEVICES
    delay(WIRE_INIT_DELAY);
    generalTest();
    if(STRICT_TESTING)
    {
      delay(WIRE_INIT_DELAY);
      strictTest();
    }

}

void loop()
{
  for (int add_i = START_ADDRESS; add_i <= END_ADDRESS; add_i++)
  {
    read_len = Wire.requestFrom(add_i, N_READ_BYTES, STOP_AT_END);
    if(read_len)
    {
      read_response = Wire.read();
      emulateWait(add_i, read_response);
    }
    delay(1);
  }
}

uint8_t sendCMD(uint8_t address, uint8_t cmd)
{
  uint8_t ret_value = 0;
  Wire.beginTransmission(address);
  Wire.write(cmd);
  ret_value = Wire.endTransmission(STOP_AT_END);
  return ret_value;

}

void strictTest(void)
{
  // Turn on all devices
  sendCMD(0, ON_CMD);

  uint8_t readed_len = 0;
  uint8_t readed_value = 0x00;
  uint8_t test = 0x00;
  // Check buttons on each device
  for (uint8_t add_i = START_ADDRESS; add_i <= END_ADDRESS; add_i++)
  {
    while(test != 0xFF)
    {
      readed_len = Wire.requestFrom((int)add_i, N_READ_BYTES, STOP_AT_END);
      if(readed_len)
      {
        readed_value = Wire.read();
        switch(readed_value)
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
    sendCMD(add_i, OFF_CMD);
    readed_value = 0x00;
    test = 0x00;
  }
}

void generalTest(void)
{
  char error_code = 0x00;
  char cmd = ON_CMD;
  for (int i = 0; i < 2; i++)
  {
    error_code = sendCMD(0, cmd);
    if(error_code)
    {
      Serial.print("Error during TEST - code: ");
      Serial.println(error_code);
      // Could add LED indication for error
    }
    delay(TEST_DELAY);
    cmd = OFF_CMD;
  }
  
}

void emulateWait(uint8_t address, uint8_t read)
{
  if(read)
  {
    Serial.print("Read from address: ");
    Serial.print(address);
    Serial.print(" | value: ");
    Serial.println(read, HEX);

    if(read == INPUT_A_PRESSED)
    {
      sendCMD(address, ON_CMD);
    }

    else
    {
      sendCMD(0, ON_CMD);
    }
    // Emulate wait
    digitalWrite(STATUS_LED, HIGH);
    delay(10000);
    digitalWrite(STATUS_LED, LOW);
    if(read == INPUT_A_PRESSED)
    {
      sendCMD(address, OFF_CMD);
    }

    else
    {
      sendCMD(0, OFF_CMD);
    }

  }
}