#include <Wire.h>

#define DEBUG_SERIAL false
#define DEBUG_DELAY 1
#define START_ADDRESS 8
#define END_ADDRESS 9
#define ON_CMD 0x11
#define OFF_CMD 0x00
#define N_READ_BYTES 1
#define STOP_AT_END true
#define NODE_ID 0x80
#define STATUS_LED 13

// VARIABLES
uint8_t read_response = 0x00;
int read_len = 0;
bool dev_status[] = {false, false};
int offset = 8;
bool EMULATE_DELAY = false;
bool NEW_TEST = false;

// FUNCTIONS
void debugRead(int address, char response);
void answerWire(int address, char read_val);
void emulateWait(void);

void setup()
{
    pinMode(STATUS_LED, OUTPUT);
    Serial.begin(9600);
    // Start i2c
    Wire.begin();

    delay(1000);
    // Initial setup
    // Wire.beginTransmission(8);
    // Wire.write(0x11);
    // char demo = Wire.endTransmission(true);
    // Serial.print("DEMO:");
    // Serial.println(demo, HEX);

}

void loop()
{
  for (int add_i = START_ADDRESS; add_i <= END_ADDRESS; add_i++)
  {
    read_len = Wire.requestFrom(add_i, N_READ_BYTES, STOP_AT_END);
    if(read_len)
    {
      read_response = Wire.read();
      if((read_response == 0x80) || (read_response == 0x01)){
        Serial.println(read_response, HEX);
      }
    }
    delay(1);
  }
//   if(EMULATE_DELAY)
//   {
//     emulateWait();
//     EMULATE_DELAY = false;
//   }
//   // For each address
//   for (int add_i = START_ADDRESS; add_i <= END_ADDRESS; add_i++)
//   {
//     // Read the current status if available
//     read_len = Wire.requestFrom(add_i, N_READ_BYTES, STOP_AT_END);
//     if(read_len) // If data get the response
//     {
//       read_response = Wire.read();
//       EMULATE_DELAY = !dev_status[add_i-offset];
//       // TEMPORAL DEBUG
//       debugRead(add_i, read_response);
//       // Send response
//       answerWire(add_i, read_response);
//     }
//     else
//     {
//       if(DEBUG_SERIAL)
//       {
//         Serial.print("Address: ");
//         Serial.print(add_i, HEX);
//         Serial.println(" NO RESPONSE");
//       }
//     }
//     if(DEBUG_DELAY)
//     {
//       delay(DEBUG_DELAY);
//     }
//   }
// }


// void debugRead(int address, char response)
// {
//   if(DEBUG_SERIAL)
//   {
//     Serial.print("ADDRESS: ");
//     Serial.print(address, HEX);
//     Serial.print(" | Response: ");
//     Serial.println(response, HEX);
//   }
// }

// void answerWire(int address, char read_val)
// {
//   bool current_status = dev_status[address-offset];
//   bool new_status = !current_status;
//   char cmd = 0x00;
//   if(current_status) // Led on
//   {
//     cmd = 0x00;
//   }
//   else
//   {
//     cmd = 0x11;
//   }

//   // Send data
//   Wire.beginTransmission(address);
//   if(read_val & NODE_ID)
//   {
//     Wire.write(cmd);
//   }
//   else
//   {
//     cmd = 0x00;
//     new_status = false;
//     Wire.write(cmd);
//   }
//   int code = Wire.endTransmission(STOP_AT_END);
//   if(code && DEBUG_SERIAL)
//   {
//     Serial.print("ERROR: ");
//       Serial.print(code);
//       Serial.print(" | Address: ");
//       Serial.print(address);
//       Serial.print("| CMD: ");
//       Serial.println(cmd, HEX);
//   }
//   else
//   {
//     // UPDATE STATUS
//     dev_status[address-offset] = new_status;
//     if(DEBUG_SERIAL)
//     {
//       Serial.print("SUCCESS - ADDRESS: ");
//       Serial.print(address);
//       Serial.print(" | CMD: ");
//       Serial.println(cmd, HEX);
//     }

//   }
}

void emulateWait(void)
{
  digitalWrite(STATUS_LED, HIGH);
  delay(10000);
  digitalWrite(STATUS_LED, LOW);
}