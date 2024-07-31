#include <Wire.h>

bool led_status = false;
void setup()
{
    // Start i2c
    Wire.begin();
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
}

void loop()
{
    digitalWrite(13, LOW);
    Wire.beginTransmission(0x0A);
    Wire.write(0x11);
    // if(led_status) // led on -> turn off
    // {
    //     Wire.write(0x00);
    //     led_status = false;
    // }
    // else //led off -> Turn on
    // {
    //     Wire.write(0x11);
    //     led_status = true;
    // }
    Wire.endTransmission();
    digitalWrite(13, HIGH);
    delay(2000);

}