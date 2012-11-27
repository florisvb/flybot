// Ethernet client for Arduino, intended for use with ROS
// Floris van Breugel 2012

// Written for Arduino Uno with integrated Ethernet Shield

// Some useful info:
// from: http://arduino.cc/en/Main/ArduinoEthernetShield
// Arduino communicates with both the W5100 and SD card using the SPI bus 
// (through the ICSP header). This is on digital pins 11, 12, and 13 on the 
// Duemilanove and pins 50, 51, and 52 on the Mega. On both boards, pin 10 
// is used to select the W5100 and pin 4 for the SD card. These pins cannot 
// be used for general i/o. On the Mega, the hardware SS pin, 53, is not 
// used to select either the W5100 or the SD card, but it must be kept as an 
// output or the SPI interface won't work.

// Note that because the W5100 and SD card share the SPI bus, only one can 
// be active at a time. If you are using both peripherals in your program, 
// this should be taken care of by the corresponding libraries. If you're 
// not using one of the peripherals in your program, however, you'll need 
// to explicitly deselect it. To do this with the SD card, set pin 4 as an 
// output and write a high to it. For the W5100, set digital pin 10 as a 
// high output.

#include "Streaming.h"
#include "SerialReceiver.h"
#include "ModStepper.h"

SerialReceiver receiver;

// for UNO: 3, 5, 6, 9, 10, and 11. (the pins with the ~ next to them) 
// Provide 8-bit PWM output with the analogWrite() function
int clock_pin_a = 3;
int clock_pin_b = 3;
int dir_pin_a = 7;
int dir_pin_b = 8;
#define stepsPerRevolution 1000  // change this to fit the number of steps per revolution
ModStepper stepper(stepsPerRevolution,clock_pin_a,clock_pin_b);            

// Variables used for decoding and delimiting TCP msg
long spd;
long absspd;
long spd2;

void setup()
{
  // start the serial for debugging
  Serial.begin(19200);
  
  // set direction pins to output, and initialize stepper motors to zero
  delay(1000);
  pinMode(clock_pin_a,OUTPUT);
  pinMode(clock_pin_b,OUTPUT);
  pinMode(dir_pin_a,OUTPUT);
  pinMode(dir_pin_b,OUTPUT);
  
  pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS22);
  OCR2A = 256;
  OCR2B = 1;
  setPwmFrequency(3, 1024);
  
  spd = 0;
  spd2 = 0;

}

void loop()
{
  
  while (Serial.available() > 0) {
    receiver.process(Serial.read());
    if (receiver.messageReady()) {
        spd = receiver.readLong(0);
        spd2 = receiver.readLong(1);
        receiver.reset();
    
    
  
        Serial << analogRead(0) << endl;
          
        if (1) {
          // set speed on stepper
          
          if (spd > 0) {
            digitalWrite(dir_pin_b,HIGH);
          }
          if (spd < 0) {
            digitalWrite(dir_pin_b,LOW);
          }
          
          absspd = abs(spd);
          
          
          if (absspd <= 250) {
            setPwmFrequency(clock_pin_a, 1024);
            int newVal = -1*(absspd-256);
            if (newVal < 4) newVal = 4;
            OCR2A = newVal;
          }
          if (absspd > 250) {
            setPwmFrequency(clock_pin_a, 64);
            int newVal = -1*(absspd-256-256)-192+25;
            if (newVal < 20) newVal = 20;
            OCR2A = newVal;
          }
        }  
        
    }
  }
  delay(1);
  
    
    
}



   
/**
 * Divides a given PWM pin frequency by a divisor.
 *
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 *
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 *
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 *
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1235060559/0#4
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
   
          
