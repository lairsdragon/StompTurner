#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"


#define VERBOSE_MODE                   true  // If set to 'true' enables debug output
#define MINIMUM_FIRMWARE_VERSION       "0.6.6"

// HARDWARE SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for HW SPI communication.
// SCK, MISO and MOSI should be connected to the HW SPI pins on the Uno, etc.
// This should be used with nRF51822 based Bluefruit LE modules that use SPI.
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_CS               8
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              4    // Optional but recommended, set to -1 if unused

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


#define VBATPIN A7
#define VBATTIME 5*1000

// https://cdn-learn.adafruit.com/assets/assets/000/046/245/original/adafruit_products_Feather_M0_Bluefruit_v2.2-1.png?1504885440
// https://learn.adafruit.com/adafruit-feather-m0-bluefruit-le?view=all#pinouts

// Debounce
// https://forum.arduino.cc/index.php?topic=45000.0
#define DEBOUNCETIME 500

#define BUTTON1 A5
#define BUTTON2 A4

int           button1_pressed=0;
unsigned long button1_lastirq=0;

int           button2_pressed=0;
unsigned long button2_lastirq=0;

unsigned long vbat_last;

void button1()
{
  unsigned long irq_ts = millis();
  
  int state = digitalRead(BUTTON1);

    // Debounce the button 
  if (irq_ts - button1_lastirq > DEBOUNCETIME) {  
    // interpret only the button Press
    if (state == LOW) {
      button1_pressed=1;
    }
  }
  
  button1_lastirq=irq_ts;
}

void button2()
{
  unsigned long irq_ts = millis();
  
  // Debounce the button 
  if (irq_ts - button2_lastirq > DEBOUNCETIME) {
    // interpret only the button Press
    if (digitalRead(BUTTON2) == LOW) {
      button2_pressed=1;
    }
  }
  button2_lastirq=irq_ts;
}

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1)
  ;
}
// the setup function runs once when you press reset or power the board
void setup() {
  delay(1000);
  // while(!Serial);
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit HID Keyboard Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'Bluefruit Keyboard': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=StompTurner" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
      error(F("Could not enable Keyboard"));
    }
  }else
  {
    if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  ))) {
      error(F("Could not enable Keyboard"));
    }
  }
  if (! ble.sendCommandCheckOK(F( "AT+BLEBATTEN=On"  ))) {
    error(F("Could not enable BatteryService"));
  }
  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  pinMode(BUTTON1,INPUT);
  pinMode(BUTTON2,INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON1), button1, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON2), button2, FALLING);
}

void read_battery()
{
  String battpercent;
   
  analogReadResolution(10);     
  int meas = analogRead(VBATPIN);
  float measuredvbat = meas;
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.print(measuredvbat);
  Serial.print(" / " ); Serial.println(meas);
  battpercent="AT+BLEBATTVAL=";
  if (meas > 652) {
    battpercent += "100";
  } else if (meas > 641) {
    battpercent += "90";
  } else if (meas > 630) {
    battpercent += "80";
  } else if (meas > 619) {
    battpercent += "70";
  } else if (meas > 608) {
    battpercent += "60";
  } else if (meas > 597) {
    battpercent += "50";
  } else if (meas > 586) {
    battpercent += "40";    
  } else if (meas > 576) {
    battpercent += "30";    
  } else if (meas > 565) {
    battpercent += "20";    
  } else if (meas > 554) {
    battpercent += "10";     
  } else {
    battpercent += "0"; 
  }
  if ( !ble.sendCommandCheckOK(battpercent.c_str() )) {
    Serial.println("Failed Sendkey");
  }
}

// the loop function runs over and over again forever
void loop() {

  if (vbat_last+VBATTIME < millis()) {
    read_battery();
    vbat_last=millis();  
  }
  
  if (button1_pressed) {
    Serial.print("Button1 pressed (");
    Serial.print(millis());
    Serial.println(")");
    if ( !ble.sendCommandCheckOK(F( "AT+BLEKEYBOARDCODE=00-00-4B-00-00-00-00" ))) {
      Serial.println("Failed Sendkey");
    }
    if ( !ble.sendCommandCheckOK(F( "AT+BLEKEYBOARDCODE=00-00" ))) {
      Serial.println("Failed Sendkey");
    }
    button1_pressed=0;    
  }
  if (button2_pressed) {
    Serial.print("Button2 pressed (");
    Serial.print(millis());
    Serial.println(")");
    if ( !ble.sendCommandCheckOK(F( "AT+BLEKEYBOARDCODE=00-00-4E-00-00-00-00" ))) {
      Serial.println("Failed Sendkey");
    }
    if ( !ble.sendCommandCheckOK(F( "AT+BLEKEYBOARDCODE=00-00" ))) {
      Serial.println("Failed Sendkey");
    }
    button2_pressed=0;
  }
}
