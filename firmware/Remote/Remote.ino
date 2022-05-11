// RFM95_CS = 8   Does not have a built in pullup resistor.
// RFM95_RST = 4
//RFM95__INT = 3

#include <Wire.h>
#include <RH_RF95.h>
#include "common.h"


#define DISPLAY_ADDRESS 0x72

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define STOP_PIN 10
#define MOBSTOP_PIN 11
#define START_PIN 12
#define RF95_FREQ 900.0 // RFM95 operates at 900 MHz

#define LED 13    // used to blink for status

RH_RF95 rf95(RFM95_CS, RFM95_INT);

volatile char sendSig = 0;

bool isCon = false;
char led = 0;
unsigned long lastAck = 0;
unsigned long lastBeat = 0;
unsigned long lastDisplay = 0;
unsigned long lastKill = 0;

RadioPacket toSend;

void stopInterrupt()
{
  if (sendSig == 0)
    sendSig = 1;
}

void mobStopInterrupt()
{
  if (sendSig == 0)
    sendSig = 2;
}

void startInterrupt()
{
  if (sendSig == 0)
    sendSig = 3;
}



void setup() {

  Wire.begin();
  Serial.begin(115200);

  // Clear the LCD screen
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write('|'); //Put LCD into setting mode
  Wire.write('-'); //Send clear display command
  Wire.endTransmission();


  pinMode(13, OUTPUT);
  digitalWrite(13, led);

  // reset rfm95
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // init rfm95
  rf95.init();
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);

  // button interrupt setup
  pinMode(STOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopInterrupt, FALLING);

  pinMode(MOBSTOP_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MOBSTOP_PIN), mobStopInterrupt, FALLING);

  pinMode(START_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(START_PIN), startInterrupt, FALLING);

}

void updateDisplay() {
  Wire.beginTransmission(DISPLAY_ADDRESS);
  Wire.write('|'); //Put LCD into setting mode
  Wire.write('-'); //Send clear display command
  Wire.endTransmission();

  if (sendSig > 0) {
    Wire.beginTransmission(DISPLAY_ADDRESS);
    Wire.print("Sending: ");
    // add printed signal that is being sent.
    Wire.endTransmission();
    return;
  }

  if (isCon) {
    Wire.beginTransmission(DISPLAY_ADDRESS);
    Wire.print("Connected.");
    Wire.endTransmission();
  }
  else {
    Wire.beginTransmission(DISPLAY_ADDRESS);
    Wire.print("Waiting...");
    Wire.endTransmission();
  }
  
}

void loop() {
  
  if (rf95.available()) {

    led = !led;
    digitalWrite(13, led);

    uint8_t buf[sizeof(RadioPacket)];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      
      RadioPacket msg = *(RadioPacket*)buf;

      if (!isCon && msg.id == 1) {
        lastAck = millis();
        lastBeat = millis();
        isCon = true;
        toSend.id = 2;
        rf95.send((uint8_t*)&toSend, sizeof(toSend));
        rf95.waitPacketSent();
      }

      if (isCon && msg.id == 4) {
        lastAck = millis();
      }

      if (isCon && msg.id == 9) {
        lastAck = millis();
        sendSig = 0;
      }
      
    }
  }

  // Mark as disconnected if we haven't received an acknowledge in a while
  if (isCon && (millis() - lastAck) > 5000) {
    isCon = false;
    sendSig = 0;
  }

  // Send a signal if we have one waiting
  if (isCon && sendSig > 0 && (millis() - lastKill) > 300) {
    lastKill = millis();
    toSend.id = 0;
    toSend.message[0] = sendSig;

    rf95.send((uint8_t*)&toSend, sizeof(toSend));
    rf95.waitPacketSent();
  }

  // Send a hearbeat periodically if we haven't received an acknowledge in a while
  if (isCon && sendSig == 0 && (millis() - lastAck) > 500 && (millis() - lastBeat) > 300) {
    lastBeat = millis();
    toSend.id = 3;
    rf95.send((uint8_t*)&toSend, sizeof(toSend));
    rf95.waitPacketSent();
  }

  // update display periodically
  if ((millis() - lastDisplay) > 800) {
    lastDisplay = millis();
    updateDisplay();
  }

  

  
















  
}
