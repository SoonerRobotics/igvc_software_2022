// RFM95_CS = 8   Does not have a built in pullup resistor.
// RFM95_RST = 4
//RFM95__INT = 3

#include <mcp_can.h>
#include <RH_RF95.h>
#include "common.h"

#define DISPLAY_ADDRESS 0x72

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 900.0 // RFM95 operates at 900 MHz
#define LED 13    // used to blink for status
#define STOP_SIG 11

#define CAN0_INT 6
MCP_CAN CAN0(5);

RH_RF95 rf95(RFM95_CS, RFM95_INT);

bool isCon = false;
char led = 0;
bool killed = false;
unsigned long lastAck = -1;
unsigned long lastHandshake = -1;

RadioPacket toSend;


void setup() {

  Serial.begin(115200);

  //init led
  pinMode(LED, OUTPUT);
  digitalWrite(LED, led);

  pinMode(STOP_SIG, OUTPUT);
  digitalWrite(STOP_SIG, HIGH);

  delay(2000);

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

   // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  // Since we do not set NORMAL mode, we are in loopback mode by default.
  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT);                           // Configuring pin for /INT input

  lastHandshake = millis();
 

}

void loop() {

  if (rf95.available()) {

    led = !led;
    digitalWrite(LED, led);

    uint8_t buf[sizeof(RadioPacket)];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {

      RadioPacket msg = *(RadioPacket*)buf;

      // Received a signal
      if (msg.id == 0) {
        // Send signal to CAN
        byte data[] = {};

        // Kill
        if (msg.message[0] == 1) {
          digitalWrite(LED, HIGH);
          digitalWrite(STOP_SIG, LOW);

          CAN0.sendMsgBuf(0x0, 0, data);

          isCon = false;
          killed = true;
          Serial.println("Received Kill signal");
        }

        // Mobility Stop
        if (msg.message[0] == 2) {
          CAN0.sendMsgBuf(0x1, 0, data);
          Serial.println("Received Mobility Stop message.");
        }

        // Mbility Start
        if (msg.message[0] == 3) {
          CAN0.sendMsgBuf(0x9, 0, data);
          Serial.println("Received Mobility Start message.");
        }

        toSend.id = 9;
        lastAck = millis();
        rf95.send((uint8_t*)&toSend, sizeof(toSend));
        rf95.waitPacketSent();
        
      }

      // Handshake Ack
      if (!isCon && msg.id == 2) {
        isCon = true;
        lastAck = millis();
      }

      // Heartbeat
      if (isCon && msg.id == 3) {
        toSend.id = 4; // Respond with Ack
        lastAck = millis();
        rf95.send((uint8_t*)&toSend, sizeof(toSend));
        rf95.waitPacketSent();
      }
    }
  }

   // Send handshake requests periodically if we aren't isConnected
  if (!isCon && !killed && (millis() - lastHandshake) > 400) {
    lastHandshake = millis();
    toSend.id = 1;
    rf95.send((uint8_t*)&toSend, sizeof(toSend));
    rf95.waitPacketSent();
  }

  // Kill robot if we haven't heard from robot in 5 seconds
  if (isCon && (millis() - lastAck) > 5000) {
    digitalWrite(LED, HIGH);
    digitalWrite(STOP_SIG, LOW);
    // digitalWrite(STOP_SIG, HIGH);
    isCon = false;
    killed = true;
  }

  

}
