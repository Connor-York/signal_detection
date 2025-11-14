#include <XBee.h>
#include <SoftwareSerial.h>

SoftwareSerial xbeeSerial(2, 3); // Rx, Tx
XBee xbee;

uint8_t count = 0;
uint8_t payload[] = ">:)";
// Tx16Request takes: 16-bit destination, pointer to payload, length
Tx16Request tx = Tx16Request(0xFFFF, payload, sizeof(payload));

unsigned long loop_period = 50;
unsigned long time_check = millis();

void setup() {
  Serial.begin(115200);
  xbeeSerial.begin(115200);
  xbee.setSerial(xbeeSerial);
  delay(100);
  Serial.println("XBee sender starting (802.15.4, API)");
}

void loop() {  

  unsigned long now = millis();
  if (now - time_check >= loop_period) {
    xbee.send(tx);  
    time_check = now;
  }
}