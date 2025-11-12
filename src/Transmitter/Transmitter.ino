#include <XBee.h>
#include <SoftwareSerial.h>

SoftwareSerial xbeeSerial(2, 3); // Rx, Tx
XBee xbee;

uint8_t count = 0;
uint8_t payload[] = ">:)";

void setup() {
  Serial.begin(9600);
  xbeeSerial.begin(9600);
  xbee.setSerial(xbeeSerial);
  delay(100);
  Serial.println("XBee sender starting (802.15.4, API)");
}

void loop() {  
  // Tx16Request takes: 16-bit destination, pointer to payload, length
  Tx16Request tx = Tx16Request(0xFFFF, payload, sizeof(payload));
  xbee.send(tx);

  delay(20); // 50 Hz
}
