#include <XBee.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// SoftwareSerial LoraSerial(2,3); //Rx,Tx

XBee xbee;

Rx16Response rx16 = Rx16Response();

int8_t xbee_rssi = 0;
int8_t lora_rssi = 0;

unsigned long time_check = millis();
  
void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);

  Serial1.begin(9600); // XBee
  delay(100);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    // while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }
  // rf95.setTxPower(23, false);
  xbee.setSerial(Serial1);

}

void read_xbee(){
  xbee.readPacket(100);                         // wait up to 100 ms
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      // Serial.print(rx16.getRssi(), DEC); Serial.print("\n");
      xbee_rssi = rx16.getRssi();
    }
  }
}

void read_lora(){
  if (rf95.available()){
        // Should be a message for us now   
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        if (rf95.recv(buf, &len)){
          lora_rssi = rf95.lastRssi();
        }
      }
}

void loop() {
  read_xbee();
  read_lora();

  if (millis() - time_check >= 50) {
      Serial.print("-"); Serial.print(xbee_rssi, DEC); Serial.print(","); Serial.print(lora_rssi, DEC); Serial.print("\n");
      time_check = millis();
  }
}
