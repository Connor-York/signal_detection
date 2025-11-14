#include <XBee.h>
#include <SPI.h>

XBee xbee;

Rx16Response rx16 = Rx16Response();

int8_t xbee_rssi = -999;

unsigned long time_check = millis(); 
  
void setup() {

  Serial.begin(115200);

  Serial1.begin(115200); // XBee
  delay(100);

  xbee.setSerial(Serial1);

}

void read_xbee(){
  xbee.readPacket();                         // wait up to 100 ms
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      //Serial.print(rx16.getRssi(), DEC); Serial.print("\n");
      xbee_rssi = rx16.getRssi();
      Serial.print("-"); Serial.print(xbee_rssi, DEC); Serial.print(","); Serial.print(millis() - time_check); Serial.print("\n");
      // Serial.println("======");
      // Serial.println(millis() - time_check);
      time_check=millis();
      // xbee_rssi = rx16.getRssi();
    }
  }
}


void loop() {

  read_xbee();
}
