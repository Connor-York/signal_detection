#include <XBee.h>
#include <SPI.h>

XBee xbee;

Rx16Response rx16 = Rx16Response();

int8_t xbee_rssi = -999;
int transmission_timeout = 60; // expecting messages at 50ms intervals, if over (by 5ms) assume missed 

//time_check resets every 50ms ish, 
//time_check 2 resets after every real reading.
unsigned long time_check = millis(); 
unsigned long time_check2 = millis(); 
  
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
      Serial.print("-"); Serial.print(xbee_rssi, DEC); Serial.print(","); Serial.print(millis() - time_check2); Serial.print("\n");
      // Serial.println("======");
      // Serial.println(millis() - time_check);
      time_check=millis();
      time_check2=millis(); // This one only resets after a real reading
      // xbee_rssi = rx16.getRssi();
    }
  }

  if (millis() - time_check >= transmission_timeout) {
    // If package hasn't been received in the expected transmission time assume dropped packet
    // and return current rssi
    if (xbee_rssi != -999) { // If no reading had yet just ignore
      Serial.print("-"); Serial.print(xbee_rssi, DEC); Serial.print(","); Serial.print(millis() - time_check2); Serial.print("\n");
      // Serial.println("======");
      // Serial.println("Missed");
      // Serial.println(millis() - time_check);
      time_check=millis();
    }
  }
}


void loop() {

  read_xbee();
}
