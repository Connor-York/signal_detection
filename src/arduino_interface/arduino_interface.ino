#include<XBee.h>

// SoftwareSerial xbeeSerial(2,3); //Rx,Tx

XBee xbee;

Rx16Response rx16 = Rx16Response();



int exp_dist = 0;


  
void setup() {

  Serial.begin(9600);

  Serial1.begin(9600); // XBee

  xbee.setSerial(Serial1);

}

void read_xbee(){
  xbee.readPacket(100);                         // wait up to 100 ms
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      Serial.print(rx16.getRssi(), DEC); Serial.print("\n");
    }
  }
}

void loop() {
  read_xbee();
}
