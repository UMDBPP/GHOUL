#include <XBee.h>
#include <SoftwareSerial.h>

XBee xbee = XBee();
SoftwareSerial xbeeSerial(8, 9);

uint8_t payload[] = { 't', 'e' , 's', 't'};

// SH + SL Address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x417B4A3B);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

void setup() {
  Serial.begin(9600);
  xbeeSerial.begin(9600);
  xbee.setSerial(xbeeSerial);
}

void loop() {

  xbee.send(zbTx);

  // after sending a tx request, we expect a status response
  // wait up to half second for the status response
  if (xbee.readPacket(500)) {
    // got a response!
    Serial.println("THINGS!");

    // should be a znet tx status
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);

      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS) {
        Serial.println("EvenMore");
        // success.  time to celebrate
      } else {
        // the remote XBee did not receive our packet. is it powered on?
      }
    }
  } else if (xbee.getResponse().isError()) {
    Serial.print("Error reading packet.  Error code: ");
    Serial.println(xbee.getResponse().getErrorCode());
  } else {
    // local XBee did not provide a timely TX Status Response -- should not happen
  }
  delay(1000);
}

