#include <RH_RF95.h>
#include <string.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
 
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0  
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
 
void setup()
{
  Serial.begin(115200);
  rf95.init();
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);
}
 
void loop(){
  if (rf95.available()){
    // Should be a message for us now
    uint8_t message[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(message);
    
    if (rf95.recv(message, &len)){
      Serial.println((char *) message);
    }
  }
}
