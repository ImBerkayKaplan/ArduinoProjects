#include <RH_RF95.h>
#include <string>
#include <cmath>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <lp55231.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define WIRE_PORT Wire
#define AD0_VAL   1

// Must be 915.0 and match RX's freq!
#define RF95_FREQ 915.0
 
// Instances of the sensors and LoRa
RH_RF95 rf95(RFM95_CS, RFM95_INT);
SFE_UBLOX_GNSS myGNSS;
Lp55231 ledChip;

using namespace std;

void setup() {
  
  // Run a manual reset
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  rf95.init();
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);
  
  // Start the serial communication
  Serial.begin(115200);
  
  // Start the I2C protocol
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  myICM.begin( WIRE_PORT, AD0_VAL );

  // Start the GNSS sensor ()
  if (myGNSS.begin() == false) Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}
 
void loop(){
  string message = "01,01";
  
  double latitude = myGNSS.getLatitude()/10000000.;
  double longitude = myGNSS.getLongitude()/10000000.;
  message = message + ',' + to_string(latitude) + "," + to_string(longitude);
  
  Serial.println(message.c_str());
  rf95.send((uint8_t*) message.c_str(), RH_RF95_MAX_MESSAGE_LEN);
  rf95.waitPacketSent();
}
