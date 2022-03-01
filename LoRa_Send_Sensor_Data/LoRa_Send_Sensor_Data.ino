#include <RH_RF95.h>
#include "SparkFunBME280.h"
#include "SparkFun_I2C_GPS_Arduino_Library.h"
#include <TinyGPS++.h>
#include <string>
#include "ICM_20948.h"
#include <cmath>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define CCS811_ADDR 0x5B
#define WIRE_PORT Wire
#define AD0_VAL   1

// Must be 915.0 and match RX's freq!
#define RF95_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


BME280 myBME280; //Object to interface with the Sparkfun Environmental
I2CGPS myI2CGPS; //Object to interface with the Sparkfun GPS
TinyGPSPlus gps; //Declare gps object
ICM_20948_I2C myICM;  // Object to interface with the Sparkfun IMU
SFE_UBLOX_GNSS myGNSS;
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

  // Start the GPS sensor
  if (myI2CGPS.begin() == false) Serial.println("Module failed to respond. Please check wiring.");

  // Start the IMU sensor
  if( myICM.status != ICM_20948_Stat_Ok ) Serial.println( "Could not connect to the IMU sensor for 3D movement." );

  // Start the Environmental Sensor
  if (myBME280.beginI2C() == false) Serial.println("Could not connect to the environmental sensor for temperature.");

  if (myGNSS.begin() == false) Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}
 
void loop(){
  string message = "C848E70850553158342E3120FF19312E";
  
  // Temperature Portion
  /*
  float BMEtempC = myBME280.readTempF();
  string BMEtempCstring= to_string(BMEtempC);
  message = message + ',' + BMEtempCstring.substr(0,BMEtempCstring.length() - 4);
  
  delay(10); //Don't spam the I2C bus
  */

  /*
  // Obtain IMU information
  if( myICM.dataReady() ){
    myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
    message = message + printScaledAGMT(&myICM); //',' + printScaledAGMT(&myICM);
    delay(30);
  }
  
  // Obtain GPS information
  while (myI2CGPS.available()){
    gps.encode(myI2CGPS.read()); //Feed the GPS parser
  }
  if (gps.time.isUpdated()){
    message = message + ',' + GPSInfo();
  }
  
  // Print the message and send it to the receiver Adafruit Feather through LoRa
  Serial.println(message.c_str());
  rf95.send((uint8_t*) message.c_str(), RH_RF95_MAX_MESSAGE_LEN);
  rf95.waitPacketSent();
  */

  double latitude = myGNSS.getLatitude()/10000000.0;
  double longitude = myGNSS.getLongitude()/10000000.0;
  message = message + ',' + to_string(latitude) + " " + to_string(longitude);
  
  Serial.println(message.c_str());
  rf95.send((uint8_t*) message.c_str(), RH_RF95_MAX_MESSAGE_LEN);
  rf95.waitPacketSent();
  delay(500);
  
}

//Display new GPS info
string GPSInfo(){
  string result = "";
  result = to_string(gps.location.lat()) + " " + to_string(gps.location.lng());
  return result;
}

string printScaledAGMT(ICM_20948_I2C *sensor){
  return to_string(sensor->accX()/9800) + " " + to_string(sensor->accY()/9800) + " " + to_string(sensor->accZ()/9800) + " " + to_string(sensor->gyrX()) + " " + to_string(sensor->gyrY()) + " " + to_string(sensor->gyrZ()) + " " + to_string(sensor->magX()) + " " + to_string(sensor->magY()) + " " + to_string(sensor->magZ()) + "," + to_string((sensor->temp()*(9/5)+32));
}
