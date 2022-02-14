#include <RH_RF95.h>
#include "SparkFunBME280.h"
#include "SparkFun_I2C_GPS_Arduino_Library.h"
#include <TinyGPS++.h>
#include <string>
#include "ICM_20948.h"
#include <cmath>

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

  // Start the GPS sensor
  if (myI2CGPS.begin() == false) Serial.println("Module failed to respond. Please check wiring.");
  
  // Start the IMU sensor
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  myICM.begin( WIRE_PORT, AD0_VAL );
  if( myICM.status != ICM_20948_Stat_Ok ) Serial.println( "Could not connect to the IMU sensor for 3D movement." );

  // Start the Environmental Sensor
  if (myBME280.beginI2C() == false) Serial.println("Could not connect to the environmental sensor for temperature.");
}
 
void loop(){
  string message = "CD5857F750553158342E3120FF172906";
  
  // Temperature Portion
  float BMEtempC = myBME280.readTempF();
  string BMEtempCstring= to_string(BMEtempC);
  message = message + ',' + BMEtempCstring.substr(0,BMEtempCstring.length() - 4);
  
  delay(10); //Don't spam the I2C bus

  // Obtain IMU information
  if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
    printScaledAGMT(&myICM);   // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
  }
  
  // Obtain GPS information
  while (myI2CGPS.available()){
    gps.encode(myI2CGPS.read()); //Feed the GPS parser
  }
  if (gps.time.isUpdated()){
    message = message + ',' + GPSInfo();
    
  }

  // Print the message and send it to the receive Adafruit Feather through LoRa
  Serial.println(message.c_str());
  rf95.send((uint8_t*) message.c_str(), 50);
  rf95.waitPacketSent();
  Serial.println();
  
  delay(100);
}


//Display new GPS info
string GPSInfo(){
  string result = "";
  if (gps.location.isValid()){
    result = to_string(gps.location.lat()) + " " + to_string(gps.location.lng());
  }
  return result;
}

void printScaledAGMT(ICM_20948_I2C *sensor){
  Serial.print("Scaled. Acc (mg) [");
  printFormattedFloat(sensor->accX()/9800, 5, 2);
  Serial.print(" ");
  printFormattedFloat(sensor->accY()/9800, 5, 2);
  Serial.print(" ");
  printFormattedFloat(sensor->accZ()/9800, 5, 2);
  Serial.print("], Gyr (DPS) [");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  Serial.print(" ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial.print(" ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial.print("], Mag (uT) [");
  printFormattedFloat(sensor->magX(), 5, 2);
  Serial.print(" ");
  printFormattedFloat(sensor->magY(), 5, 2);
  Serial.print(" ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  Serial.print("], Tmp (C) [");
  printFormattedFloat(sensor->temp(), 5, 2);
  Serial.println("]");
  Serial.println((sqrt(pow(sensor->accX()/9800, 2) + pow(sensor->accY()/98000, 2) + pow(sensor->accZ()/98000, 2))/2)*pow(1,2));
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if (val < 0){
    Serial.print("-");
  }
  else{
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++){
    uint32_t tenpow = 0;
    if (indi < (leading - 1)){
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++){
      tenpow *= 10;
    }
    if (aval >= tenpow){
      break;
    }
  }
  if (val < 0){
    Serial.print(-val, decimals);
  }
  else{
    Serial.print(val, decimals);
  }
}
