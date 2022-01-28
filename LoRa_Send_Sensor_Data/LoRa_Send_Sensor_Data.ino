#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <SparkFunBME280.h>
#include <SparkFunCCS811.h>
#include "SparkFun_I2C_GPS_Arduino_Library.h"
#include <TinyGPS++.h> //From: https://github.com/mikalhart/TinyGPSPlus

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define CCS811_ADDR 0x5B //Default I2C Address

// Change to 915.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

CCS811 myCCS811(CCS811_ADDR);
BME280 myBME280;
I2CGPS myI2CGPS; //Hook object to the library
TinyGPSPlus gps; //Declare gps object

void setup() {
  
  // manual reset
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  rf95.init();

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);
  
  Serial.begin(115200);
  Serial.println("Start Comm");
  Wire.begin(); //Inialize I2C Hardware
  myI2CGPS.begin();
  
  //This begins the CCS811 sensor and prints error status of .beginWithStatus()
  CCS811Core::CCS811_Status_e returnCode = myCCS811.beginWithStatus();
  Serial.print("CCS811 begin exited with: ");
  Serial.println(myCCS811.statusString(returnCode));
  initialize_BME280();

  
}
 
void loop(){
  //Check to see if data is ready with .dataAvailable()
  if (myCCS811.dataAvailable()){
    myCCS811.readAlgorithmResults();
    float BMEtempC = myBME280.readTempC();
    while (myI2CGPS.available()){
      gps.encode(myI2CGPS.read()); //Feed the GPS parser
    }
    if (gps.time.isUpdated()){ //Check to see if new GPS info is available
      displayInfo();
    }
    uint8_t *data = (uint8_t*)(&BMEtempC);
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
  delay(10); //Don't spam the I2C bus
}

//Initialize BME280
void initialize_BME280(){
    
  //For I2C, enable the following and disable the SPI section
  myBME280.settings.commInterface = I2C_MODE;
  myBME280.settings.I2CAddress = 0x77;
  myBME280.settings.runMode = 3; //Normal mode
  myBME280.settings.tStandby = 0;
  myBME280.settings.filter = 4;
  myBME280.settings.tempOverSample = 5;
  myBME280.settings.pressOverSample = 5;
  myBME280.settings.humidOverSample = 5;

  //Calling .begin() causes the settings to be loaded
  delay(10); //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.
  myBME280.begin();
}

//Display new GPS info
void displayInfo(){
  //We have new GPS data to deal with!
  Serial.println();

  if (gps.location.isValid()){
    Serial.print("Location: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(", "));
    Serial.print(gps.location.lng(), 6);
    Serial.println();
  }
  else
  {
    Serial.println(F("Location not yet valid"));
  }
}
