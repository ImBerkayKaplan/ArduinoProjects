#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include "SparkFunCCS811.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define CCS811_ADDR 0x5B //Default I2C Address
CCS811 mySensor(CCS811_ADDR);

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
  Serial.println("This is a test");
  Wire.begin(); //Inialize I2C Hardware
  if (mySensor.begin() == false){
    Serial.print("CCS811 error. Please check wiring. Freezing...");
    while (1);
  }
}
 
void loop()
{

  //Check to see if data is ready with .dataAvailable()
  if (mySensor.dataAvailable()){
    mySensor.readAlgorithmResults();
    Serial.println(mySensor.getTemperature());
    uint8_t data[] = "0011";
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
  delay(10); //Don't spam the I2C bus
}
