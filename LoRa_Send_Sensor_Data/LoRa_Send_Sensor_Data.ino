#include <RH_RF95.h>
#include <string>
#include <cmath>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
#include <Wire.h>

// ------------------------------- RFM LoRa Variables ---------------------------------

#define RFM95_CS 8                    
#define RFM95_RST 4                   
#define RFM95_INT 3                  
#define WIRE_PORT Wire                
#define AD0_VAL 1                     
#define RF95_FREQ 915.0               // Must be 915.0 and match RX’s freq!

// ------------------------------- Thermistor Variables -------------------------------

#define THERMISTORPIN 15              // which analog pin to connect
#define THERMISTORNOMINAL 92500       // resistance at 25 degrees C
#define TEMPERATURENOMINAL 22.5       // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 5                  // samples for moving avg filter -- not using rn
#define BCOEFFICIENT 3950             // The beta coefficient of the thermistor
#define SERIESRESISTOR 100000         // the value of the fixed resistor in series

// -------------------------------- Object Interfaces ---------------------------------

//Adafruit_BNO055 bno = Adafruit_BNO055(55);
SFE_UBLOX_GNSS myGNSS;                
RH_RF95 rf95(RFM95_CS, RFM95_INT);    

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
  
  //Use vARF as 3.3v reference signal for Thermistor ADC -- less noise
  analogReference(AR_EXTERNAL);
  
  // Start the I2C protocol
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  // Start GPS Sensor
  if (myGNSS.begin() == false) Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  /*
  //Start the BNO055 IMU
  if (bno.begin()==false) Serial.print(“Could not connect to the IMU sensor for 3D movement.“);
  bno.setExtCrystalUse(true);
  */
}
void loop(){
  delay(1000);
  string message = "";
  
  //Engine & Firefighter ID
  string eid = "01";    //Engine Number
  string fid = "01";    //Fire Fighter Number

  /*
  BNO055 IMU Call event types
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  */
  
  //Add GPS to the message
  double latitude = myGNSS.getLatitude()/10000000.;
  double longitude = myGNSS.getLongitude()/10000000.;
  //-----------engine---------firefighter-------therm temp-----------GNSS lat------------------------GNSS long-----------
  message = eid + ',' + fid + ',' + analog2Temp() + ',' + to_string(latitude) + ',' + to_string(longitude);
  Serial.println(message.c_str());
  rf95.send((uint8_t*) message.c_str(), strlen(message.c_str()));
  rf95.waitPacketSent();
}

string analog2Temp() {
  uint8_t i;
  float average;
  average = analogRead(THERMISTORPIN);
  average = 1023 / average - 1;         // convert the value to resistance
  average = SERIESRESISTOR / average;
  float steinhart;                             // degC
  float farenheit;                             // degF
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C
  farenheit = steinhart*1.8 + 32;              // convert C to F
  //Serial.print(String(steinhart) +“C  ” + String(farenheit) + “F “);
  return to_string(farenheit);
}

/*
string bno2quat() {
  string result = “”;
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Quaternion quat = bno.getQuat();
  result = to_string(quat.w()) + “,” + to_string(quat.x()) + “,” + to_string(quat.y()) + “,” + to_string(quat.z());
  return result;
}


string bno2pos() {
  string result = “”;
  sensors_event_t event;
  bno.getEvent(&event);
  result = to_string(event.orientation.x) + “,” + to_string(event.orientation.y) + “,” + to_string(event.orientation.z);
  return result;
}


string bno2rawData(sensors_event_t* event){
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  string result = “”;
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    //Serial.print(“Accl:“);
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    //Serial.print(“Orient:“);
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    //Serial.print(“Mag:“);
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    //Serial.print(“Gyro:“);
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    //Serial.print(“Rot:“);
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    //Serial.print(“Linear:“);
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    //Serial.print(“Gravity:“);
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  result = to_string(x) + ',' + to_string(y) + ',' + to_string(z);
  return result;
}
*/
