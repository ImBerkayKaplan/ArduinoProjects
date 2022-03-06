#include <bluefruit.h>
#define MANUFACTURER_ID   0x004C 

int buzz_pin = 13;
int led_pin = 3;
uint8_t beaconUuid[16] = 
{ 
  0xFF, 0xFF, 0xFF, 0xFF, 0xDD, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
};

// A valid Beacon packet consists of the following information:
// UUID, Major, Minor, RSSI @ 1M
BLEBeacon beacon(beaconUuid, 0x0000, 0x0000, -54);

void play_frequency(int freq){
    for (int i = 0; i < 100; i++){
      digitalWrite(buzz_pin, HIGH);
      delay(freq);
      digitalWrite(buzz_pin, LOW);
      delay(freq);
    }
}

void beep(){
  play_frequency(3);
  play_frequency(5);
  play_frequency(7);
}

void startAdv(){
    // Manufacturer ID is required for Manufacturer Specific Data
    beacon.setManufacturer(MANUFACTURER_ID);
    pinMode(LED_BUILTIN, OUTPUT);

    // Setup the advertising packet
    Bluefruit.Advertising.setBeacon(beacon);
    // Secondary Scan Response packet (optional)
    // Since there is no room for 'Name' in Advertising packet
    Bluefruit.ScanResponse.addName();
  
    //Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_ADV_NONCONN_IND);
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(160, 160);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
  }

void setup() {
  pinMode(buzz_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);

  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin();
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(0);    // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit52");

  startAdv();
  
  // Start Central Scan
  Bluefruit.setConnLedInterval(250);
  Bluefruit.Scanner.setInterval(1000, 100);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.start(0);

  // Suspend Loop() to save power, since we didn't have any code there
  suspendLoop();
}

void scan_callback(ble_gap_evt_adv_report_t* report){
  int count_match = 0;
  for (int i = 25; i < 31; i++){
    Serial.print(report->data.p_data[i], HEX);
    Serial.print("-");
    if (report->data.p_data[i] == beaconUuid[i - 25]){
        count_match++;
      }
  }
  Serial.println();
  if (count_match == 6){
    digitalWrite(led_pin, HIGH);
    beep();
    digitalWrite(led_pin, LOW);
  }
  
  // For Softdevice v6: after received a report, scanner will be paused
  // We need to call Scanner resume() to continue scanning
  Bluefruit.Scanner.resume();
}

void loop() {
}
