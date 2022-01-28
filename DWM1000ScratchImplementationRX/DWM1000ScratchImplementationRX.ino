#include <SPI.h>

const int ss = 10;

void write_to_register(byte *header, int header_size_bytes, byte *data_to_write, int data_size_bytes) {
  // take the chip select low to select the device:
  digitalWrite(ss, LOW);
  for(int i = 0; i < header_size_bytes; i++){
    SPI.transfer(header[i]);
  }

  for(int i = 0; i < data_size_bytes; i++){
    SPI.transfer(data_to_write[i]);
  }
  // take the chip select high to de-select:
  digitalWrite(ss, HIGH);
}

void read_from_register(byte *header, int header_size_bytes, int register_data_size_bytes) {
  // take the chip select low to select the device:
  digitalWrite(ss, LOW);
  for(int i = 0; i < header_size_bytes; i++){
    SPI.transfer(header[i]);
  }

  for(int i = 0; i < register_data_size_bytes; i++){
    Serial.println(SPI.transfer(0), HEX);
  }
  // take the chip select high to de-select:
  digitalWrite(ss, HIGH);
  Serial.println();
}

void setup() {
  Serial.begin(9600);
  pinMode(ss, OUTPUT);
  SPI.begin();

  byte rx_buffer_read_header[] = {0x11};

  read_from_register(rx_buffer_read_header, 1, 127);
}

void loop() {
  
}
