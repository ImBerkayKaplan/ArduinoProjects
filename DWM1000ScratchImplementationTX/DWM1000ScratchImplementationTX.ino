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

  byte tx_conf_data[] = {0x7F, 0x40, 0x00, 0x00, 0x00};
  byte tx_conf_write_header[] = {0x88};
  byte tx_conf_read_header[] = {0x08};
  byte txstart[1] = {0x02};
  byte tx_buffer[127] = {0};
  byte tx_buffer_write_header[] = {0x89};
  byte tx_buffer_read_header[] = {0x09};
  byte tx_start_write_header[] = {0x8D};
  byte tx_start_read_header[] = {0x0D};
  
  
  //Serial.println(tx_conf.size());
  write_to_register(tx_conf_write_header, 1, tx_conf_data, 5);
  write_to_register(tx_buffer_write_header, 1, tx_buffer, 127);
  write_to_register(tx_start_write_header, 1, txstart, 1);

  read_from_register(tx_conf_read_header, 1, 5);
  read_from_register(tx_buffer_read_header, 1, 127);
  read_from_register(tx_start_read_header, 1, 1);
}

void loop() {
  
}
