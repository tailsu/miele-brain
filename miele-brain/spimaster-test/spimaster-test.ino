

// include the SPI library:
#include <SPI.h>


// set pin 10 as the slave select for the digital pot:
const int slaveSelectPin = 10;

SPISettings settings(50000, LSBFIRST, SPI_MODE0);

void setup() {
  // set the slaveSelectPin as an output:
  pinMode(slaveSelectPin, OUTPUT);
  // initialize SPI:
  SPI.begin();
}

void loop() {
  for (int i = 0; i < 256; ++i) {
    spiWrite(i, 4);
    delay(1000);
  }
}

void spiWrite(int value, int bytes) {
  // take the SS pin low to select the chip:
  digitalWrite(slaveSelectPin, LOW);
  delay(1);
  SPI.beginTransaction(settings);
  //  send in the address and value via SPI:
  SPI.transfer(reinterpret_cast<void*>(&value), bytes);
  delay(1);
  SPI.endTransaction();
  // take the SS pin high to de-select the chip:
  digitalWrite(slaveSelectPin, HIGH);
  delay(10);
}
