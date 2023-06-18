#include <SD.h>
#define BAUD 921600
#define IMUSerial Serial1
#define IMUInterrupt 3
#define SDChipSelect BUILTIN_SDCARD
unsigned long timestamp_micros = 0;
void setup() {
  IMUSerial.begin(BAUD);
  attachInterrupt(digitalPinToInterrupt(IMUInterrupt), timestamp, FALLING);
  while (!SD.begin(SDChipSelect)) {
    delay(1000);
  }
}
void loop() {
  char currentMessage[100]; //should be way more than needed, maximum message size is 50 bytes
  unsigned int i = 0;
  while(!IMUSerial.available()) {}
  while(IMUSerial.available()) {
    currentMessage[i] = (char)IMUSerial.read();
    i++;
  }
  File datalogFile = SD.open("log.txt", FILE_WRITE);
  datalogFile.println(String(timestamp_micros)+": "+String(currentMessage));
  datalogFile.close();
}
void timestamp() {
  timestamp_micros = micros();
}