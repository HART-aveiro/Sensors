#include <Thum.h>
  
  Thum thum;                                      // Creates variable of Thum class
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  thum.set();                                     // Starts DHT sensor
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(1000);                                     // Fa = 1 Hz
  float tmp = thum.readtemp();                     // Reads temperature and stores it in tmp
  float humi = thum.readhum();                     // Reads temperature and stores it in humi
  thum.tprint(tmp);                                // Prints the surrounding temperature
  thum.hprint(humi);                               // Prints the surrounding humidity
}