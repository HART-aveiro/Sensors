#include <flamesensor.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int s1 = getFS1values();
  int s2 = getFS2values();
  int s3 = getFS3values();
  flameposition(s1,s2,s3);
}
