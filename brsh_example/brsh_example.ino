
#include <Servo.h> 
#include "brsh.h"

void setup() {
  brsh motor(9);
  motor.turnOn();
  motor.defineVelocity(10);
  delay(5000);
  motor.turnOff();
}

void loop() {

}
