
#include "brsh.h"
#include <PWMServo.h>
#include "Arduino.h"


void initBrush(int pin,PWMServo brushless){
	
	brushless.attach(pin,1000,2000); // associa a variavel servo a um pin do arduino
}


void turnOn(PWMServo brushless){
  brushless.write(0);
  delay(5000);
}

void turnOff(PWMServo brushless){
  brushless.write(0);
  delay(5000);
}

void defineVelocity(int velocity,PWMServo brushless){
	if (velocity<0)
		velocity=0;
	if (velocity>100)
		velocity=100;
 
 velocity=1100+velocity; // soma 1100 à velocidade para ser enviado directamente para o PWM da função SERVO
 velocity=map(velocity,1000,2000,0,180);
 brushless.write(velocity);
}


/*void revive(int pin, PWMServo brushless){
	brushless.detach();
	delay(1000);
  // Set first the position to prevent the servo to move to the middle
  // position at initialisation
  brushless.writeMicroseconds(2000);  // Full power
  
  // Only now attach the servo. When passing the proper
  // minimum and maximum, you will be able to use the degrees
  // for write. I assumed the standard range from 1000 to 2000
  // microseconds.
  brushless.attach(pin, 1000, 2000);
  
  // Wait for beep from ESC
  delay(10000);
  
  brushless.writeMicroseconds(1000);  // Power off
  
  // Wait for beep from ESC
  delay(10000);

  
}*/

void detach(PWMServo brushless){
	brushless.detach();
}
