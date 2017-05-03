
#include "brsh.h"
#include <Servo.h>
#include "Arduino.h"

//using namespace std;



void initBrush(int pin,Servo brushless){
	//Servo brushless; // inicializa o objecto servo como variavel global
	// associa a variavel servo a um pin do arduino
	brushless.attach(pin,1000,2000);
}


void turnOn(Servo brushless){
	//liga o brushless e aguarda até ser estavel
	brushless.writeMicroseconds(0);
	brushless.writeMicroseconds(1000);
	delay(5000);
}

void turnOff(Servo brushless){
	//desliga o brushless e aguarda até estar estavel
	brushless.writeMicroseconds(1000);
	brushless.writeMicroseconds(0);
	delay(5000);
}

void defineVelocity(int velocity,Servo brushless){
	//dá uma velocidade e espera que estabilize, valor de percent entre 0 e 650
	if (velocity<0)
		velocity=0;
	if (velocity>100)
		velocity=100;

	velocity=1100+velocity; // soma 1100 à velocidade para ser enviado directamente para o PWM da função SERVO
	brushless.writeMicroseconds(velocity);
	//delay(4000);
}


void revive(int pin, Servo brushless){
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

  
}

void detach(Servo brushless){
	brushless.detach();
}
