#include "brsh.h"
#include <Servo.h>
#include "Arduino.h"

using namespace std;

Servo brushless; // inicializa o objecto servo como variavel global

brsh::brsh(int pin){
	// associa a variavel servo a um pin do arduino
	brushless.attach(pin);
}


void brsh::turnOn(void){
	//liga o brushless e aguarda até ser estavel
  brushless.writeMicroseconds(0);
	brushless.writeMicroseconds(1000);
	delay(6000);
}

void brsh::turnOff(void){
	//desliga o brushless e aguarda até estar estavel
  brushless.writeMicroseconds(1000);
	brushless.writeMicroseconds(0);
	delay(6000);
}

void brsh::defineVelocity(int percent){
	//dá uma velocidade e espera que estabilize
	if (percent<1)
	percent=1;
	if (percent>100)
	percent=100;

	int velocity=1100+0.75*percent;
  brushless.writeMicroseconds(velocity);
	//delay(4000);
}
