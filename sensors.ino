#include <Arduino.h>
#include <RingBuf.h>
//#include <TimerThree.h>
#include <TimerOne.h>
//#include "hartimu.h"
#include <Wire.h>
//#include <DHT.h>
#include <PWMServo.h> 
//#include "brsh.h"
//#include "flamesensor.h"
#include <LIDARLite.h>

//Things to change
#define DEBUG 1
//to exit debug mode delete previous line ( #define DEBUG 1 )

#define UART_BAUDRATE 2000000
#define INT_PERIOD 1000
#define MAX_TIME_COUNT 90000 //90 segundos

#define pinSTARTlidar 7
//#define pinPRINTlidar 7
#define pinSAVElidar 8

////////////////

#define idLIDAR 0x01

//Servo
#define pinServo 9
#define lowAngle 60
#define upAngle 140

int canDo;
int sDirection=0;
int pos = lowAngle, lastPos;

#ifdef DEBUG
//Debug LEDs
#define L1 11 //DHT
#define L2 31 //MPU
#define L3 32 //Brushless
#define L4 33 //MQ7
#define L5 34 //FIRE
#define L6 35 //LIDAR
#define L7 36 ////ERROR/////////////////////////
#define L8 37 ////OK////////////////////////////
#endif

//BUFFERS
//RingBuf *bufMPU = RingBuf_new(sizeof(short), 21);
RingBuf *bufLIDAR = RingBuf_new(sizeof(short), 3000);
//RingBuf *bufDHT = RingBuf_new(sizeof(byte), 8);
//RingBuf *bufFLAMES = RingBuf_new(sizeof(byte), 8);
//RingBuf *bufMQ7 = RingBuf_new(sizeof(byte), 8);

//INITS
//LIDAR
LIDARLite myLidarLite;

int startLIDAR=0;
int printLIDARdata=0;
int saveLIDAR=0;

int flagLIDAR=0;

short distance=0;
int numPointsLIDAR, numPrintLidar;
int printNum=0;
int toggle2=0;

// Read distance. The approach is to poll the status register until the device goes
// idle after finishing a measurement, send a new measurement command, then read the
// previous distance data while it is performing the new command.
int distanceFast(bool biasCorrection){
	interrupts();
	byte isBusy = 1;
	int distance;
	int loopCount=0;

  	// Poll busy bit in status register until device is idle
	while(isBusy){
    // Read status register
		Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
		Wire.write(0x01);
		Wire.endTransmission();
		Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
		isBusy = Wire.read();
		isBusy = bitRead(isBusy,0); // Take LSB of status register, busy bit
		loopCount++; // Increment loop counter
		// Stop status register polling if stuck in loop
		if(loopCount > 9999){
			break;  
		}
	}

  	// Send measurement command
	Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
	Wire.write(0X00); // Prepare write to register 0x00
	if(biasCorrection == true){
	    Wire.write(0X04); // Perform measurement with receiver bias correction
	}else{
	    Wire.write(0X03); // Perform measurement without receiver bias correction
	}
	Wire.endTransmission();

  	// Immediately read previous distance measurement data. This is valid until the next measurement finishes.
  	// The I2C transaction finishes before new distance measurement data is acquired.
  	// Prepare 2 byte read from registers 0x0f and 0x10
	Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
	Wire.write(0x8f);
	Wire.endTransmission();

  	// Perform the read and repack the 2 bytes into 16-bit word
	Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 2);
	distance = Wire.read();
	distance <<= 8;
	distance |= Wire.read();

  	// Return the measured distance
	return distance;
}


void setup(void){
  #ifdef DEBUG
    //Define debug LEDpins as output///////////
	pinMode(L1,OUTPUT);
	pinMode(L2,OUTPUT);
	pinMode(L3,OUTPUT);
	pinMode(L4,OUTPUT);
	pinMode(L5,OUTPUT);
	pinMode(L6,OUTPUT);
	pinMode(L7,OUTPUT);
	pinMode(L8,OUTPUT);
	digitalWrite(L1,LOW);
	digitalWrite(L2,LOW);
	digitalWrite(L3,LOW);
	digitalWrite(L4,LOW);
	digitalWrite(L5,LOW);
	digitalWrite(L6,LOW);
	digitalWrite(L7,LOW);
	digitalWrite(L8,LOW);
	pinMode(13,OUTPUT);
	digitalWrite(13,LOW);
    ///////////////////////////////////////////
  #endif
	pinMode(pinSAVElidar, INPUT);
	pinMode(pinSTARTlidar, INPUT);
	//pinMode(pinPRINTlidar, INPUT);

	//Serial initialization////////////////////////
	Serial.begin(UART_BAUDRATE);

	//Lidar stuff

	  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz

	  myLidarLite.write(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
	  myLidarLite.write(0x04, 0b00000100); // Use non-default reference acquisition count
	  myLidarLite.write(0x12, 0x03); // Reference acquisition count of 3 (default is 5)

}
/////////////////////////////////////end setup

union sendShort{    //definition of data typre to be able to separate data bytes
	short send1;
	byte send2[2];
}sendSHORT;
int test =0;
void loop(void){////////////////////////////////////////////////////////////////////////////
	#ifdef DEBUG
		/*if(printLIDARdata==1)
			digitalWrite(13,HIGH);
		else
			digitalWrite(13,LOW);*/
		//digitalWrite(13,digitalRead(pinSAVElidar));

	#endif

	/*
	//Record num of points to print
	//saveLIDAR=digitalRead(pinSAVElidar);
		if(digitalRead(pinSAVElidar)==HIGH){
			saveLIDAR=1;
			//digitalWrite(13,HIGH);
		}else{
			saveLIDAR=0;
			//digitalWrite(13,LOW);
		}



	if(saveLIDAR==1 && printLIDARdata==0){
		numPrintLidar=numPointsLIDAR;
		printLIDARdata=1;
		printNum=numPrintLidar;
		//Serial.write(printNum);
	}
*/
	//Print section
	//printLIDARdata=digitalRead(pinPRINTlidar);
	/*if( printLIDARdata==1){
		if(numPrintLidar > 100){
			digitalWrite(13,HIGH);
			if(Serial.availableForWrite() > 16){// numPrintLidar){

				if(printNum==0){
					printLIDARdata=0;
					//Serial.write(0x00);

					//Serial.write(0x00);
					
					//Serial.write(idLIDAR);
				}else{
					printNum--;
				}

				bufLIDAR->pull(bufLIDAR, &sendSHORT);
				digitalWrite(13,LOW);
				Serial.println(sendSHORT.send1);
			}
		}
	}*/

	if(digitalRead(pinSTARTlidar)==HIGH){
			startLIDAR=1;
			digitalWrite(13,HIGH);
		}else{
			startLIDAR=0;
			digitalWrite(13,LOW);
	}

	//Measurement section
	//startLIDAR=digitalRead(pinSTARTlidar);
	//if(startLIDAR==1){
		// if(flagLIDAR=1){
		digitalWrite(L1,HIGH);
		//flagLIDAR=0;//is controllled by an interrupt
		//if(numPointsLIDAR%100==0)
		//	distance = (distanceFast(true));
		//else
			distance = (distanceFast(false));


		//Serial.println(distance);
		test = distance;
		//RingBufAdd(bufLIDAR,&distance);
		bufLIDAR->add(bufLIDAR,&test);
	//	numPointsLIDAR++;
		delay(10000);
	//	distance=0;
		//bufLIDAR->pull(bufLIDAR,&distance);
		Serial.println(distance);


		digitalWrite(L1,LOW);
		//delay(10);
	//}


	
		//}
	// Take a measurement with receiver bias correction and print to serial terminal
  /*Serial.println(distanceFast(true));

  // Take 99 measurements without receiver bias correction and print to serial terminal
  for(int i = 0; i < 99; i++)
  {
    Serial.println(distanceFast(false));
  }*/

	
}



