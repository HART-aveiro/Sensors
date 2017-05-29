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
//#define idDHT 0x02
//#define idMPU 0x03
//#define idMQ7 0x05
//#define idFLAMES 0x04

//PhotoDiode
//#define pinPhotoDiode 3

//Servo
#define pinServo 9
#define lowAngle 60
#define upAngle 140

int canDo;
int sDirection=0;
int pos = lowAngle, lastPos;

//Brushless
//#define pinBrushless 12

//MQ7
//#define readPeriodMQ7 90000 //ad more features to mq7
//#define heatingTime 2 //2*baseTime
// #define readingTime 3 //3*readingTime
// #define baseTime 15000
// #define togglePIN 4
// #define readPIN A3

// //FLAMES
// #define readPeriodFLAMES 500


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
RingBuf *bufLIDAR = RingBuf_new(sizeof(short), 1000);
//RingBuf *bufDHT = RingBuf_new(sizeof(byte), 8);
//RingBuf *bufFLAMES = RingBuf_new(sizeof(byte), 8);
//RingBuf *bufMQ7 = RingBuf_new(sizeof(byte), 8);

//time variables for photodiode interrupt
//long lastTime, currentTime;
		/*
		//counter variables for interrupts
		int countInt=0; //Counter for timer 3 interrupt
		int timeCount=0;

		//flags to read and cal FLAME sensors
		byte flagFLAMESdone =0;
		byte flagCAlCflames=0;
		byte flagFLAMES1=0;
		byte flagFLAMES2=0;
		byte flagFLAMES3=0;

		byte s1, s2, s3;

		byte flagSend=1;

		//flag to run MQ7 reading
		byte flagMQ7=0;
		int tempMQ7;

		//temporary vartiable for exchange data between variables
		short temp;
		byte temp2, sendBYTE, flameBYTE;

		int distCount=0;
		short dist;

		int i=0; // temporary val
		*/

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

//Servo
PWMServo servo;

//Brushless
//PWMServo brushless;
//int velocity=10;


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

void getSensors();

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
    ///////////////////////////////////////////
  #endif
	pinMode(pinSAVElidar, INPUT);
	pinMode(pinSTARTlidar, INPUT);
	//pinMode(pinPRINTlidar, INPUT);

	//Serial initialization////////////////////////
	Serial.begin(115200 );

	//Lidar stuff

	  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz

	  myLidarLite.write(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
	  myLidarLite.write(0x04, 0b00000100); // Use non-default reference acquisition count
	  myLidarLite.write(0x12, 0x03); // Reference acquisition count of 3 (default is 5)


	  //Initialize servo and sendo to pos 60
  servo.attach(pinServo);
  servo.write(100);



 	//Timer stuff
/*cli();//stop interrupts
	   //set timer2 interrupt at 8kHz
  TCCR1A = 0;// set entire TCCR2A register to 0
  TCCR1B = 0;// same for TCCR2B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR1A = 170;// = (16*10^6) / (405*8) - 1 (must be <256) 4938  4954
  // turn on CTC mode
  TCCR1A |= (1 << WGM11);
  // Set CS21 bit for 8 prescaler
  TCCR1B |= (1 << CS12);//|(1<<CS10);   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

	
sei();//allow interrupts
*/

	 /* Timer1.initialize(2778);
	  Timer1.attachInterrupt(getSensors);
  //interrupt every 1ms

	  Timer1.start();*/
	}
/////////////////////////////////////end setup

void getSensors(){//timer1 interrupt 8kHz toggles pin 9
	//TCNT1=0;
	#ifdef DEBUG
	digitalWrite(13,HIGH);
	#endif
	flagLIDAR=1;
	#ifdef DEBUG
	analogRead(A0);
	digitalWrite(13,LOW);
	#endif
}


union sendShort{    //definition of data typre to be able to separate data bytes
	short send1;
	byte send2[2];
}sendSHORT;

void loop(void){////////////////////////////////////////////////////////////////////////////
	//Record num of points to print
	saveLIDAR=digitalRead(pinSAVElidar);
	if(saveLIDAR==1 && printLIDARdata==0){
		numPrintLidar=numPointsLIDAR;
		printLIDARdata=1;
		printNum=numPrintLidar;
		Serial.write(printNum);
	}

	//Print section
	//printLIDARdata=digitalRead(pinPRINTlidar);
	if( printLIDARdata==1){
		if(numPrintLidar > 100){
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

				Serial.write(sendSHORT.send2[1]);  
				Serial.write(sendSHORT.send2[0]);
			}
		}
	}

	//Measurement section
	startLIDAR=digitalRead(pinSTARTlidar);
	if(startLIDAR==1){
		//if(flagLIDAR=1){
			digitalWrite(L1,HIGH);
				flagLIDAR=0;//is controllled by an interrupt
				Serial.println(distanceFast(false));
				bufLIDAR->add(bufLIDAR,&distance);
				numPointsLIDAR++;

				digitalWrite(L1,LOW);
			}
		//}
	// Take a measurement with receiver bias correction and print to serial terminal
  /*Serial.println(distanceFast(true));

  // Take 99 measurements without receiver bias correction and print to serial terminal
  for(int i = 0; i < 99; i++)
  {
    Serial.println(distanceFast(false));
  }*/

	
	}



