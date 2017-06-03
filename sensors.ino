#include <Arduino.h>
#include <RingBuf.h>
#include <TimerOne.h>
#include <Wire.h>
#include <PWMServo.h> 
#include <LIDARLite.h>


//Things to change
#define DEBUG 1
//to exit debug mode delete previous line ( #define DEBUG 1 )

#define UART_BAUDRATE 2000000
#define INT_PERIOD 1000
#define MAX_TIME_COUNT 90000 //90 segundos

#define pinSTARTlidar 7
#define pinSAVElidar 8
#define pinDirection 9
#define pinCompleteLIDAR 10

////////////////

#define idLIDAR 0x01

//Servo
#define pinServo 9
#define lowAngle 60
#define upAngle 140

int canDo;
int sDirection=0;
int pos = lowAngle;


//BUFFERS
RingBuf *bufLIDAR = RingBuf_new(sizeof(byte), 700);

//INITS
//LIDAR
LIDARLite myLidarLite;

byte startLIDAR=0;
byte printLIDARdata=0;
byte saveLIDAR=0;

//int flagLIDAR=0;

byte distance=0;
int numPointsLIDAR, numPrintLidar;
int printNum=0;


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
	pinMode(pinSAVElidar, INPUT);
	pinMode(pinSTARTlidar, INPUT);
	pinMode(pinDirection, OUTPUT);

	pinMode(11,OUTPUT);
	pinMode(12,OUTPUT);
	pinMode(13,OUTPUT);
	pinMode(pinCompleteLIDAR,OUTPUT);

	//Serial initialization////////////////////////
	Serial.begin(UART_BAUDRATE);

	//Lidar stuff
 	myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz

	myLidarLite.write(0x02, 0x0d); // Maximum acquisition count of 0x0d. (default is 0x80)
	myLidarLite.write(0x04, 0b00000100); // Use non-default reference acquisition count
  	myLidarLite.write(0x12, 0x03); // Reference acquisition count of 3 (default is 5)
}

byte sendBYTE;

void loop(void){
	//Record num of points to print
	//saveLIDAR=digitalRead(pinSAVElidar);
	if(digitalRead(pinSAVElidar)==HIGH){
		saveLIDAR=1;
		digitalWrite(13,HIGH);
	}else{
		//saveLIDAR=0;
		digitalWrite(13,LOW);
	}

	

  	if(sDirection==1){
  		digitalWrite(pinDirection,HIGH);
  		pos=lowAngle;
  		sDirection=1;
  	}else if(sDirection==0){
  		digitalWrite(pinDirection,LOW);
  	}

	if(saveLIDAR==1){
		saveLIDAR=0;
		//Servo angle set

    	digitalWrite(pinCompleteLIDAR,LOW);
        if(pos>=upAngle){
        	//make lidar stop and go back to 60degrees
        	digitalWrite(pinCompleteLIDAR,HIGH);
        	pos=lowAngle;
        	sDirection=1;
	    }
	    if(pos<=lowAngle){
	    	sDirection=1;
	  	}
      	
    	//Changes servo angle if time canDo flag is set
        if(sDirection==1){
          	pos++;
        }
        /*if(sDirection==0){
          	pos--;
        }*/
      
		numPrintLidar=numPointsLIDAR;
		numPointsLIDAR=0; //reset ao numero de pontos do lidar
		printNum=numPrintLidar; //variavel de controlo para impressao
		printLIDARdata=1;//activa a impressao de pontos para a serie
		//Serial.write(0x03);
		//Serial.write(printNum);
	}

	//Print section
	//printLIDARdata=digitalRead(pinPRINTlidar);
	if(printLIDARdata==1){
		
		if(numPrintLidar > 200){
		digitalWrite(12,HIGH);
			//digitalWrite(13,HIGH);
			if(Serial.availableForWrite() > 13){
				if(printNum==numPrintLidar){
					//Serial.write(idLIDAR);
					//Serial.write(pos);
					//Serial.println(idLIDAR);
					//Serial.println(pos);
				}

				if(printNum==0){
					printLIDARdata=0;
				}else if (printNum >=10){
					printNum=printNum-10;

					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);

					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);

					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);

					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);

					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);

					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);

					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);

					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);

					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);

					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);
				}else{
					printNum=printNum-1;
					bufLIDAR->pull(bufLIDAR, &sendBYTE);
					//Serial.write(sendBYTE);
					Serial.println(sendBYTE);
				}


				
			}
		}

		digitalWrite(12,LOW);
	}

	if(digitalRead(pinSTARTlidar)==HIGH){
		startLIDAR=1;
		pos=lowAngle;
		digitalWrite(11,HIGH);
	}else{
		startLIDAR=0;		
		digitalWrite(11,LOW);
	}

	//Measurement section
	//startLIDAR=digitalRead(pinSTARTlidar);
	if(startLIDAR==1){
		if(bufLIDAR->numElements(bufLIDAR)<690){
			if(numPointsLIDAR%100==0){
				distance = (distanceFast(true));
			}else{
				distance = (distanceFast(false));
			}
			if(distance>=10){
				if(distance>300){
					distance=255;
				}else if(distance < 10){
					distance=0;
				}else{
					//distance=map(distance,10,500,0,254);
				}
				bufLIDAR->add(bufLIDAR,&distance);
				numPointsLIDAR++;
			}
		}
	}
	delay(2);
}



