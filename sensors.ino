#include <RingBuf.h>
#include <TimerThree.h>
#include <TimerOne.h>
#include "hartimu.h"
#include <Wire.h>
#include <DHT.h>
#include <PWMServo.h> 
#include "brsh.h"
#include "flamesensor.h"
#include <LIDARLite.h>

//Things to change
#define DEBUG 1
//to exit debug mode delete previous line ( #define DEBUG 1 )

#define UART_BAUDRATE 2000000
#define INT_PERIOD 1000

////////////////

#define idLIDAR 0x01
#define idDHT 0x02
#define idMPU 0x03
#define idMQ7 0x05
#define idFLAMES 0x04

//PhotoDiode
#define pinPhotoDiode 3

//Servo
#define pinServo 11
#define lowAngle 60
#define upAngle 140

int canDo;
int sDirection=0;
int pos = lowAngle, lastPos;

//Brushless
#define pinBrushless 12

//MQ7
#define heatingTime 2 //2*baseTime
#define readingTime 3 //3*readingTime
#define baseTime 15000
#define togglePIN 4
#define readPIN A3

//DHT
#define DHTPIN 5  
#define DHTTYPE DHT11
//DHT connections
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

#ifdef DEBUG
//Debug LEDs
#define L1 30 //DHT
#define L2 31 //MPU
#define L3 32 //Brushless
#define L4 33 //MQ7
#define L5 34 //FIRE
#define L6 35 //LIDAR
#define L7 36 ////ERROR/////////////////////////
#define L8 37 ////OK////////////////////////////
#endif

//BUFFERS
RingBuf *bufMPU = RingBuf_new(sizeof(short), 21);
RingBuf *bufLIDAR = RingBuf_new(sizeof(short), 1000);
//RingBuf *bufDHT = RingBuf_new(sizeof(byte), 8);
RingBuf *bufFLAMES = RingBuf_new(sizeof(byte), 8);
RingBuf *bufMQ7 = RingBuf_new(sizeof(byte), 8);

//time variables for photodiode interrupt
long lastTime, currentTime;

//counter variables for interrupts
int countInt=0; //Counter for timer 3 interrupt


//Variables (to be decided which stay, which go) 
unsigned int counterMQ7;
char mark;
char isReading;
char flagFlames, flagDHT;

volatile int countTemp=0; //Counter for DHT11
//temporary vartiable for exchange data between variables
short temp;
byte temp2, sendBYTE, flameBYTE;

int distCount=0;
short dist;

//LIDAR
LIDARLite myLidarLite;

//Servo
PWMServo servo;

//Brushless
PWMServo brushless;
int velocity=10;

//DHT
float h = NAN,t = NAN;
DHT dht(DHTPIN, DHTTYPE); //dht object declaration



// Read distance. The approach is to poll the status register until the device goes
// idle after finishing a measurement, send a new measurement command, then read the
// previous distance data while it is performing the new command.
int distanceFast(bool biasCorrection){
  interrupts();
  byte isBusy = 1;
  int distance;
  int loopCount;

  // Poll busy bit in status register until device is idle
  while(isBusy)
  {
    // Read status register
    Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.requestFrom(LIDARLITE_ADDR_DEFAULT, 1);
    isBusy = Wire.read();
    isBusy = bitRead(isBusy,0); // Take LSB of status register, busy bit

    loopCount++; // Increment loop counter
    // Stop status register polling if stuck in loop
    if(loopCount > 9999)
    {
      break;
    }
  }

  // Send measurement command
  Wire.beginTransmission(LIDARLITE_ADDR_DEFAULT);
  Wire.write(0X00); // Prepare write to register 0x00
  if(biasCorrection == true)
  {
    Wire.write(0X04); // Perform measurement with receiver bias correction
  }
  else
  {
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
    ///////////////////////////////////////////
  #endif

 //Serial initialization////////////////////////
  Serial.begin(UART_BAUDRATE);

 //MPU6050 initialization//////////////////////
  initialize_imu();

 //MQ7//////////////////////////////////
  pinMode(togglePIN,OUTPUT);
  digitalWrite(togglePIN,HIGH);

 //Initialize servo and sendo to pos 60
  servo.attach(pinServo);
  servo.write(pos);

 //initialize brushless/////////////////////////
 //set speed to 10

  brushless.attach(pinBrushless,1000,2000);
  turnOn(brushless);
  defineVelocity(velocity,brushless);

 //Timer initialization///////////////////////////
 //Timer is used for interrupt

  Timer3.initialize(INT_PERIOD);
  Timer3.attachInterrupt(getSensors);
  //interrupt every 1ms

  Timer3.start();

 //Interrupt from photodiode
  attachInterrupt(digitalPinToInterrupt(pinPhotoDiode),changeAngle,RISING);
 }
/////////////////////////////////////end setup

void changeAngle(){  
  digitalWrite(L4,HIGH);
  lastTime=currentTime;
  currentTime=millis();

  if(currentTime-lastTime >100){
  //Brushless motor speed feedback

    if(currentTime-lastTime > 630 && currentTime-lastTime <680){
      canDo=1;

    }else if( currentTime-lastTime <= 630){
      (velocity > 0)?(velocity--):(0);
      // if (velocity == 0){
      //   velocity=0;
      // }else{
      //   velocity--;
      // }
      canDo=0;
    }else if( currentTime-lastTime >= 680){
      velocity++;
      canDo=0;
    }

    defineVelocity(velocity,brushless);

    //Servo angle set
    if(pos>=upAngle){
      sDirection=0;
    }
    if(pos<=lowAngle){
      sDirection=1;
    }

    //Changes servo angle if time canDo flag is set
    if(canDo==1){
      canDo = 0;
      lastPos=pos;
      if(sDirection==1){
        pos++;
      }
      if(sDirection==0){
        pos--;
      }
    }

    //Only writes position to servo if the difference between positions is one
    //Prevents false skips....
    if(lastPos-pos==1 || pos-lastPos==1){
      servo.write(pos);
    }

  }
  digitalWrite(L4,LOW);
}

void getSensors(void){ //ISR function, gets data from MPU@250HZ, LIDAR and  sets counter for DHT11 to run
//increment time variables
  #ifdef DEBUG
  digitalWrite(L2,HIGH);
  #endif


  countInt++;
  //RUNNING LIST
  //1 - MPU   - getReadings
  //2 - MPU   - calculations
  //3 - LIDAR -






  
 //MPU6050 data aquisition  
  // if(countMPU==1){
  //   if(distCount==100){
  //     //dist = (short) myLidarLite.distance(false);
  //     dist = (short) distanceFast(false);
  //   }else{
  //     //dist = (short) myLidarLite.distance();
  //     dist = (short) distanceFast(false);
  //   }


  //   bufLIDAR->add(bufLIDAR, &dist);
  // }
  /*if (countMPU==1)
  {
    compute_data();
    temp= (short) get_yaw()*10;
    bufMPU->add(bufMPU, &temp);
    temp= (short) get_roll()*10;
    bufMPU->add(bufMPU, &temp);
    temp= (short) get_pitch()*10;
    bufMPU->add(bufMPU, &temp);  
  }*/


 /* if(countMPU==2){
    countMPU=0;


    read_mpu_values();
    //compute_data();

    //temp= (short) get_yaw()*10;
   // bufMPU->add(bufMPU, &temp);
   // temp= (short) get_roll()*10;
   // bufMPU->add(bufMPU, &temp);
  //temp= (short) get_pitch()*10;
   // bufMPU->add(bufMPU, &temp);  
  }*/
  
//end MPU6050 data aquisition


  // MQ7 data aquisition
  counterMQ7++;
  if (counterMQ7 == baseTime){
    mark++;
    counterMQ7 = 0;
  }

  if (mark == heatingTime && isReading == 0){
    digitalWrite(togglePIN,LOW);
    isReading = 1;
    mark = 0;
  }

  if (mark == readingTime && isReading == 1){
    digitalWrite(togglePIN,HIGH);
    isReading = 0;
    mark = 0;
  }
  // end MQ7 data aquisition

  // LER valores DHT no loop
  if(countTemp >= 1000){
    flagDHT=1;
  }

  // LER valores flame no loop
  if(countTemp % 500 == 0){
    flagFlames=1;
  }

  #ifdef DEBUG
  digitalWrite(L2,LOW);
  #endif
}


union sendShort{    //definition of data typre to be able to separate data bytes
  short send1;
  byte send2[2];
 }sendSHORT;


////// inicializações para testes

int run=1, s1,s2, s3,i;
float data =0;
bool errorBrush = true;

////// fim das initializações para testes 

//LOOP

 void loop(void){
   ///////////////////////////////////////////////////////////////////////////////
  // FUNCOES NO LOOP

  // DHT funtion

   /*if(flagDHT==1) {
     flagDHT=0;
     countTemp=0;


     digitalWrite(L1,HIGH);
     temp2 =(byte) dht.readHumidity();
     bufDHT->add(bufDHT,&temp2);

     temp2 =(byte) dht.readTemperature();
     bufDHT->add(bufDHT,&temp2);
     digitalWrite(L1,LOW);
   }*/

  // MQ7 leitura


   if(isReading == 1 && mark == 2 && countTemp%1000==0){

     temp2= (byte) analogRead(readPIN)*0.48876; // analogRead(readPIN)*5.0/1024*10;
     bufMQ7->add(bufMQ7,&temp2);
   }

  ////////////////////////////////////////////////////////////////////////////////

  // ENVIAR DADOS

  //MPU
   if(bufMPU->numElements(bufMPU) >3){
    Serial.write(idMPU);

    bufMPU->pull(bufMPU, &sendSHORT);
    Serial.write(sendSHORT.send2[0]);
    Serial.write(sendSHORT.send2[1]);

    bufMPU->pull(bufMPU, &sendSHORT);
    Serial.write(sendSHORT.send2[0]);
    Serial.write(sendSHORT.send2[1]);

    bufMPU->pull(bufMPU, &sendSHORT);
    Serial.write(sendSHORT.send2[0]);
    Serial.write(sendSHORT.send2[1]);
   }

  // DHT

   /*if(bufDHT->numElements(bufDHT) >2){
    Serial.write(idDHT);

    bufDHT->pull(bufDHT, &sendBYTE);    
    Serial.write(sendBYTE);

    bufDHT->pull(bufDHT, &sendBYTE); 
    Serial.write(sendBYTE);
   }*/

  // MQ7
   if(bufMQ7->numElements(bufMQ7) >1){

    Serial.write(idMQ7);

    bufMQ7->pull(bufMQ7, &sendBYTE);   
    Serial.write(sendBYTE);
   }

  //FLAMES

   if(flagFlames==1) {
    flagFlames=0;

    s1 = getFS1values();
    s2 = getFS2values();
    s3 = getFS3values();
    flameBYTE=(byte) flameposition(s1,s2,s3);

    bufFLAMES->add(bufFLAMES,&flameBYTE);
   }


   if (bufFLAMES-> numElements(bufFLAMES) > 1){
    Serial.write(idFLAMES);

    bufFLAMES->pull(bufFLAMES, &sendBYTE);    
    Serial.write(sendBYTE);
   }

  // LIDAR

   if(bufLIDAR->numElements(bufLIDAR) >181){
      //Serial.print(3);
    Serial.write(idLIDAR);

    for(i =180; i >= 0; i--){
      bufLIDAR->pull(bufLIDAR, &sendSHORT);

      Serial.write(sendSHORT.send2[1]);  
      Serial.write(sendSHORT.send2[0]);
    }
   } 
 }



