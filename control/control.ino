#include <RingBuf.h>
#include <TimerThree.h>
#include <TimerOne.h>
#include "hartimu.h"
#include <Wire.h>
#include <DHT.h>
#include <PWMServo.h> 
#include "brsh.h"
#include "flamesensor.h"
//#include <LIDARLite.h>

//Things to change
#define DEBUG 1
//to exit debug mode delete previous line ( #define DEBUG 1 )

#define UART_BAUDRATE 2000000
#define INT_PERIOD 2000
#define MAX_TIME_COUNT 90000 //90 segundos
#define WAIT_TIME 600

////////////////

#define idLIDAR 0x01
#define idDHT 0x02
#define idMPU 0x03
#define idMQ7 0x05
#define idFLAMES 0x04

//Pin LIDAR
#define pinStartLIDAR 7
#define pinSaveLIDAR 8
#define pinDirection 9

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
#define readPeriodMQ7 90000 //ad more features to mq7
#define heatingTime 2 //2*baseTime
#define readingTime 3 //3*readingTime
#define baseTime 15000
#define togglePIN 4
#define readPIN A3

//FLAMES
#define readPeriodFLAMES 500

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
RingBuf *bufFLAMES = RingBuf_new(sizeof(byte), 8);
RingBuf *bufMQ7 = RingBuf_new(sizeof(byte), 8);

//time variables for photodiode interrupt
long lastTime, currentTime;

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

int val=0;

//temporary vartiable for exchange data between variables
short temp;
byte temp2, sendBYTE, flameBYTE;

int distCount=0;
short dist;

int i=0; // temporary val

//INITS
//LIDAR
//LIDARLite myLidarLite;
int numLIDAR=0;
int numPointsLIDAR=0;
byte flagLIDARcomplete=0;

byte sendMPUtoRobot=0;


int oldTime=0;
byte flagSave=0, printLIDAR, printNum;
int numPrintLidar;

//Servo
PWMServo servo;

//Brushless
PWMServo brushless;
int velocity=20;

// //DHT
// float h = NAN,t = NAN;
// DHT dht(DHTPIN, DHTTYPE); //dht object declaration

void getSensors(void);
void changeAngle(void);

void runA(void){ //function used to execute the 1st case of the interrupt (execution must be less than 1ms)

  read_mpu_values();
}

void runB(void){ //function used to execute the 2nd case of the interrupt (execution must be less than 1ms)

  compute_data();
  temp= (short)((int) get_yaw());
  bufMPU->add(bufMPU, &temp);
  temp= (short)((int) get_roll());
  bufMPU->add(bufMPU, &temp);
  temp= (short)((int) get_pitch());
  bufMPU->add(bufMPU, &temp); 
}

void runC(void){
  noInterrupts();
  if(flagFLAMES1==1){
   /* #ifdef DEBUG
      digitalWrite(L1,HIGH);
      #endif*/

    flagFLAMES1=0;
    s1 = getFS1values();

  }else if(flagFLAMES2==1){
    /*#ifdef DEBUG
      digitalWrite(L1,HIGH);
      #endif*/
    flagFLAMES2=0;
    s2 = getFS2values();

  }else if(flagFLAMES3==1){
    /*#ifdef DEBUG
    digitalWrite(L1,HIGH);
    #endif*/

    flagFLAMES3=0;
    s3 = getFS3values();


    flameBYTE=(byte) flameposition(s1,s2,s3);
    bufFLAMES->add(bufFLAMES,&flameBYTE);
    

  }else if( flagMQ7==1){

    flagMQ7=0;
    tempMQ7= (byte) analogRead(readPIN)*0.48876; // analogRead(readPIN)*5.0/1024*10;
    bufMQ7->add(bufMQ7,&tempMQ7);

  }
  interrupts();
}

void setup(void){
  //Define debug LEDpins as output///////////
   #ifdef DEBUG

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

  pinMode(pinPhotoDiode, INPUT);
  pinMode(13, OUTPUT);


  pinMode(pinStartLIDAR,OUTPUT);
  pinMode(pinSaveLIDAR,OUTPUT);
    //pinMode(pinPrintLIDAR,OUTPUT);


  //Serial initialization////////////////////////
  Serial.begin(UART_BAUDRATE);
  Serial1.begin(115200); //Serial used to send to robot group
  

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

 //Interrupt from photodiode
  //attachInterrupt(digitalPinToInterrupt(pinPhotoDiode),changeAngle,RISING);////////////////////////////////////////////////////////////////////////////////

  //Timer initialization///////////////////////////
  //Timer is used for interrupt

  Timer3.initialize(INT_PERIOD);
  Timer3.attachInterrupt(getSensors);
  //interrupt every 1ms
  Timer3.start();
}

void getSensors(void){ //ISR function, gets data from MPU@250HZ, LIDAR and  sets counter for DHT11 to run
  //increment time variables
  #ifdef DEBUG
  digitalWrite(L2,HIGH);
  #endif

  digitalWrite(pinSaveLIDAR,LOW);
  flagSave=0;

  currentTime++;
  
  if(val==HIGH){
    val=LOW;
    if(currentTime >WAIT_TIME){
      #ifdef DEBUG
        digitalWrite(L5,HIGH);
      #endif

      digitalWrite(pinSaveLIDAR,HIGH);
      if(digitalRead(pinDirection) == HIGH){
        sDirection=1;
      }else{
        sDirection=0;
      }

      flagSave=1;

      //Brushless motor speed feedback
      numLIDAR=numPointsLIDAR;
      numPointsLIDAR=0; 
      canDo=1; // Overide motor speed correction

      /*
        if(currentTime > 630 && currentTime <680){
          canDo=1;
        }else if( currentTime<= 630){
          if (velocity == 0){
            velocity=0;
          }else{
            velocity--;
          }
          canDo=0;
        }else if( currentTime>= 680){
          velocity++;
          canDo=0;
        }

        defineVelocity(velocity,brushless);
      */
      //Servo angle set
      /*
        if(pos>=upAngle){
         sDirection=0;
        }
        if(pos<=lowAngle){
          sDirection=1;
        }
      */

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
      currentTime=0;
    }
  }

  if(timeCount% readPeriodFLAMES == 0){
    sendMPUtoRobot=1;
    flagFLAMES1 =1;
    flagFLAMES2 =1;
    flagFLAMES3 =1;
    flagCAlCflames=1;
  }

  if( timeCount%(readPeriodMQ7)==0){
    flagMQ7=1;
  }

  if(timeCount < MAX_TIME_COUNT){
    timeCount++;
  }else{
    timeCount=0;
  }

  switch(countInt){
    case 0:

    //runA();
    //flagSend=0;
    // break;
    //case 1:
    //runB();

    //break;
    case 1:


    runC();
    flagSend=1;

    break;
    default:

    break;
  }

  if(countInt <1){
    countInt++;
  }else{
    countInt=0;
  }

  //countInt=(countInt<3)?(countInt++):(0);//verifica se countInt <3 se for incrementa se não passa para 0;
  //RUNNING LIST
  //1 - MPU   - getReadings//lidar
  //2 - MPU   - calculations//lidar
  //3 - analogSensor

  #ifdef DEBUG
  digitalWrite(L2,LOW);
  #endif

  #ifdef DEBUG
  digitalWrite(L5,LOW);
  #endif
}


union sendShort{    //definition of data typre to be able to separate data bytes
  short send1;
  byte send2[2];
}sendSHORT, sendShortRobot;

void loop(void){
  ////////////////////////////////////////////////////////////////////////////
  digitalWrite(pinStartLIDAR,HIGH);

  val=digitalRead(pinPhotoDiode);//photodiode val
  digitalWrite(13, val);

  if(sendMPUtoRobot==1){ //send MPU z data to robot team
    Serial1.write(sendShortRobot.send1); //only send the x value as a byte from 0 to 255
  }

}

