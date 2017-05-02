#include <RingBuf.h>

#include <TimerOne.h>
#include "hartimu.h"
#include <Wire.h>

#include <DHT.h>

#include <Servo.h> 
#include "brsh.h"

#define DEBUG 1


//Debug LEDs
#define L1 30 //DHT
#define L2 31 //MPU
#define L3 32 //Brushless
#define L4 33 //MQ7
#define L5 34 //FIRE
#define L6 35 //LIDAR
#define L7 36 ////ERROR/////////////////////////
#define L8 37 ////OK////////////////////////////



//DHT
#define DHTPIN 8
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE); //dht object declaration
float h = NAN,t = NAN;
//DHT connections
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor


//PhotoDiode
#define pinPhotoDiode 2
//Servo
#define pinServo 4
//Brushless
#define pinBrushless 6

#define lowAngle 60
#define upAngle 140




Servo servo;

//initBrush(pinBrushless);
Servo brushless;
int velocity=10;

const int led = 13;  // the pin with a LED

int canDo;
int flag=0;
int pos = lowAngle;


RingBuf *bufMPU = RingBuf_new(sizeof(short), 21);
RingBuf *bufDHT = RingBuf_new(sizeof(byte), 8);

//time variables for photodiode interrupt
long lastTime, currentTime;

//counter variables for interrupts
int countMPU=0; //Counter for MPU
volatile int countTemp=0; //Counter for DHT11
//temporary vartiable for exchange data between variables
short temp;
byte temp2, sendBYTE;



void setup(void){
  if(DEBUG){
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
  }

 //Serial initialization////////////////////////
  Serial.begin(115200);
  if (DEBUG){
   Serial.println("Hello im working");
 }
 
 //////////////////////////////////////////////

 //DHT/////////////////////////////////////////
 dht.begin();

 /////////////////////////////////////////////

 //MPU6050 initialization//////////////////////
 initialize_imu();
 /////////////////////////////////////////////

 //Initialize servo and sendo to pos 60
 servo.attach(pinServo);
 servo.write(pos);
 ////////////////////////////////////////////////

 //initialize brushless/////////////////////////
 //set speed to 10
 //brsh brush(pinBrushless);

 brushless.attach(pinBrushless,1000,2000);
 //brush.turnOn();
 turnOn(brushless);

 //brush.defineVelocity(10);
 defineVelocity(velocity,brushless);
 ///////////////////////////////////////////////

//Timer initialization///////////////////////////
//Timer is used for interrupt
 Timer1.initialize(1000);
 Timer1.attachInterrupt(getSensors);
  //interrupt every 1ms
 Timer1.start();

 ////////////////////////////////////////////////


//Interrupt from photodiode
 attachInterrupt(digitalPinToInterrupt(pinPhotoDiode),changeAngle,RISING);
 ///////////////////////////////////////////////







}/////////////////////////////////////end setup/
////////////////////////////////////////////////



void changeAngle(){  
  digitalWrite(L4,HIGH);
  lastTime=currentTime;
  currentTime=millis();
  /*
  Serial.println(currentTime-lastTime);
  Serial.println(velocity);
//*/
  if(currentTime-lastTime > 600 && currentTime-lastTime <700){
    canDo=1;
  }else if( currentTime-lastTime < 600){
    velocity--;
    defineVelocity(velocity,brushless);
    canDo=0;
  }else if( currentTime-lastTime > 700){
    velocity++;
    defineVelocity(velocity,brushless);
    canDo=0;
  }


  if(pos>=upAngle){
    flag=0;
  }
  if(pos<=lowAngle){
    flag=1;
  }

  if(flag==1  && canDo==1){
    canDo = 0;
    pos++;
  }
  if(flag==0  && canDo==1){
    canDo = 0;
    pos--;
  }
  servo.write(pos);
  digitalWrite(L4,LOW);
}




void getSensors(void){ //ISR function, gets data from MPU@250HZ, LIDAR and  sets counter for DHT11 to run
//increment time variables
  digitalWrite(L2,HIGH);


  countMPU+=1; 
  countTemp+=1;
  /*if(countTemp%60==0){
    canDo=1;
  }*/

//MPU6050 data aquisition  
  if(countMPU==4){
    countMPU=0;

    read_mpu_values();
    compute_data();

    temp= (short) get_yaw()*10;
    bufMPU->add(bufMPU, &temp);
    temp= (short) get_roll()*10;
    bufMPU->add(bufMPU, &temp);
    temp= (short) get_pitch()*10;
    bufMPU->add(bufMPU, &temp);  
  }
//end MPU6050 data aquisition



  digitalWrite(L2,LOW);
}


union sendShort{    //definition of data typre to be able to separate data bytes
  short send1;
  byte send2[2];
}sendSHORT;



int run=1;
bool errorBrush = true;



void loop(void){
  if(run==1){
    ////////////////////////////////////////////////
////////////////////////////////////////////////
//////////////////TESTS/////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////



//MPU//////////////////////////////////////////////

///////////////////////////////////////////////////

 ////Brushless test////////////////////////////////
/*
    lastTime=millis();
    currentTime= lastTime;
    errorBrush = true;
   // digitalWrite(L3,HIGH);
    canDo=0;
    do{
     while(currentTime-lastTime <= 4000){
      if(currentTime-lastTime < 700 && canDo == 1){
        errorBrush=false;
        break;
      }
    }
    if(errorBrush==true){
      digitalWrite(L7,HIGH);
      Serial.print("Error setting brushless speed");
      //detach(brushless);
      //revive(pinBrushless ,brushless);
      brushless.attach(pinBrushless,1000,2000);
      turnOn(brushless);
      defineVelocity(10,brushless);

    }
  }while(errorBrush==true);

  digitalWrite(L7,LOW);
  digitalWrite(L3,LOW);
  //*/


//DHT11 test //////////////////////////////////
 //if (DEBUG){
    digitalWrite(L1,HIGH);
    do{

      delay(2000);
//get readings
      h =dht.readHumidity();
      t=dht.readTemperature();


//verify if readings are valid
      if (isnan(h) || isnan(t)) {
        digitalWrite(L7,HIGH);
        Serial.println("Failed to read from DHT sensor!");

      }
    }while(isnan(h) || isnan(t));

    Serial.println(t);
    Serial.println(h);

    digitalWrite(L1,LOW);
//}
///////////////////////////////////////////////////*/

    run=0;
  }






//servo.write(pos);

//  Serial.println(get_roll());
  if(bufMPU->numElements(bufMPU) >3){
    Serial.print(3);
    Serial.print("            ");


    bufMPU->pull(bufMPU, &sendSHORT);
    Serial.print(sendSHORT.send1);

    Serial.print("            ");

    bufMPU->pull(bufMPU, &sendSHORT);
    Serial.print(sendSHORT.send1);

    Serial.print("            ");

    bufMPU->pull(bufMPU, &sendSHORT);
    Serial.println(sendSHORT.send1 );


//    Serial.write(sendSHORT.send2[0]);
//    Serial.write(sendSHORT.send2[1]);

  }

  if(countTemp >= 2000) {
    countTemp=0;
//
    digitalWrite(L1,HIGH);
    temp2 =(byte) dht.readHumidity();
    bufDHT->add(bufDHT,&temp2);

    temp2 =(byte) dht.readTemperature();
    bufDHT->add(bufDHT,&temp2);
    digitalWrite(L1,LOW);

  }

  if(bufDHT->numElements(bufDHT) >2){
    Serial.println(2);
    bufDHT->pull(bufDHT, &sendBYTE);    
//Serial.write(sendBYTE);

    Serial.println(sendBYTE);
    bufDHT->pull(bufDHT, &sendBYTE);    
//Serial.write(sendBYTE);

    Serial.println(sendBYTE);


  }
}//*/




//}



