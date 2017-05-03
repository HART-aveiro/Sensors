#include <RingBuf.h>

//#include <MsTimer2.h>
#include <TimerOne.h>

#include "hartimu.h"
#include <Wire.h>

#include <DHT.h>

#include <Servo.h> 
#include "brsh.h"

#include "flamesensor.h"


#include <LIDARLite.h>

#define DEBUG 1


//LIDARLite myLidarLite;


//Debug LEDs
#define L1 30 //DHT
#define L2 31 //MPU
#define L3 32 //Brushless
#define L4 33 //MQ7
#define L5 34 //FIRE
#define L6 35 //LIDAR
#define L7 36 ////ERROR/////////////////////////
#define L8 37 ////OK////////////////////////////

#define heatingTime 2 //2*baseTime
#define readingTime 3 //3*readingTime
#define baseTime 15000
#define togglePIN 4
#define readPIN A3
unsigned int counterMQ7;
char mark;
char isReading;





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
#define pinServo 3
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
RingBuf *bufLIDAR = RingBuf_new(sizeof(short), 1000);
RingBuf *bufDHT = RingBuf_new(sizeof(byte), 8);
RingBuf *bufFlame = RingBuf_new(sizeof(byte), 8);

//time variables for photodiode interrupt
long lastTime, currentTime;

//counter variables for interrupts
int countMPU=-1; //Counter for MPU
volatile int countTemp=0; //Counter for DHT11
//temporary vartiable for exchange data between variables
short temp;
byte temp2, sendBYTE, flameBYTE;



int distCount=0;
short dist;

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
//*/


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
 //MQ7//////////////////////////////////
 pinMode(togglePIN,OUTPUT);
 digitalWrite(togglePIN,HIGH);
  ///////////////////////////////////




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
 Timer1.initialize(2000);
 Timer1.attachInterrupt(getSensors);
  //interrupt every 1ms
 Timer1.start();//*/
/*
 MsTimer2::set(1, getSensors); // 500ms period
  MsTimer2::start();
//*/
 ////////////////////////////////////////////////


//Interrupt from photodiode
 attachInterrupt(digitalPinToInterrupt(pinPhotoDiode),changeAngle,RISING);
 ///////////////////////////////////////////////

/*
/*
    begin(int configuration, bool fasti2c, char lidarliteAddress)

    Starts the sensor and I2C.

    Parameters
    ----------------------------------------------------------------------------
    configuration: Default 0. Selects one of several preset configurations.
    fasti2c: Default 100 kHz. I2C base frequency.
      If true I2C frequency is set to 400kHz.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  //*/
 // myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz

  /*
    configure(int configuration, char lidarliteAddress)

    Selects one of several preset configurations.

    Parameters
    ----------------------------------------------------------------------------
    configuration:  Default 0.
      0: Default mode, balanced performance.
      1: Short range, high speed. Uses 0x1d maximum acquisition count.
      2: Default range, higher speed short range. Turns on quick termination
          detection for faster measurements at short range (with decreased
          accuracy)
      3: Maximum range. Uses 0xff maximum acquisition count.
      4: High sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for high sensitivity and noise.
      5: Low sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for low sensitivity and noise.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
 // myLidarLite.configure(0); // Change this number to try out alternate configurations


//*/



}/////////////////////////////////////end setup/
////////////////////////////////////////////////



void changeAngle(){  
  digitalWrite(L4,HIGH);
  lastTime=currentTime;
  currentTime=millis();
  if(currentTime-lastTime >100){
/*
    Serial.println(currentTime-lastTime);

    Serial.println(velocity);
//*/
    if(currentTime-lastTime > 630 && currentTime-lastTime <680){
      canDo=1;
    }else if( currentTime-lastTime <= 630){
      if (velocity == 0){
        velocity=0;
      }else{
        velocity--;
      }
    //defineVelocity(velocity,brushless);
      canDo=0;
    }else if( currentTime-lastTime >= 680){
      velocity++;

      canDo=0;
    }
    defineVelocity(velocity,brushless);


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
  }
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
  /*if (countMPU==3){

    read_mpu_values();
  }//*/
  if(countMPU==1){
    if(distCount==100){
      //dist = (short) myLidarLite.distance(false);
      dist = (short) distanceFast(false);
    }else{
      //dist = (short) myLidarLite.distance();
      dist = (short) distanceFast(false);
    }
    
    bufLIDAR->add(bufLIDAR, &dist);
  }//*/

  if(countMPU==2){
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





  digitalWrite(L2,LOW);
}


union sendShort{    //definition of data typre to be able to separate data bytes
  short send1;
  byte send2[2];
}sendSHORT;



int run=1, s1,s2, s3;
float data =0;
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
 /* if(bufMPU->numElements(bufMPU) >3){
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

  }//*/

  if(countTemp >= 1000) {
    countTemp=0;
//
    digitalWrite(L1,HIGH);
    temp2 =(byte) dht.readHumidity();
    bufDHT->add(bufDHT,&temp2);

    temp2 =(byte) dht.readTemperature();
    bufDHT->add(bufDHT,&temp2);
    digitalWrite(L1,LOW);

  }
/*
  if(bufDHT->numElements(bufDHT) >2){
    Serial.println(2);
    bufDHT->pull(bufDHT, &sendBYTE);    
//Serial.write(sendBYTE);

    Serial.println(sendBYTE);
    bufDHT->pull(bufDHT, &sendBYTE);    
//Serial.write(sendBYTE);

    Serial.println(sendBYTE);


  }//*/
  if(countTemp%100){

    data= /*(float)*/ analogRead(readPIN)*5.0/1023;
   // Serial.println(data);

  }
/*
  if(countTemp % 500 == 0) {
    s1 = getFS1values();
    s2 = getFS2values();
    s3 = getFS3values();
    flameBYTE=(byte) flameposition(s1,s2,s3);
    Serial.println(flameBYTE+'a');
    bufFlame->add(bufFlame,&flameBYTE);
  }*/



  if(bufLIDAR->numElements(bufLIDAR) >181){
    Serial.print(3);
    
    Serial.print("            ");


    bufMPU->pull(bufLIDAR, &sendSHORT);
    Serial.print(sendSHORT.send1);

    Serial.print("            ");

    bufMPU->pull(bufLIDAR, &sendSHORT);
    Serial.print(sendSHORT.send1);

    Serial.print("            ");

    bufMPU->pull(bufLIDAR, &sendSHORT);
    Serial.println(sendSHORT.send1 );


//    Serial.write(sendSHORT.send2[0]);
//    Serial.write(sendSHORT.send2[1]);

  }//*/








}//*/




//}



