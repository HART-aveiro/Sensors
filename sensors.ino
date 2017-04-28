#include <RingBuf.h>

#include <TimerOne.h>
#include "hartimu.h"
#include <Wire.h>

#include <idDHT11.h>

#include <Servo.h> 
#include "brsh.h"

#define pinPhotoDiode 2
#define pinServo 4
#define pinBrushless 6

#define lowAngle 60
#define upAngle 140

Servo servo;


const int led = 13;  // the pin with a LED

int flag=0, canDo=0;
int pos = lowAngle;

//DHT11 declarations
int idDHT11pin = 3; //Digital pin for comunications
int idDHT11intNumber = 1; //interrupt number (must be the one that use the previus defined pin (see table above)
void dht11_wrapper(); // must be declared before the lib initialization

// Lib instantiate
idDHT11 DHT11(idDHT11pin,idDHT11intNumber,dht11_wrapper);
//end DHT11 declarations



RingBuf *bufMPU = RingBuf_new(sizeof(short), 21);
//RingBuf *bufDHT = RingBuf_new(sizeof(byte), 8);

//short teste=-741; 

//int lastTime=0;

void setup(void){
  //noInterrupts();
  pinMode(led, OUTPUT);
  digitalWrite(led,LOW);


  servo.attach(pinServo);

  brsh brushless(pinBrushless);
  brushless.turnOn();

  //MPU6050 initialization
  initialize_imu();

  //Serial initialization
  Serial.begin(115200);
  Serial.println("Hello im working");

  //Timer initialization
  //Timer is used for interrupt
  Timer1.initialize(1000);
  Timer1.attachInterrupt(getSensors); // blinkLED to run every 0.15 seconds
  Timer1.start();

 
  brushless.defineVelocity(10);

  attachInterrupt(digitalPinToInterrupt(pinPhotoDiode),changeAngle,HIGH);

  
}



void changeAngle(){  
  if(pos>=upAngle && canDo==1){
    canDo=0;
    flag=0;
  }
  if(pos<=lowAngle &&canDo==1){
    canDo=0;
    flag=1;
  }
  
  if(flag==1){
    pos++;
  }
  if(flag==0){
    pos--;
  }
}

void dht11_wrapper() {
  DHT11.isrCallback();
}

int countMPU=0; //Counter for MPU
volatile int countTemp=0; //Counter for DHT11


//temporary vartiable for exchange data between variables
short temp;



void getSensors(void){ //ISR function, gets data from MPU@250HZ, LIDAR and  sets counter for DHT11 to run
  //increment time variables
  countMPU+=1; 
  countTemp+=1;
  if(countTemp%60==0){
    canDo=1;
  }

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



}


union sendShort{    //definition of data typre to be able to separate data bytes
  short send1;
  byte send2[2];
}sendSHORT;




void loop(void){
  servo.write(pos);
  
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
  DHT11.acquire();
  while (DHT11.acquiring())
    ;
  int result = DHT11.getStatus();
  switch (result)
  {
    case IDDHTLIB_OK: 
    //Serial.println("OK"); 
    Serial.print("2");
    Serial.print("            ");
    Serial.print(DHT11.getHumidity(), 2);
    Serial.print("            ");
    Serial.println(DHT11.getCelsius(), 2);
    break;
    case IDDHTLIB_ERROR_CHECKSUM: 
    Serial.println("Error\n\r\tChecksum error"); 
    break;
    case IDDHTLIB_ERROR_ISR_TIMEOUT: 
    Serial.println("Error\n\r\tISR Time out error"); 
    break;
    case IDDHTLIB_ERROR_RESPONSE_TIMEOUT: 
    Serial.println("Error\n\r\tResponse time out error"); 
    break;
    case IDDHTLIB_ERROR_DATA_TIMEOUT: 
    Serial.println("Error\n\r\tData time out error"); 
    break;
    case IDDHTLIB_ERROR_ACQUIRING: 
    Serial.println("Error\n\r\tAcquiring"); 
    break;
    case IDDHTLIB_ERROR_DELTA: 
    Serial.println("Error\n\r\tDelta time to small"); 
    break;
    case IDDHTLIB_ERROR_NOTSTARTED: 
    Serial.println("Error\n\r\tNot started"); 
    break;
    default: 
    Serial.println("Unknown error"); 
    break;
  }

}

/* if(bufDHT->numElements(bufDHT) >2){
    Serial.println(2);
    bufDHT->pull(bufDHT, &sendBYTE);    
    //Serial.write(sendBYTE);
    
    Serial.println(sendBYTE);
    bufDHT->pull(bufDHT, &sendBYTE);    
    //Serial.write(sendBYTE);
     
    Serial.println(sendBYTE);


 }*/



}



