#include <RingBuf.h>

#include <TimerOne.h>
#include "hartimu.h"
#include <Wire.h>

#include <idDHT11.h>



const int led = 13;  // the pin with a LED


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

  initialize_imu();
  Serial.begin(115200);
  Serial.println("Hello im working");

  
//  lastTime=millis();

  Timer1.initialize(1000);

  Timer1.attachInterrupt(getSensors); // blinkLED to run every 0.15 seconds
  Timer1.start();
}

void dht11_wrapper() {
  DHT11.isrCallback();
}
int flag=0;

int countMPU=0;
volatile int countTemp=0;
short temp;


void getSensors(void){

  //Serial.write('i');

  countMPU+=1; 
  countTemp+=1;
  
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

  //digitalWrite(led, LOW);
}


union sendSHORT{
  short send1;
  byte send2[2];
}sendSHORT;




void loop(void){
//  Serial.println(get_roll());
 if(bufMPU->numElements(bufMPU) >3){
  Serial.print(3);
  Serial.print("            ");


  bufMPU->pull(bufMPU, &sendSHORT);
  Serial.print(sendSHORT.send1 );

  Serial.print("            ");

  bufMPU->pull(bufMPU, &sendSHORT);
  Serial.print(sendSHORT.send1 );

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



