#include <RingBuf.h>

#include <TimerOne.h>
#include "hartimu.h"
#include <Wire.h>

const int led = 13;  // the pin with a LED

RingBuf *bufMPU = RingBuf_new(sizeof(short), 21);

short teste=-741; 

void setup(void){
  //noInterrupts();
  pinMode(led, OUTPUT);
  digitalWrite(led,LOW);


  initialize_imu();
  Serial.begin(115200);
  //Serial.println("Hello im working");

  


  Timer1.initialize(1000);

  Timer1.attachInterrupt(getSensors); // blinkLED to run every 0.15 seconds
  Timer1.start();
}

int flag=0;

int countMPU=0;
short temp;

void getSensors(void){
  digitalWrite(led, HIGH);
  //Serial.write('i');
  
  countMPU+=1; 
  if(countMPU==4){
    
    countMPU=0;
    read_mpu_values();
    compute_data();
    temp= (short) get_yaw()*10;
//    bufMPU->RingBufAdd(bufMPU, &temp);
//    temp= (short) get_roll()*10;
//    bufMPU->add(bufMPU, &temp);
//    temp= (short) get_pitch();
    
//    bufMPU->add(bufMPU, &temp);


    
  }
         
  digitalWrite(led, LOW);
  
  
}

union sendMPU{
  short send1;
  byte send2[2];
}sendMPU;

void loop(void)
{
//  Serial.println(get_roll());
//  if(bufMPU->numElements(bufMPU) >3){
//  
    bufMPU->add(bufMPU, &teste);
    bufMPU->pull(bufMPU, &sendMPU);
    
    Serial.write(sendMPU.send2[0]);
    Serial.write(sendMPU.send2[1]);

//  }

  
  
}



