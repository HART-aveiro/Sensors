/*Flame sensor library source file - written by Micael Monteiro
Functionalities: Reads analog values from sensors, converts, filters using Exponential Moving Average (EMA) algorithm and compares to see which one is closer to flame */


#include "flamesensor.h"
#include <Arduino.h>

const int threshold = 300;                                                //threshold value to reject enviornment radiation. 900 in range 0-1023

int ema_filter(int current_value)
{ 
    static uint16_t exponential_average=current_value;
    
    exponential_average=(EMA_ALPHA*(uint32_t)current_value + (100 - EMA_ALPHA)*(uint32_t)exponential_average)/100;      //EMA algorithm
    return exponential_average;
}//end ema_filter

int ema_filter1(int current_value)
{ 
    static uint16_t exponential_average1=current_value;
    
    exponential_average1=(EMA_ALPHA*(uint32_t)current_value + (100 - EMA_ALPHA)*(uint32_t)exponential_average1)/100;    //EMA algorithm
    return exponential_average1;
}//end ema_filter1

int ema_filter2(int current_value)
{ 
    static uint16_t exponential_average2=current_value;
    
    exponential_average2=(EMA_ALPHA*(uint32_t)current_value + (100 - EMA_ALPHA)*(uint32_t)exponential_average2)/100;    //EMA algorithm
    return exponential_average2;
}//end ema_filter2

int getFS1values(){
  const int SensorPin1 = A0;                                                //sensor1 on pin A0                                                                               
  int SensorVal1 = analogRead(SensorPin1);                                  //reads analog value on A0 and stores it in int variable                                                                        
  SensorVal1 = analogRead(SensorPin1);                                      //reads again and this time this value will be used. This to give time to ADC capacitor to charge.                                                                                                                                                                                   // Time for ADC stabilization                                                   						  	// Time for ADC stabilization
  int processed_value1 = ema_filter(SensorVal1);                            //filters the value using EMA                                                            
   if (processed_value1 > threshold)                                        // if reading is bigger than threshold, there is flame. If not 0
    return processed_value1;
  else
    return 0;                                                               
}//end getFS1values

int getFS2values(){                                                         //Equal behaviour as above
  const int SensorPin2 = A1;                                                                                                                                                                                  // Time for ADC stabilization
  int SensorVal2 = analogRead(SensorPin2);																			                                                                    								                                                                       
  SensorVal2 = analogRead(SensorPin2);                                                                                                                                               						  	// Time for ADC stabilization
  int processed_value2 = ema_filter1(SensorVal2);
   if (processed_value2 > threshold)
    return processed_value2;
  
  else
    return 0;
}//end getFS2values

int getFS3values(){                                                         //Equal behaviour as above
  const int SensorPin3 = A3;                                                                                                                                                                                   // Time for ADC stabilization
  int SensorVal3 = analogRead(SensorPin3);																	                                                                          								                                                                       
  SensorVal3 = analogRead(SensorPin3);                                                                                                                                                    						  	// Time for ADC stabilization
  int processed_value3 = ema_filter2(SensorVal3);
  if (processed_value3 > threshold)
    return processed_value3;
  else
    return 0;
}//end getFS3values

void flameposition(int sensor1, int sensor2, int sensor3){
 byte info = 0;                                                               //byte to be conveyed to platform
 if ((sensor1 > sensor2) && (sensor1 > sensor3))                              //closer to sensor 1
 	info = 1;	                                                                  //info=00000001
 else if ((sensor2 > sensor1) && (sensor2 > sensor3))                         //closer to sensor 2
 	info = 2;	                                                                  //info=00000010
 else if ((sensor3 > sensor1) && (sensor3 > sensor2))                         //closer to sensor 3
 	info = 3;	                                                                  //info=00000011
 else if (sensor1 == 0 && sensor2 == 0 && sensor3 == 0)
  Serial.println("No flame");
  
  if(sensor1 !=0 || sensor2 != 0 || sensor3 != 0){                            // all sensors with values above 500
    Serial.print("Data: ");
    Serial.println(info);
  }
}//end flameposition;
