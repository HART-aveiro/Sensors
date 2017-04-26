/*
  temphumidity.h - Library for Reading temperature and humidity using DHT11 sensor.
  Created by Micael Monteiro, March 18, 2017.
*/
#include "Arduino.h"
#ifndef Thum_h
#define Thum_h


class Thum
{

public:									// public parameters - functions prototypes
  void set();
  /*!< Function that sets up the DHT sensor*/

  float readtemp();
  /*!< Function that enables the temperature reading*/

  float readhum();
  /*!< Function that enables the humidity reading*/

  void hprint(float hum);
  /*!< Method whose function is to print the humidity value trough serial communication*/

  void tprint(float temp);
  /*!< Method whose function is to print the temperature value trough serial communication*/

private:
  //nothing here
};
#endif                    			   //ends the #ifndef preprocessor directive
