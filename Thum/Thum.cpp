/*
  temphumidity.cpp - Library for Reading temperature and humidity using DHT11 sensor.
  Created by Micael Monteiro, March 18, 2017.
*/

// Begin of File
  #include "Thum.h"
  #include "DHT.h"
  #define DHTTYPE DHT11                                                 // DHT 11
  #define DHTPIN 2                                                     // Data received in Analog Input 0
  DHT dht(DHTPIN, DHTTYPE);
  
void Thum::set()
  {
    dht.begin();
  }

float Thum::readhum()                                                   // Function whose purpose is to read the humidity and return the value to the user
{
  float humidity = dht.readHumidity();                                  // Read humidity in %
  while (isnan(humidity)){
    Serial.println("Error during humidity acquisition. Retrying!");     // Checks if humidity read failed and exit early.
  }
  return humidity;                                                      
}// end readhumidity

float Thum::readtemp()                                                  // Function whose purpose is to read the temperature and return the value to the user (in Celsius)
{
  float temperature = dht.readTemperature();                            // Read temperature as Celsius
  while(isnan(temperature)){
    Serial.println("Error during temperature acquisition. Retrying!");  // Checks if temperature read failed and exit early.
  }
  return temperature;                                                   
}// end readtemp

void Thum::tprint(float temp)                                           // Method to print the temperature value
{
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(" *C \n");
}// end tprint

void Thum::hprint(float hum)                                            // Method to print the humidity value
{
  Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.print(" *percent \n");
}// end hprint

// End File
