#include <Wire.h>
#include <Arduino.h>
#include "hartimu.h"

  long AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ,GyXant,GyYant,GyZant;
  long gyro_x_calc,gyro_y_calc,gyro_z_calc;
  float angle_roll, angle_pitch, angle_yaw;
  float total_vect_acc,angle_pitch_acc,angle_roll_acc;

  void start_mpu_6050(){
   // Wire.setClock(400000L);

    
    //Activate the MPU-6050
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x6B);                                                    //Send the requested starting register
    Wire.write(0x00);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x1C);                                                    //Send the requested starting register
    Wire.write(0x10);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
    //Configure the gyro (500dps full scale)
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x1B);                                                    //Send the requested starting register
    Wire.write(0x08);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
  }
  
  void read_mpu_values(){
    interrupts();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    while(Wire.available() < 14);    //Wait until all the bytes are received
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    
  }

  void calibrate_mpu(){

    for(int i=0;i<4000;i++){
      read_mpu_values();

      gyro_x_calc += GyX;
      gyro_y_calc += GyY;
      gyro_z_calc += GyZ;
    }

    gyro_x_calc /= 4000;
    gyro_y_calc /= 4000;
    gyro_z_calc /= 4000;

  }

  void initialize_imu(){
      start_mpu_6050();
      calibrate_mpu();
  }

  void compute_data(){
    //gyro data treatment
    GyX -= gyro_x_calc;
    GyY -= gyro_y_calc;
    GyZ -= gyro_z_calc;

    angle_roll += GyX*0.004/65.5;
    angle_pitch += GyY*0.004/65.5;
    angle_yaw += GyZ*0.004/65.5;

    float sinGyZ =sin(GyZ*0.000001066);
    angle_roll -= angle_pitch * sinGyZ;
    angle_pitch += angle_roll * sinGyZ;

    //accel data treatment
    total_vect_acc = sqrt((AcX*AcX) + (AcY*AcY) + (AcZ*AcZ));
    angle_pitch_acc = asin((float)AcX/total_vect_acc)*-57.296;//180/pi = 57.296
    angle_roll_acc = asin((float)AcY/total_vect_acc)*57.296;//180/pi = 57.296

    angle_roll_acc -= -0.7;
    angle_pitch_acc -= -0.8;

    //total roll and pitch angle are a combination of gyro and accel
    //since the gyro drifts a little over time we use the accel to compensate it
    //because of the vibrations, only the average value of the accel is usable
    //and we should only use a bit of bit, just to compensate the gyros drift

    angle_roll = angle_roll*0.9996 + angle_roll_acc*0.0004;
    angle_pitch = angle_pitch*0.9996 + angle_pitch_acc*0.0004;

  }

  float get_roll(){
    return angle_roll;
  }

  float get_pitch(){
    return angle_pitch;
  }

  float get_yaw(){
    return angle_yaw;
  }

  float get_temp(){
    return Tmp;
  }

