#ifndef _HARTIMU_H
#define _HARTIMU_H

#define MPU_addr 0x68

      void calibrate_mpu();
      /*!< Function that calibrates the IMU.
           Saves 4000 values and takes the mean value from them.
           This mean values should be used to correct offsets that accumulate during usage*/

      void initialize_imu();
      /*!< First fuction to be used on code, usually on the setup() function
           It starts the IMU I2C connection and calibrates*/

      void read_mpu_values();
      /*!< Function used to read raw values from the IMU, raw values are
      stored internally and cant be accessed*/

      void compute_data();
      /*!< Function that takes raw values and computes them.
           This function calculates the X,Y and Z angles and stores them internally*/
      float get_roll();
      /*!< Function that returns the X angle*/

      float get_pitch();
      /*!< Function that returns the Y angle*/

      float get_yaw();
      /*!< Function that returns the Z angle*/

      float get_temp();
      /*!< Function that returns the Temperature*/

      void start_mpu_6050();
      /*!< Function that starts the IMU I2C connection*/


#endif
