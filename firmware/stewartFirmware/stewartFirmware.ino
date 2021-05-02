/*
 *  Stewart Platform Firmware
 *  
 *  This software is written to operate a 6-DOF Stewart Platform. It uses an MPU6050 IMU to provide sensor data.
 *  This data is then used to keep the top platform stable as the bottom moves around. 
 *  
 *  This code uses portions from the following sources:
 *  
 *  
 *  
 *  Written by Garrett Sisk
 *  garrett@gsisk.com
 * 
 */

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  // measure the IMU status

  // integrate each twice to determine location data

  // send location data to platform transformation library to write servo positions

  // repeat

}
