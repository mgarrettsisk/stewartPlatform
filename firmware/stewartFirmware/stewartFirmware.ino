/*
 *  Stewart Platform Firmware
 *  
 *  This software is written to operate a 6-DOF Stewart Platform. It uses an MPU6050 IMU to provide sensor data.
 *  This data is then used to keep the top platform stable as the bottom moves around. 
 *  
 *  This code uses derivations taken from the following sources:
 *  
 *  https://content.instructables.com/ORIG/FFI/8ZXW/I55MMY14/FFI8ZXWI55MMY14.pdf
 *  https://www.xarg.org/paper/inverse-kinematics-of-a-stewart-platform/
 *  
 *  For an in depth description of the mathematics, please see the above links.
 *  
 *  Written by Garrett Sisk
 *  garrett@gsisk.com
 * 
 */
#include "configuration.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <stdio.h>
#include <SPI.h>
#include <String.h>

// create variables/objects
Adafruit_MPU6050 mpu;
Servo servoOne, servoTwo, servoThree, servoFour, servoFive, servoSix;

// instantiate matrix/vector variables
// physical parameter matrices
float A_1[3][1] = {{X_A_1},{Y_A_1},{Z_A_1}};
float A_2[3][1] = {{X_A_2},{Y_A_2},{Z_A_2}};
float A_3[3][1] = {{X_A_3},{Y_A_3},{Z_A_3}};
float A_4[3][1] = {{X_A_4},{Y_A_4},{Z_A_4}};
float A_5[3][1] = {{X_A_5},{Y_A_5},{Z_A_5}};
float A_6[3][1] = {{X_A_6},{Y_A_6},{Z_A_6}};

float B_1[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
float B_2[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
float B_3[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
float B_4[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
float B_5[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
float B_6[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};

float P_1[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
float P_2[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
float P_3[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
float P_4[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
float P_5[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
float P_6[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};


// product matrices (intermittent results)
float P_R_b_P_1[3][1] = {{0.0},{0.0},{0.0}};
float P_R_b_P_2[3][1] = {{0.0},{0.0},{0.0}};
float P_R_b_P_3[3][1] = {{0.0},{0.0},{0.0}};
float P_R_b_P_4[3][1] = {{0.0},{0.0},{0.0}};
float P_R_b_P_5[3][1] = {{0.0},{0.0},{0.0}};
float P_R_b_P_6[3][1] = {{0.0},{0.0},{0.0}};

// summation matrices (intermittent results)
float T_P_R_b_P_1[3][1] = {{0.0},{0.0},{0.0}};
float T_P_R_b_P_2[3][1] = {{0.0},{0.0},{0.0}};
float T_P_R_b_P_3[3][1] = {{0.0},{0.0},{0.0}};
float T_P_R_b_P_4[3][1] = {{0.0},{0.0},{0.0}};
float T_P_R_b_P_5[3][1] = {{0.0},{0.0},{0.0}};
float T_P_R_b_P_6[3][1] = {{0.0},{0.0},{0.0}};

// arm length matrices
float L_1[3][1] = {{0.0},{0.0},{0.0}};
float L_2[3][1] = {{0.0},{0.0},{0.0}};
float L_3[3][1] = {{0.0},{0.0},{0.0}};
float L_4[3][1] = {{0.0},{0.0},{0.0}};
float L_5[3][1] = {{0.0},{0.0},{0.0}};
float L_6[3][1] = {{0.0},{0.0},{0.0}};

void setup() {
  
  // configure the Inertial Measurement Unit (IMU)
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // set up the servo motors and set all of them to home position
  servoOne.attach(SERVO_ONE_PIN);
  servoOne.writeMicroseconds(SERVO_ZERO_PWM);
  servoTwo.attach(SERVO_TWO_PIN);
  servoTwo.writeMicroseconds(SERVO_ZERO_PWM);
  servoThree.attach(SERVO_THREE_PIN);
  servoThree.writeMicroseconds(SERVO_ZERO_PWM);
  servoFour.attach(SERVO_FOUR_PIN);
  servoFour.writeMicroseconds(SERVO_ZERO_PWM);
  servoFive.attach(SERVO_FIVE_PIN);
  servoFive.writeMicroseconds(SERVO_ZERO_PWM);
  servoSix.attach(SERVO_SIX_PIN);  
  servoSix.writeMicroseconds(SERVO_ZERO_PWM);

  // Set up a serial port for debugging

Serial.begin(9600);// opens serial port, sets data rate to 9600
Serial.println("Stewart Platform Initalizing....");
delay(1000);
}

void loop() {
  //-------------------------------------------------------------------------------------------------------
  //
  //          Intertial Measurement Unit Inputs and Time Step
  //
  //-------------------------------------------------------------------------------------------------------

  // get start time for measurement
  int startTime = millis();
  
  // measure the IMU status
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // get X acceleration
  float xAccel = X_OFFSET + a.acceleration.x;
  // get Y acceleration
  float yAccel = Y_OFFSET + a.acceleration.y;
  // get Z acceleration
  float zAccel = Z_OFFSET + a.acceleration.z;
  // get roll acceleration (around X-axis)
  float xRoll = X_ROLL_OFFSET + g.gyro.x;
  // get pitch acceleration (around Y-axis)
  float yRoll = Y_ROLL_OFFSET + g.gyro.y;
  // get heading acceleration (around Z-axis)
  float zRoll = Z_ROLL_OFFSET + g.gyro.z;

  // account for gravity effect in Z-Axis
  zRoll = zRoll + 9.81; // m/s^2

  // get end time for measurement
  int endTime = millis();

  // calculate the time step
  int timeStep = endTime - startTime;

  //-------------------------------------------------------------------------------------------------------
  //
  //          Control Equation Solution Based on IMU Inputs (Second Order Integral)
  //
  //-------------------------------------------------------------------------------------------------------
  
  // translational conversions
  float xVelocity = V_0 + xAccel*timeStep;
  float xPos = S_0 + xVelocity*timeStep + (1/2)*xAccel*(timeStep^2);
  float yVelocity = V_0 + yAccel*timeStep;
  float yPos = S_0 + yVelocity*timeStep + (1/2)*yAccel*(timeStep^2);
  float zVelocity = V_0 + zAccel*timeStep;
  float zPos = S_0 + zVelocity*timeStep + (1/2)*zAccel*(timeStep^2);

  // rotational conversions
  float xOmega = OMEGA_0 + xRoll*timeStep;
  float thetaX = THETA_0 + xOmega*timeStep + (1/2)*xRoll*(timeStep^2);
  float yOmega = OMEGA_0 + yRoll*timeStep;
  float thetaY = THETA_0 + yOmega*timeStep + (1/2)*yRoll*(timeStep^2);
  float zOmega = OMEGA_0 + zRoll*timeStep;
  float thetaZ = THETA_0 + zOmega*timeStep + (1/2)*zRoll*(timeStep^2);

  
  //-------------------------------------------------------------------------------------------------------
  //
  //          Translation of Raw Position Commands to PWM Signals for Servo Actuation
  //
  //-------------------------------------------------------------------------------------------------------

  // Compute the translational input vector based on integrated values from accelerometer sensor
  float T_vector[3][1] = {{xPos}, {yPos}, {zPos}};

  // Compute the rotational matrix based on integrated values from gyroscope sensor
  float P_R_b[3][3] = {
    {cos(thetaZ)*cos(thetaY), (-sin(thetaZ)*cos(thetaX)+cos(thetaZ)*sin(thetaY)*sin(thetaX)), (sin(thetaZ)*sin(thetaX)+cos(thetaZ)*sin(thetaY)*cos(thetaX))},
    {sin(thetaZ)*cos(thetaY), (cos(thetaZ)*cos(thetaX)+sin(thetaZ)*sin(thetaY)*sin(thetaX)), (-cos(thetaZ)*sin(thetaX)+sin(thetaZ)*sin(thetaY)*sin(thetaX))},
    {-sin(thetaY), (cos(thetaY)*sin(thetaX)), (cos(thetaY)*cos(thetaX))}
    };

  // compute the arm length vectors for each of the six servos
  // Compute product of P_vector and rotational matrix
  multiplyMatrices(P_1, P_R_b, P_R_b_P_1);
  multiplyMatrices(P_2, P_R_b, P_R_b_P_2);
  multiplyMatrices(P_3, P_R_b, P_R_b_P_3);
  multiplyMatrices(P_4, P_R_b, P_R_b_P_4);
  multiplyMatrices(P_5, P_R_b, P_R_b_P_5);
  multiplyMatrices(P_6, P_R_b, P_R_b_P_6);

  // Add the input translational vector to each of the above matrices
  addMatrices(T_vector, P_R_b_P_1, T_P_R_b_P_1);
  addMatrices(T_vector, P_R_b_P_2, T_P_R_b_P_2);
  addMatrices(T_vector, P_R_b_P_3, T_P_R_b_P_3);
  addMatrices(T_vector, P_R_b_P_4, T_P_R_b_P_4);
  addMatrices(T_vector, P_R_b_P_5, T_P_R_b_P_5);
  addMatrices(T_vector, P_R_b_P_6, T_P_R_b_P_6);

  // Subtrace the B vector from each of the above matrices
  subtractMatrices(T_P_R_b_P_1, B_1, L_1);
  subtractMatrices(T_P_R_b_P_2, B_2, L_2);
  subtractMatrices(T_P_R_b_P_3, B_3, L_3);
  subtractMatrices(T_P_R_b_P_4, B_4, L_4);
  subtractMatrices(T_P_R_b_P_5, B_5, L_5);
  subtractMatrices(T_P_R_b_P_6, B_6, L_6);

  
}

void multiplyMatrices(float A[3][3], float B[3][1], float C[3][1]) {
  // this method takes three matrices as inputs, performs the matrix multiplication A*B and adds results as
  // elements to C
  
}

void addMatrices(float A[3][1], float B[3][1], float C[3][1]) {
  // this method takes two column vectors of length 3 and adds them together
  
}

void subtractMatrices(float A[3][1], float B[3][1], float C[3][1]) {
  // this method takes two column vectors of length 3 and subtracts them.
  
}
