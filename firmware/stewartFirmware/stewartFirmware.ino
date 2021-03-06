/*
 *  Stewart Platform Firmware
 *  
 *  This software is written to operate a 6-DOF Stewart Platform. It uses an MPU6050 IMU to provide sensor data.
 *  Full implementation of this data has yet to occur, however the results are displayed in a serial terminal. 
 *  
 *  This code uses derivations taken from the following sources:
 *  
 *  https://content.instructables.com/ORIG/FFI/8ZXW/I55MMY14/FFI8ZXWI55MMY14.pdf
 *  https://www.xarg.org/paper/inverse-kinematics-of-a-stewart-platform/
 *  
 *  For an in depth description of the mathematics, please see the above links. Note that the derivation given in
 *  source 1 (instructables link) has an error in the derivation that causes the formulas to fail. 
 *  
 *  When deriving the results for l^2 and s^2, the P vector values are used, and this is incorrect. They need to 
 *  be the Q vector values (vector between the origin of the base and platform mounting point.) Another script
 *  has been written to more easily diagnose issues with the transformations involved. See mathTestScript.m in the
 *  parent directory. It must be run in MATLAB and mirrors the mathematics performed here.
 *  
 *  Written by Garrett Sisk
 *  garrett@gsisk.com or msisk2@students.kennesaw.edu
 * 
 */
#include "configuration.h"

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <stdio.h>
#include <SPI.h>
#include <String.h>
#include <math.h>

// create variables/objects
double translation[3] = {0.0, 0.0, 0.0};
double rotation[3] = {0.0, 0.0, 0.0};
Adafruit_MPU6050 mpu;
Servo servoOne, servoTwo, servoThree, servoFour, servoFive, servoSix;
double L1, L2, L3, L4, L5, L6;
double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;

String inputString;


// instantiate matrix/vector variables
// Servo Output Shaft Position Vectors (in base frame)
double B_1[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
double B_2[3][1] = {{X_B_2},{Y_B_2},{Z_B_2}};
double B_3[3][1] = {{X_B_3},{Y_B_3},{Z_B_3}};
double B_4[3][1] = {{X_B_4},{Y_B_4},{Z_B_4}};
double B_5[3][1] = {{X_B_5},{Y_B_5},{Z_B_5}};
double B_6[3][1] = {{X_B_6},{Y_B_6},{Z_B_6}};

// Platform Mounting Point Position Vectors (in platform frame)
double P_1[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
double P_2[3][1] = {{X_P_2},{Y_P_2},{Z_P_2}};
double P_3[3][1] = {{X_P_3},{Y_P_3},{Z_P_3}};
double P_4[3][1] = {{X_P_4},{Y_P_4},{Z_P_4}};
double P_5[3][1] = {{X_P_5},{Y_P_5},{Z_P_5}};
double P_6[3][1] = {{X_P_6},{Y_P_6},{Z_P_6}};

// product matrices (intermittent results)
double RP_1[3][1] = {{0.0},{0.0},{0.0}};
double RP_2[3][1] = {{0.0},{0.0},{0.0}};
double RP_3[3][1] = {{0.0},{0.0},{0.0}};
double RP_4[3][1] = {{0.0},{0.0},{0.0}};
double RP_5[3][1] = {{0.0},{0.0},{0.0}};
double RP_6[3][1] = {{0.0},{0.0},{0.0}};

// summation matrices (intermittent results)
double Q_1[3][1] = {{0.0},{0.0},{0.0}};
double Q_2[3][1] = {{0.0},{0.0},{0.0}};
double Q_3[3][1] = {{0.0},{0.0},{0.0}};
double Q_4[3][1] = {{0.0},{0.0},{0.0}};
double Q_5[3][1] = {{0.0},{0.0},{0.0}};
double Q_6[3][1] = {{0.0},{0.0},{0.0}};

// arm length matrices (final results, prior to servo conversion)
double L_1[3][1] = {{0.0},{0.0},{0.0}};
double L_2[3][1] = {{0.0},{0.0},{0.0}};
double L_3[3][1] = {{0.0},{0.0},{0.0}};
double L_4[3][1] = {{0.0},{0.0},{0.0}};
double L_5[3][1] = {{0.0},{0.0},{0.0}};
double L_6[3][1] = {{0.0},{0.0},{0.0}};

void setup() {
  
  // configure the Inertial Measurement Unit (IMU)
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // set up the servo motors and set all of them to home position
  servoOne.attach(SERVO_ONE_PIN,SERVO_MIN_PWM, SERVO_MAX_PWM);
  servoOne.writeMicroseconds(SERVO_ZERO_PWM);
  
  servoTwo.attach(SERVO_TWO_PIN, SERVO_MIN_PWM, SERVO_MAX_PWM);
  servoTwo.writeMicroseconds(SERVO_ZERO_PWM);
  
  servoThree.attach(SERVO_THREE_PIN, SERVO_MIN_PWM, SERVO_MAX_PWM);
  servoThree.writeMicroseconds(SERVO_ZERO_PWM);
  
  servoFour.attach(SERVO_FOUR_PIN, SERVO_MIN_PWM, SERVO_MAX_PWM);
  servoFour.writeMicroseconds(SERVO_ZERO_PWM);
  
  servoFive.attach(SERVO_FIVE_PIN, SERVO_MIN_PWM, SERVO_MAX_PWM);
  servoFive.writeMicroseconds(SERVO_ZERO_PWM);
  
  servoSix.attach(SERVO_SIX_PIN, SERVO_MIN_PWM, SERVO_MAX_PWM);  
  servoSix.writeMicroseconds(SERVO_ZERO_PWM);

  // Set up a serial port for debugging

Serial.begin(115200);// opens serial port, sets data rate to 115200 bps
if( !Serial ) {
  // wait for the serial to connect
}
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
  double xAccel = X_OFFSET + a.acceleration.x;
  // get Y acceleration
  double yAccel = Y_OFFSET + a.acceleration.y;
  // get Z acceleration
  double zAccel = Z_OFFSET + a.acceleration.z;
  // get roll acceleration (around X-axis)
  double xRoll = X_ROLL_OFFSET + g.gyro.x;
  // get pitch acceleration (around Y-axis)
  double yRoll = Y_ROLL_OFFSET + g.gyro.y;
  // get heading acceleration (around Z-axis)
  double zRoll = Z_ROLL_OFFSET + g.gyro.z;

  // account for gravity effect in Z-Axis
  zAccel = zAccel + 9.81; // m/s^2

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
  double xVelocity = V_0 + xAccel*timeStep;
  double xPos = S_0 + xVelocity*timeStep + (1/2)*xAccel*(timeStep^2);
  double yVelocity = V_0 + yAccel*timeStep;
  double yPos = S_0 + yVelocity*timeStep + (1/2)*yAccel*(timeStep^2);
  double zVelocity = V_0 + zAccel*timeStep;
  double zPos = S_0 + zVelocity*timeStep + (1/2)*zAccel*(timeStep^2);
  Serial.print("Translational Values: X: "); Serial.print(xPos); Serial.print(", Y:"); Serial.print(yPos);
  Serial.print(", Z"); Serial.print(zPos); Serial.println();

  // rotational conversions
  double xOmega = OMEGA_0 + xRoll*timeStep;
  double thetaX = THETA_0 + xOmega*timeStep + (1/2)*xRoll*(timeStep^2);
  double yOmega = OMEGA_0 + yRoll*timeStep;
  double thetaY = THETA_0 + yOmega*timeStep + (1/2)*yRoll*(timeStep^2);
  double zOmega = OMEGA_0 + zRoll*timeStep;
  double thetaZ = THETA_0 + zOmega*timeStep + (1/2)*zRoll*(timeStep^2);
  Serial.print("Rotational Values: X:"); Serial.print(thetaX); Serial.print(", Y:"); Serial.print(thetaY);
  Serial.print(", Z:"); Serial.print(thetaZ); Serial.println();
  
  //-------------------------------------------------------------------------------------------------------
  //
  //          Translation of Raw Position Commands to PWM Signals for Servo Actuation
  //
  //-------------------------------------------------------------------------------------------------------
  // set desired position
  if (Serial.available() > 0) {
      inputString = Serial.readStringUntil("\n");
  }

  //translation[1] = (double)inputString.toInt();
  translation[0] = (double)inputString.toInt();

  // Set values in the 
  double T[3][1] = {{translation[0]}, {translation[1]}, {translation[2]+H_0}};

  // Compute the rotational matrix
  double Phi = radians(rotation[0]);
  double Theta = radians(rotation[1]);
  double Psi = radians(rotation[2]);
  
  double R[3][3] = {
              {cos(Phi)*cos(Theta),   -sin(Phi)*cos(Psi)+cos(Phi)*sin(Theta)*sin(Psi),   sin(Phi)*sin(Psi)+cos(Phi)*sin(Theta)*cos(Psi)},
              {sin(Phi)*cos(Theta),   cos(Phi)*cos(Psi)+sin(Phi)*sin(Theta)*sin(Psi),   -cos(Phi)*sin(Psi)+sin(Phi)*sin(Theta)*cos(Psi)},
              {-sin(Theta),           cos(Theta)*sin(Psi),                               cos(Theta)*cos(Psi)}                            
             };
  
  // Compute product of P_vector and rotational matrix (Note: function equal to P_R_b * P_n = P_R_b_P_n)
  multiplyMatrices(R, P_1, RP_1);
  multiplyMatrices(R, P_2, RP_2);
  multiplyMatrices(R, P_3, RP_3);
  multiplyMatrices(R, P_4, RP_4);
  multiplyMatrices(R, P_5, RP_5);
  multiplyMatrices(R, P_6, RP_6);

  // Add the input translational vector to each of the above matrices (Note: T_vector + P_R_b_P_n = T_P_R_b_P_n)
  addMatrices(T, RP_1, Q_1);
  addMatrices(T, RP_2, Q_2);
  addMatrices(T, RP_3, Q_3);
  addMatrices(T, RP_4, Q_4);
  addMatrices(T, RP_5, Q_5);
  addMatrices(T, RP_6, Q_6);
  
  // Subtrace the B vector from each of the above matrices (Note: T_P_R_b_P_n - B_n = L_n)
  subtractMatrices(Q_1, B_1, L_1);
  subtractMatrices(Q_2, B_2, L_2);
  subtractMatrices(Q_3, B_3, L_3);
  subtractMatrices(Q_4, B_4, L_4);
  subtractMatrices(Q_5, B_5, L_5);
  subtractMatrices(Q_6, B_6, L_6);

  // calculate the raw lengths of the arms
  L1 = vectorMagnitude(L_1);
  L2 = vectorMagnitude(L_2);
  L3 = vectorMagnitude(L_3);
  L4 = vectorMagnitude(L_4);
  L5 = vectorMagnitude(L_5);
  L6 = vectorMagnitude(L_6);

  // calculate alpha for each servo
  alpha1 = calculateServoAngle(Q_1, B_1, L1, BETA_ONE);
  alpha2 = calculateServoAngle(Q_2, B_2, L2, BETA_TWO);
  alpha3 = calculateServoAngle(Q_3, B_3, L3, BETA_THREE);
  alpha4 = calculateServoAngle(Q_4, B_4, L4, BETA_FOUR);
  alpha5 = calculateServoAngle(Q_5, B_5, L5, BETA_FIVE);
  alpha6 = calculateServoAngle(Q_6, B_6, L6, BETA_SIX);

  // move the actual servo
  servoOne.writeMicroseconds(convertAngleToPWM(alpha1, 1));
  servoTwo.writeMicroseconds(convertAngleToPWM(alpha2, 2));
  servoThree.writeMicroseconds(convertAngleToPWM(alpha3, 3));
  servoFour.writeMicroseconds(convertAngleToPWM(alpha4, 4));
  servoFive.writeMicroseconds(convertAngleToPWM(alpha5, 5));
  servoSix.writeMicroseconds(convertAngleToPWM(alpha6, 6));

  delay(2000);
}


//-------------------------------------------------------------------------------------------------------
//
//          Utility and Mathematic Functions
//
//-------------------------------------------------------------------------------------------------------

void multiplyMatrices(double A[3][3], double B[3][1], double C[3][1]) {
  // this method takes three matrices as inputs, performs the matrix multiplication A*B and adds results as
  // elements to C. Only a single loop is needed since the output is a 3x1 vector.
  for (int i = 0; i < 3; i++) {
    C[i][0] = (A[i][0]*B[i][0]) + (A[i][1]*B[i][0]) + (A[i][2]*B[i][0]);
    }
  }

void addMatrices(double A[3][1], double B[3][1], double C[3][1]) {
  // this method takes two column vectors of length 3 and adds them together
  for (int i = 0; i < 3; i++) {
    C[i][0] = A[i][0] + B[i][0];
  }
}

void subtractMatrices(double A[3][1], double B[3][1], double C[3][1]) {
  // this method takes two column vectors of length 3 and subtracts them.
  for (int i = 0; i < 3; i++) {
    C[i][0] = A[i][0] - B[i][0];
  }
}

double vectorMagnitude(double A[3][1]) {
  // this method returns the scalar magnitude of the vector provided
  double output = sqrt( pow(A[0][0],2) + pow(A[1][0],2) + pow(A[2][0],2) );
  return output;
}

double calculateServoAngle(double Q_vector[3][1], double B_vector[3][1], double inputL, double inputBeta) {
  // calculate "Big L"
  double bigL = pow(inputL,2) - ( pow(LINKAGE_ARM,2) - pow(SERVO_ARM,2) );
  // calculate "Big M"
  double bigM = 2 * SERVO_ARM * (Q_vector[2][0] - B_vector[2][0]);
  // calculate "Big N"
  double bigN = 2 * SERVO_ARM * (cos(radians(inputBeta))*(Q_vector[0][0] - B_vector[0][0]) + sin(radians(inputBeta))*(Q_vector[1][0] - B_vector[1][0]));
  // calculate alpha
  return asin(bigL/sqrt(pow(bigM,2)+pow(bigN,2))) - atan(bigN/bigM);
}

double convertAngleToPWM(double inputAngle, int inputType) {
  // if the input type is 1, the servo motor is an odd entry. If the input type is 2, the servo motor is an even entry
  if (inputType == 2 || inputType == 4 || inputType == 6) { // if the type is even
    return SERVO_ZERO_PWM - (inputAngle - radians(ALPHA_0)) * SERVO_CONVERT;
  } else {
    return SERVO_ZERO_PWM + (inputAngle - radians(ALPHA_0)) * SERVO_CONVERT;
  }
}
