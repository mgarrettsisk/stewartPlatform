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
double translational[3] = {0.0, 0.0, 0.0};
double rotational[3] = {0.0, 0.0, 0.0};
Adafruit_MPU6050 mpu;
Servo servoOne, servoTwo, servoThree, servoFour, servoFive, servoSix;
double L1, L2, L3, L4, L5, L6;
double alpha1, alpha2, alpha3, alpha4, alpha5, alpha6;

// instantiate matrix/vector variables
// Servo Output Shaft Position Vectors (in base frame)
double B_1[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
double B_2[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
double B_3[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
double B_4[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
double B_5[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};
double B_6[3][1] = {{X_B_1},{Y_B_1},{Z_B_1}};

// Platform Mounting Point Position Vectors (in platform frame)
double P_1[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
double P_2[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
double P_3[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
double P_4[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
double P_5[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};
double P_6[3][1] = {{X_P_1},{Y_P_1},{Z_P_1}};

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
  mpu.setFilterBandwidth(MPU6050_BAND_20_HZ);

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
Serial.println("Stewart Platform Initalizing....");
delay(1000);
}

void loop() {
  //-------------------------------------------------------------------------------------------------------
  //
  //          Intertial Measurement Unit Inputs and Time Step
  //
  //-------------------------------------------------------------------------------------------------------
  /*
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

  // rotational conversions
  double xOmega = OMEGA_0 + xRoll*timeStep;
  double thetaX = THETA_0 + xOmega*timeStep + (1/2)*xRoll*(timeStep^2);
  double yOmega = OMEGA_0 + yRoll*timeStep;
  double thetaY = THETA_0 + yOmega*timeStep + (1/2)*yRoll*(timeStep^2);
  double zOmega = OMEGA_0 + zRoll*timeStep;
  double thetaZ = THETA_0 + zOmega*timeStep + (1/2)*zRoll*(timeStep^2);
  */
  
  //-------------------------------------------------------------------------------------------------------
  //
  //          Translation of Raw Position Commands to PWM Signals for Servo Actuation
  //
  //-------------------------------------------------------------------------------------------------------
  // set desired position


  // Set values in the 
  double T[3][1] = {{translation[0]}, {translation[1]}, {translation[2]}};

  // Compute the rotational matrix
  double Phi = radians(rotation[0]);
  double Theta = radians(rotation[1]);
  double Psi = radians(rotation[2]);
  
  double R = {
              {cos(Phi)*cos(Theta),   -sin(Phi)*cos(Psi)+cos(Phi)*sin(Theta)*sin(Psi),   sin(Phi)*sin(Psi)+cos(Phi)*sin(Theta)*cos(Psi)},
              {sin(Phi)*cos(Theta),   cos(Phi)*cos(Psi)+sin(Phi)*sin(Theta)*sin(Psi),   -cos(Phi)*sin(Psi)+sin(Phi)*sin(Theta)*cos(Psi)},
              {-sin(Theta),           cos(Theta)*sin(Psi),                               cos(Theta)*cos(Psi)}                            
             };
  
  // compute the arm length vectors for each of the six servos
  // Compute product of P_vector and rotational matrix (Note: function equal to P_R_b * P_n = P_R_b_P_n)
  multiplyMatrices(P_R_b, P_1, P_R_b_P_1);
  multiplyMatrices(P_R_b, P_2, P_R_b_P_2);
  multiplyMatrices(P_R_b, P_3, P_R_b_P_3);
  multiplyMatrices(P_R_b, P_4, P_R_b_P_4);
  multiplyMatrices(P_R_b, P_5, P_R_b_P_5);
  multiplyMatrices(P_R_b, P_6, P_R_b_P_6);

  // Add the input translational vector to each of the above matrices (Note: T_vector + P_R_b_P_n = T_P_R_b_P_n)
  addMatrices(T_vector, P_R_b_P_1, T_P_R_b_P_1);
  addMatrices(T_vector, P_R_b_P_2, T_P_R_b_P_2);
  addMatrices(T_vector, P_R_b_P_3, T_P_R_b_P_3);
  addMatrices(T_vector, P_R_b_P_4, T_P_R_b_P_4);
  addMatrices(T_vector, P_R_b_P_5, T_P_R_b_P_5);
  addMatrices(T_vector, P_R_b_P_6, T_P_R_b_P_6);

  // Subtrace the B vector from each of the above matrices (Note: T_P_R_b_P_n - B_n = L_n)
  subtractMatrices(T_P_R_b_P_1, B_1, L_1);
  subtractMatrices(T_P_R_b_P_2, B_2, L_2);
  subtractMatrices(T_P_R_b_P_3, B_3, L_3);
  subtractMatrices(T_P_R_b_P_4, B_4, L_4);
  subtractMatrices(T_P_R_b_P_5, B_5, L_5);
  subtractMatrices(T_P_R_b_P_6, B_6, L_6);

  // calculate the raw lengths of the arms
  L1 = vectorMagnitude(L_1);
  L2 = vectorMagnitude(L_2);
  L3 = vectorMagnitude(L_3);
  L4 = vectorMagnitude(L_4);
  L5 = vectorMagnitude(L_5);
  L6 = vectorMagnitude(L_6);
  
  // calculate alpha for each servo
  alpha1 = calculateServoAngle(T_P_R_b_P_1, B_1, L1, LINKAGE_ARM, SERVO_ARM, radians(BETA_ONE));
  alpha2 = calculateServoAngle(T_P_R_b_P_2, B_2, L2, LINKAGE_ARM, SERVO_ARM, radians(BETA_TWO));
  alpha3 = calculateServoAngle(T_P_R_b_P_3, B_3, L3, LINKAGE_ARM, SERVO_ARM, radians(BETA_THREE));
  alpha4 = calculateServoAngle(T_P_R_b_P_4, B_4, L4, LINKAGE_ARM, SERVO_ARM, radians(BETA_FOUR));
  alpha5 = calculateServoAngle(T_P_R_b_P_5, B_5, L5, LINKAGE_ARM, SERVO_ARM, radians(BETA_FIVE));
  alpha6 = calculateServoAngle(T_P_R_b_P_6, B_6, L6, LINKAGE_ARM, SERVO_ARM, radians(BETA_SIX));

  // move the actual servo

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

double calculateServoAngle(double Q_vector[3][1], double B_vector[3][1], double inputL, double inputS, double inputA, double inputBeta) {
  double alpha;
  // calculate "Big L"
  double bigL = pow(inputL,2) - ( pow(inputS,2) - pow(inputA,2) );
  Serial.print("Big L = ");
  Serial.println(bigL);
  // calculate "Big M"
  double bigM = 2 * inputA * (Q_vector[2][0] - B_vector[2][0]);
  Serial.print("Big M = ");
  Serial.println(bigM);
  // calculate "Big N"
  double bigN = 2 * inputA * (cos(radians(inputBeta))*(Q_vector[0][0] - B_vector[0][0]) + sin(radians(inputBeta))*(Q_vector[1][0] - B_vector[1][0]));
  Serial.print("Big N = ");
  Serial.println(bigN);
  // calculate first element
  alpha = asin(bigL/sqrt(pow(bigM,2)+pow(bigN,2))) - atan(bigN/bigM);
  return alpha;
}
