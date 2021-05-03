/*
 *  Stewart Platform Firmware
 *  
 *  This software is written to operate a 6-DOF Stewart Platform. It uses an MPU6050 IMU to provide sensor data.
 *  This data is then used to keep the top platform stable as the bottom moves around. 
 *  
 *  This code uses portions from the following sources:
 *  
 *  https://content.instructables.com/ORIG/FFI/8ZXW/I55MMY14/FFI8ZXWI55MMY14.pdf
 *  https://www.xarg.org/paper/inverse-kinematics-of-a-stewart-platform/
 *  
 *  
 *  Written by Garrett Sisk
 *  garrett@gsisk.com
 * 
 */

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <stdio.h>
#include <SPI.h>
#include <String.h>

// create variables/objects
Adafruit_MPU6050 mpu;
const int servo_pin[6] = {15, 14, 2, 3, 4, 5};  //3,6,10 Reversed
const int servo_zero[6] = {1500, 1500, 1500, 1500, 1500, 1500};
Servo servo[6];

float xOffset = 0.0;
float yOffset = 0.0;
float zOffset = 0.0;
float xRollOffset = 0.0;
float yRollOffset = 0.0;
float zRollOffset = 0.0;

float s_0 = 0.0; // m
float v_0 = 0.0; // m/s
float a_0 = 0.0; // m/s^2
float alpha_0 = 0.0; // rad/s^2
float omega_0 = 0.0; // rad/s
float theta_0 = 0.0; // rad

// physical plant vectors, they are of the format {X, Y, Z}
const float pi = radians(180), 
  theta_s[6] = {0, 0, 30, 30, -30, -30}, // angles of servo faces in degrees from +X axis
  L1 = 0.666, 
  L2 = 6, 
  z_home = 100.0,
  YPR_max = radians(20), 
  servo_min = radians(-60), 
  servo_max = radians(60), 
  feedback = (1),
  servo_mult = (1800/pi), 
  ADC_mult = (YPR_max/512),

  // p = location of servo rotation points in base frame [x/y][1-6]
  p[2][6] = {{-52.3993, 50.8007, 90.3895, 39.5875, -38.2649, -90.3679}  ,
            {74.8623, 74.8623, 6.2712, -80.5588, -80.4696, 5.8204}},
            
  // re = location of attachment points in end effector frame [x/y][1-6]
  re[3][6] = {{-12.0000,  12.0000,  70.2820,  58.2820,  -58.2820,  -70.2820},
             {74.2260,  74.2260,  -26.7210,  -47.5060,  -47.5060,  -26.7210},
             {-21.075,  -21.075,  -21.075,   -21.075,   -21.075,   -21.075}};

void setup() {
  
  // configure the Inertial Measurement Unit (IMU)
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
/*
  // set up the servo motors and set all of them to home position
  servoOne.attach(15);
  servoOne.writeMicroseconds(1500);
  servoTwo.attach(14);
  servoTwo.writeMicroseconds(1500);
  servoThree.attach(2);
  servoThree.writeMicroseconds(1500);
  servoFour.attach(3);
  servoFour.writeMicroseconds(1500);
  servoFive.attach(4);
  servoFive.writeMicroseconds(1500);
  servoSix.attach(5);  
  servoSix.writeMicroseconds(1500);
*/
  // Set up a serial port for debugging

Serial.begin(9600);// opens serial port, sets data rate to 9600
 for(int i = 0; i < 6; i++)
 {
   servo[i].attach(servo_pin[i]);
   servo[i].writeMicroseconds(servo_zero[i]);
 }
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
  float xAccel = xOffset + a.acceleration.x;
  // get Y acceleration
  float yAccel = yOffset + a.acceleration.y;
  // get Z acceleration
  float zAccel = zOffset + a.acceleration.z;
  // get roll acceleration (around X-axis)
  float xRoll = xRollOffset + g.gyro.x;
  // get pitch acceleration (around Y-axis)
  float yRoll = yRollOffset + g.gyro.y;
  // get heading acceleration (around Z-axis)
  float zRoll = zRollOffset + g.gyro.z;

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
  float xVelocity = v_0 + xAccel*timeStep;
  float xPos = s_0 + xVelocity*timeStep + (1/2)*xAccel*(timeStep^2);
  float yVelocity = v_0 + yAccel*timeStep;
  float yPos = s_0 + yVelocity*timeStep + (1/2)*yAccel*(timeStep^2);
  float zVelocity = v_0 + zAccel*timeStep;
  float zPos = s_0 + zVelocity*timeStep + (1/2)*zAccel*(timeStep^2);

  // rotational conversions
  float xOmega = omega_0 + xRoll*timeStep;
  float thetaX = theta_0 + xOmega*timeStep + (1/2)*xRoll*(timeStep^2);
  float yOmega = omega_0 + yRoll*timeStep;
  float thetaY = theta_0 + yOmega*timeStep + (1/2)*yRoll*(timeStep^2);
  float zOmega = omega_0 + zRoll*timeStep;
  float thetaZ = theta_0 + zOmega*timeStep + (1/2)*zRoll*(timeStep^2);

  
  //-------------------------------------------------------------------------------------------------------
  //
  //          Translation of Raw Position Commands to PWM Signals for Servo Actuation
  //
  //-------------------------------------------------------------------------------------------------------
  float pe[6] = {0,0,0,radians(45),radians(45),radians(45)}, 
        theta_a[6], 
        servo_pos[6],
        q[3][6], 
        r[3][6], 
        dl[3][6], 
        dl2[6];

  Serial.println("SERVO");
 
  for(int i = 0; i < 6; i++) {
  /*    theta_1=0, Y=0, Z=0, P3=0
       |   C2C3    0    0    P1  |     |  X  |    |  q[0]  |    | X(C2C3) + P1 |
       |   C2S3    0    0    P2  |  x  |  0  |  = |  q[1]  | =  | X(C2S3) + P2 |
       |   -S2     0    0     0  |     |  0  |    |  q[2]  |    | X(-S2)       |
       |    0      0    0     1  |     |  1  |    |  1     |    |    1         |
  */
  // theta_a = angle of the servo arm
  // theta_s = orientation of the servos
     
  if (i%2 == 0) {
    q[0][i] = L1*cos(-theta_a[i])*cos(theta_s[i]) + p[0][i];  //EVEN [0,2,4]
    q[1][i] = L1*cos(-theta_a[i])*sin(theta_s[i]) + p[1][i];
    q[2][i] = -L1*sin(-theta_a[i]);
  } else { 
    q[0][i] = -L1*cos(theta_a[i])*cos(theta_s[i]) + p[0][i];  //ODD  [1,3,5]
    q[1][i] = -L1*cos(theta_a[i])*sin(theta_s[i]) + p[1][i];
    q[2][i] = L1*sin(theta_a[i]);
  }
  
  // Z = 0 to simplify equation
  // r = position of upper mounting point of connecting link
  // re = location of attachment points in end effector frame [x/y][1-6]
  // pe = location and orientation of end effector frame relative to the base frame [sway, surge, heave, pitch, roll, yaw)
        
  // Calculation of Rotational Matrix from basic equation //
  r[0][i] = re[0][i] * cos(pe[4]) * cos(pe[5]) + re[1][i] * (sin(pe[3]) * sin(pe[4]) * cos(pe[5]) - cos(pe[3]) * sin(pe[5])) + re[2][i] * ((cos(pe[3]) * sin(pe[4]) * cos(pe[5])) + sin(pe[3]) * sin(pe[5])) + pe[0];
  r[1][i] = re[0][i] * cos(pe[4]) * sin(pe[5]) + re[1][i] * (cos(pe[3]) * cos(pe[5]) + sin(pe[3]) * sin(pe[4]) * sin(pe[5])) + re[2][i] * ((cos(pe[3]) * sin(pe[4]) * cos(pe[5])) - sin(pe[3]) * cos(pe[5])) + pe[1];
  r[2][i] = -re[0][i] * sin(pe[4]) + re[1][i] * sin(pe[3]) * cos(pe[4]) + re[2][i] * (cos(pe[3]) * cos(pe[4])) + z_home + pe[2];

  // dl = difference between x,y,z coordinates of q and r
  dl[0][i] = q[0][i] - r[0][i];
  dl[1][i] = q[1][i] - r[1][i];
  dl[2][i] = q[2][i] - r[2][i];

  // dl2 = distance between q and r
  // L2 = connecting arm length
  dl2[i] = sqrt(dl[0][i]*dl[0][i] + dl[1][i]*dl[1][i] + dl[2][i]*dl[2][i]) - L2;

  // feedback in fractions of (1/x),
  // higher x --> reach target location at SLOWER speed but LESS oscillation.
  //  lower x --> reach target location at FASTER speed but MORE oscillation.
  theta_a[i] += (dl2[i]*feedback);

  
//Serial.println(dl2);
 for (int j = 0; j < 6; j++ ){
  Serial.println();
  Serial.print("theta_a value at index ");
  Serial.print(j);
  Serial.print(" and with value ");
  Serial.println(theta_a[i]);
 }
   
    // theta_a = angle of the servo arm in radians
    theta_a[i] = constrain(theta_a[i], servo_min, servo_max);
           
   if(i%2 == 0) servo_pos[i] = servo_zero[i] + theta_a[i]*servo_mult;    //EVEN - normal
   else servo_pos[i] = servo_zero[i] - theta_a[i]*servo_mult;            //ODD - reversed
   //   if(i%2 == 1) servo_pos = servo_zero + theta_a*servo_mult;
   //   else servo_pos = servo_zero - theta_a*servo_mult;
 }
   
 for(int i = 0; i < 6; i++)
 {
  Serial.println();
  Serial.print("Writing to servo with value: ");
  Serial.println(servo_pos[i]);
   servo[i].writeMicroseconds(servo_pos[i]);
 }
delay(500);
/*
  float P_R_b[3][3] = {
    {cos(thetaZ)*cos(thetaY), (-sin(thetaZ)*cos(thetaX)+cos(thetaZ)*sin(thetaY)*sin(thetaX)), (sin(thetaZ)*sin(thetaX)+cos(thetaZ)*sin(thetaY)*cos(thetaX))},
    {sin(thetaZ)*cos(thetaY), (cos(thetaZ)*cos(thetaX)+sin(thetaZ)*sin(thetaY)*sin(thetaX)), (-cos(thetaZ)*sin(thetaX)+sin(thetaZ)*sin(thetaY)*sin(thetaX))},
    {-sin(thetaY), (cos(thetaY)*sin(thetaX)), (cos(thetaY)*cos(thetaX))}
    };
*/
}

float multiplyMatrix(float A, float B) {
  for (int i=0; i < 
}
