// This header file sets all the physical parameters necessary for the Stewart Platform to operate.

#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

// Initial Kinematic Conditions
#define S_0               0.0 // meters
#define V_0               0.0 // meters/sec
#define A_0               0.0 // meters/sec^2
#define THETA_0           0.0 // radians
#define OMEGA_0           0.0 // radians/sec
#define ALPHA_0           0.0 // radians/sec^2

// IMU Calibration Parameters
#define X_OFFSET          0.0
#define Y_OFFSET          0.0
#define Z_OFFSET          0.0
#define X_ROLL_OFFSET     0.0
#define Y_ROLL_OFFSET     0.0
#define Z_ROLL_OFFSET     0.0

// Servo Parameters
#define SERVO_ONE_PIN     15
#define SERVO_TWO_PIN     14
#define SERVO_THREE_PIN   2
#define SERVO_FOUR_PIN    3
#define SERVO_FIVE_PIN    4
#define SERVO_SIX_PIN     5
#define SERVO_ZERO_PWM    1500
#define SERVO_MAX_PWM     2000
#define SERVO_MIN_PWM     1000


// Physical Dimension Parameters - all values given in millimeters. Set "B" is defined in the base reference frame.
// Set "P" is defined in the top reference frame. All values are given from their respective origins, and have been taken
// from the Solidworks Assembly using the "Evaluate -> Measure" tool. 

// NOTE: IT IS VITAL THAT THE PARAMETERS BELOW ARE CORRECT. COMPUTATIONAL ERRORS CAN OCCUR IF THE VALUES DO NOT CORRESPOND TO
//        ACTUAL PHYSICAL MEASUREMENTS. 

// NOTE: THE BASE AND PLATFORM COORDINATE SYSTEMS ARE COPLANAR IN X AND Y, BUT ARE SEPARATED BY A DISTANCE H_0 IN THE Z-AXIS

// Linkage Length Parameters
#define SERVO_ARM         24.0
#define LINKAGE_ARM       122.5
#define H_0               120.7606
#define ALPHA_0           10.2671 // degrees

// For Servo One
#define X_B_1             73.5
#define Y_B_1             -38.25
#define Z_B_1             0.0
#define X_P_1             75.0
#define Y_P_1             -7.43
#define Z_P_1             0.0
#define BETA_ONE          -90.0// degrees
// For Servo Two
#define X_B_2             73.5
#define Y_B_2             38.25
#define Z_B_2             0.0
#define X_P_2             75.0
#define Y_P_2             7.43
#define Z_P_2             0.0
#define BETA_TWO          90.0 // degrees
// For Servo Three
#define X_B_3             3.62
#define Y_B_3             82.78
#define Z_B_3             0.0
#define X_P_3             -31.06
#define Y_P_3             68.67
#define Z_P_3             0.0
#define BETA_THREE        30.0 // degrees
// For Servo Four
#define X_B_4             -69.88
#define Y_B_4             44.53
#define Z_B_4             0.0
#define X_P_4             -43.94
#define Y_P_4             61.24
#define Z_P_4             0.0
#define BETA_FOUR         30.0 // degrees
// For Servo Five
#define X_B_5             -69.88
#define Y_B_5             -44.53
#define Z_B_5             0.0
#define X_P_5             -43.94
#define Y_P_5             -61.24
#define Z_P_5             0.0
#define BETA_FIVE         -30.0 // degrees
// For Servo Six
#define X_B_6             3.62
#define Y_B_6             -82.78
#define Z_B_6             0.0
#define X_P_6             -31.06
#define Y_P_6             -68.67
#define Z_P_6             0.0
#define BETA_SIX          -30.0 // degrees

#endif // _CONFIGURATION_H
