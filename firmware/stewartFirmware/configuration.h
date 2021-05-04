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

// IMU Calibration Parameters
#define X_OFFSET          0.0
#define Y_OFFSET          0.0
#define Z_OFFSET          0.0
#define X_ROLL_OFFSET     0.0
#define Y_ROLL_OFFSET     0.0
#define Z_ROLL_OFFSET     0.0

// Physical Dimension Parameters - all values given in millimeters. Sets "A" and "B" are defined in the base reference frame.
// Set "P" is defined in the top reference frame. All values are given from their respective origins, and have beent taken
// from the Solidworks Assembly using the "Evaluate -> Measure" tool. 

// For Servo One
#define x_A_1
#define y_A_1
#define z_A_1
#define x_B_1
#define y_B_1
#define z_B_1
#define x_P_1
#define y_P_1
#define z_P_1
// For Servo Two
#define x_A_2
#define y_A_2
#define z_A_2
#define x_B_2
#define y_B_2
#define z_B_2
#define x_P_2
#define y_P_2
#define z_P_2
// For Servo Three
#define x_A_3
#define y_A_3
#define z_A_3
#define x_B_3
#define y_B_3
#define z_B_3
#define x_P_3
#define y_P_3
#define z_P_3
// For Servo Four
#define x_A_4
#define y_A_4
#define z_A_4
#define x_B_4
#define y_B_4
#define z_B_4
#define x_P_4
#define y_P_4
#define z_P_4
// For Servo Five
#define x_A_5
#define y_A_5
#define z_A_5
#define x_B_5
#define y_B_5
#define z_B_5
#define x_P_5
#define y_P_5
#define z_P_5
// For Servo Six
#define x_A_6
#define y_A_6
#define z_A_6
#define x_B_6
#define y_B_6
#define z_B_6
#define x_P_6
#define y_P_6
#define z_P_6


#endif // _CONFIGURATION_H
