%% Test Script to review numeric results from Arduino operations
clear all;
clc;
%% Commanded Position Vector (translational only)

INPUT = [0; 0; 0];
ROTATION = [0; 0; 0];

%% Set Constants

a = 24.0; % servo arm length, in mm
s = 122.5; % linkage arm length, in mm

SERVO_ZERO_PWM = 1500;
r = (500/90)*(360/(2*pi())); % servo PWM to angle conversion factor

B_1 = [73.5; -38.25; 0]; % location of servo shaft
P_1 = [75.0; -7.43; 0]; % location of top platform mounting point
BETA_ONE = -90; % degrees
BETA_ONE_RAD = deg2rad(BETA_ONE); % radians

B_2 = [73.5; 38.25; 0];
P_2 = [75.0; 7.43; 0];
BETA_TWO = 90;
BETA_TWO_RAD = deg2rad(BETA_TWO);

B_3 = [3.62; 82.78; 0];
P_3 = [-31.06; 68.67; 0];
BETA_THREE = 30;
BETA_THREE_RAD = deg2rad(BETA_THREE);

B_4 = [-69.88; 44.53; 0];
P_4 = [-43.94; 61.24; 0];
BETA_FOUR = 30;
BETA_FOUR_RAD = deg2rad(BETA_FOUR);

B_5 = [-69.88; -44.53; 0];
P_5 = [-43.94; -61.24; 0];
BETA_FIVE = -30;
BETA_FIVE_RAD = deg2rad(BETA_FIVE);

B_6 = [3.62; -82.78; 0];
P_6 = [-31.06; -68.67; 0];
BETA_SIX = -30;
BETA_SIX_RAD = deg2rad(BETA_SIX);

% Compute H_0 and ALPHA_0
H_0 = sqrt(s^2 + a^2 - (P_1(1)-B_1(1))^2 - (P_1(2)-B_1(2))^2);
bigL_0 = (2*a^2);
bigM_0 = 2 * a * (P_1(1)-B_1(1));
bigN_0 = 2 * a * (H_0);
denom_0 = sqrt(bigM_0^2+bigN_0^2);
ALPHA_0_RAD = asin(bigL_0/denom_0)-atan(bigM_0/bigN_0);
ALPHA_0 = rad2deg(ALPHA_0_RAD);

% Compute the rotation matrix within the platform frame

Phi = deg2rad(ROTATION(1));
Theta = deg2rad(ROTATION(2));
Psi = deg2rad(ROTATION(3));

R = [cos(Phi)*cos(Theta) -sin(Phi)*cos(Psi)+cos(Phi)*sin(Theta)*sin(Psi) sin(Phi)*sin(Psi)+cos(Phi)*sin(Theta)*cos(Psi);
    sin(Phi)*cos(Theta) cos(Phi)*cos(Psi)+sin(Phi)*sin(Theta)*sin(Psi) -cos(Phi)*sin(Psi)+sin(Phi)*sin(Theta)*cos(Psi);
    -sin(Theta) cos(Theta)*sin(Psi) cos(Theta)*cos(Psi)];

%% Compute the q_1 vector
T = INPUT + [0; 0; H_0];

q_1 = T + R * P_1;
q_2 = T + R * P_2;
q_3 = T + R * P_3;
q_4 = T + R * P_4;
q_5 = T + R * P_5;
q_6 = T + R * P_6;

%% Compute L_1 vector and magnitude l_1

L_1 = q_1 - B_1;
l_1 = norm(L_1);

L_2 = q_2 - B_2;
l_2 = norm(L_2);

L_3 = q_3 - B_3;
l_3 = norm(L_3);

L_4 = q_4 - B_4;
l_4 = norm(L_4);

L_5 = q_5 - B_5;
l_5 = norm(L_5);

L_6 = q_6 - B_6;
l_6 = norm(L_6);

%% Compute L, M, and N

bigL_1 = (l_1)^2 - (s^2 - a^2);
bigM_1 = 2 * a * (q_1(3) - B_1(3));
bigN_1 = 2 * a * ((cos(BETA_ONE_RAD)*(q_1(1)))+(sin(BETA_ONE_RAD)*(q_1(2)-B_1(2))));
denom_1 = sqrt(bigM_1^2+bigN_1^2);

bigL_2 = (l_2)^2 - (s^2 - a^2);
bigM_2 = 2 * a * (q_2(3) - B_2(3));
bigN_2 = 2 * a * ((cos(BETA_TWO_RAD)*(q_2(1)))+(sin(BETA_TWO_RAD)*(q_2(2)-B_2(2))));
denom_2 = sqrt(bigM_2^2+bigN_2^2);

bigL_3 = (l_3)^2 - (s^2 - a^2);
bigM_3 = 2 * a * (q_3(3) - B_3(3));
bigN_3 = 2 * a * ((cos(BETA_THREE_RAD)*(q_3(1)))+(sin(BETA_THREE_RAD)*(q_3(2)-B_3(2))));
denom_3 = sqrt(bigM_3^2+bigN_3^2);

bigL_4 = (l_4)^2 - (s^2 - a^2);
bigM_4 = 2 * a * (q_4(3) - B_4(3));
bigN_4 = 2 * a * ((cos(BETA_FOUR_RAD)*(q_4(1)))+(sin(BETA_FOUR_RAD)*(q_4(2)-B_4(2))));
denom_4 = sqrt(bigM_4^2+bigN_4^2);

bigL_5 = (l_5)^2 - (s^2 - a^2);
bigM_5 = 2 * a * (q_5(3) - B_5(3));
bigN_5 = 2 * a * ((cos(BETA_FIVE_RAD)*(q_5(1)))+(sin(BETA_FIVE_RAD)*(q_5(2)-B_5(2))));
denom_5 = sqrt(bigM_5^2+bigN_5^2);

bigL_6 = (l_6)^2 - (s^2 - a^2);
bigM_6 = 2 * a * (q_6(3) - B_6(3));
bigN_6 = 2 * a * ((cos(BETA_SIX_RAD)*(q_6(1)))+(sin(BETA_SIX_RAD)*(q_6(2)-B_6(2))));
denom_6 = sqrt(bigM_6^2+bigN_6^2);

%% Compute Alpha

alpha_1 = asin(bigL_1/denom_1)-atan(bigN_1/bigM_1);
alpha_deg_1 = rad2deg(alpha_1);

alpha_2 = asin(bigL_2/denom_2)-atan(bigN_2/bigM_2);
alpha_deg_2 = rad2deg(alpha_2);

alpha_3 = asin(bigL_3/denom_3)-atan(bigN_3/bigM_3);
alpha_deg_3 = rad2deg(alpha_3);

alpha_4 = asin(bigL_4/denom_4)-atan(bigN_4/bigM_4);
alpha_deg_4 = rad2deg(alpha_4);

alpha_5 = asin(bigL_5/denom_5)-atan(bigN_5/bigM_5);
alpha_deg_5 = rad2deg(alpha_5);

alpha_6 = asin(bigL_6/denom_6)-atan(bigN_6/bigM_6);
alpha_deg_6 = rad2deg(alpha_6);

%% Compute PWM Magnitude

W_1 = SERVO_ZERO_PWM + (alpha_1 - ALPHA_0_RAD) * r;
W_2 = SERVO_ZERO_PWM - (alpha_2 - ALPHA_0_RAD) * r;
W_3 = SERVO_ZERO_PWM + (alpha_3 - ALPHA_0_RAD) * r;
W_4 = SERVO_ZERO_PWM - (alpha_4 - ALPHA_0_RAD) * r;
W_5 = SERVO_ZERO_PWM + (alpha_5 - ALPHA_0_RAD) * r;
W_6 = SERVO_ZERO_PWM - (alpha_6 - ALPHA_0_RAD) * r;


%% Display the Results as a Matrix

RESULTS = [alpha_deg_1 W_1; alpha_deg_2 W_2; alpha_deg_3 W_3; alpha_deg_4 W_4; alpha_deg_5 W_5; alpha_deg_6 W_6]